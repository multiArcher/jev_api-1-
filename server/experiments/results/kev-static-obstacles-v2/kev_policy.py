"""Kev: five speed classes, waypoint lookahead and progress supervision."""
import copy
import json
import logging
import math
import time
from collections import deque
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import ProxyHandler, Request, build_opener

from choice_control import _candidate_facts
from tools.waypoint_geometry import KNOT_MPS, WaypointGeometry
from .waypoint import WaypointPolicy
from .turning import DEFAULT_MAX_TURN_RATE_DEG_S, reachable_courses
from .progress import ProgressSupervisor
from .speed_profile import plan_speed
from .static_obstacles import (StaticObstacleConfig, StaticNavigator, StaticObstacleBlocked,
                               static_targets, motion_clearance)
from tools.navigation_motion import predict_ground_motion

KEV_SPEEDS_KNOTS = (10.0, 20.0, 30.0, 40.0, 50.0)
DEFAULT_KEV_ENDPOINT = 'http://127.0.0.1:18028/v1/systemone'


def round_model_values(value):
    """Limit model input precision, without rounding internal geometry or physics."""
    if isinstance(value, float):
        result = round(value, 2)
        return 0.0 if result == 0 else result
    if isinstance(value, dict):
        return {key: round_model_values(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [round_model_values(item) for item in value]
    return value


class KevPolicy(WaypointPolicy):
    def __init__(self, endpoint=DEFAULT_KEV_ENDPOINT, timeout=120., client=None, max_turn_rate_deg_s=DEFAULT_MAX_TURN_RATE_DEG_S,
                 initial_course_deg=None, progress_config=None, obstacle_config=None, **kwargs):
        super().__init__(**kwargs)
        self.speeds = [speed for speed in KEV_SPEEDS_KNOTS if speed <= self.max_speed]
        if not self.speeds:
            raise ValueError('Kev max speed must allow at least the 10-knot speed class')
        self.progress_config = progress_config
        self.obstacle_config = obstacle_config or StaticObstacleConfig()
        if urlsplit(endpoint).scheme not in ('http', 'https'):
            raise ValueError('Kev endpoint must be an HTTP(S) URL')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Kev timeout must be positive and finite')
        if not math.isfinite(max_turn_rate_deg_s) or max_turn_rate_deg_s <= 0:
            raise ValueError('Max turn rate must be positive and finite')
        if initial_course_deg is not None and (not math.isfinite(initial_course_deg) or not 0 <= initial_course_deg < 360):
            raise ValueError('Initial course must be in [0, 360)')
        self.max_turn_rate_deg_s = max_turn_rate_deg_s
        self.initial_course_deg = initial_course_deg
        self.endpoint, self.timeout, self.client = endpoint, timeout, client

    def _observation_course(self, boat):
        """Use the current reported course even when reported speed is zero."""
        course = boat.get('targetCourse')
        if course is None and boat['targetSpeed'] < 1e-6:
            course = self.initial_course_deg
            if course is None:
                raise ValueError('Situation has no targetCourse; provide --initial-course-deg only if no course is reported')
        if (isinstance(course, bool) or not isinstance(course, (int, float))
                or not math.isfinite(course) or not 0 <= course < 360):
            raise ValueError('Missing or invalid situation targetCourse')
        return course

    def observation(self, packet):
        obs = super().observation(packet)
        course = obs['boat']['targetCourse']
        obs['execution'] = {**obs['execution'], 'max_turn_rate_deg_s': self.max_turn_rate_deg_s}
        obs['action_space'] = dict(speeds_knots=list(self.speeds), courses_deg=reachable_courses(
            course, self.duration, self.max_turn_rate_deg_s))
        geometry = WaypointGeometry(obs)
        if getattr(self.local, 'static_task_key', None) != self.local.task_key:
            self.local.static_task_key = copy.deepcopy(self.local.task_key)
            self.local.static_records = {}
        own_id = packet['params']['platCode']
        for record in packet.get('situation', ()):
            pid = record.get('platformId')
            if pid == own_id or type(pid) is not int:
                continue
            normalized = dict(record)
            if self.speed_unit == 'mps' and isinstance(normalized.get('targetSpeed'), (int, float)):
                normalized['targetSpeed'] /= KNOT_MPS
            valid = static_targets((normalized,), own_id, geometry.frame, self.obstacle_config)
            if valid:
                self.local.static_records[pid] = normalized
            else:
                # A target now reported moving must not stay frozen in a
                # historical static position. Dynamic prediction is separate.
                self.local.static_records.pop(pid, None)
        obs['static_obstacles'] = static_targets(self.local.static_records.values(), own_id,
                                                geometry.frame, self.obstacle_config)
        return obs

    def _request(self, obs):
        geometry = WaypointGeometry(obs)
        task, boat = obs['task'], obs['boat']
        index = task['next_waypoint_index']
        navigation = obs.get('static_navigation')
        target_position = navigation['guidance_xy_m'] if navigation else geometry.waypoint_positions[index]
        target_offset = tuple(target_position[i] - geometry.boat_position[i] for i in (0, 1))
        distance = math.hypot(*target_offset)
        limit = task['planWaypointList'][index]['routeSpeed']
        course = boat['targetCourse']
        bearing = math.degrees(math.atan2(target_offset[0], target_offset[1])) % 360
        relative_bearing = (bearing-course+180)%360-180
        target_side = 'ahead' if abs(relative_bearing) < .005 else ('left' if relative_bearing < 0 else 'right')
        progress = obs.get('navigation_progress', dict(mode='normal', recovery_cycles=0))
        profile = plan_speed(obs, geometry, self.speeds, self.max_turn_rate_deg_s,
                             recovering=progress['mode'] == 'recovery')
        speeds = profile['speeds_knots']
        if navigation and (navigation['slow'] or abs(relative_bearing) > 15):
            speeds = [min(self.speeds)]
            profile = dict(profile, phase='static_avoidance', speeds_knots=speeds,
                           speed_cap_knots=max(speeds))
        self.local.turn_diagnostics = dict(observed_next_turn_deg=profile['observed_next_turn_deg'])
        state = dict(
            task='Visit waypoint circles in order.',
            coordinate_system='Metres, east/north; origin=first waypoint. Course: 0=N, 90=E, clockwise. Relative: -=left, +=right.',
            speeds_knots=speeds, progress=progress,
            speed_profile={key: value for key, value in profile.items()
                           if key not in ('speeds_knots', 'observed_next_turn_deg')},
            actual_speed_knots=boat['targetSpeed'], actual_course_deg=course,
            current_waypoint_index=index,
            boat_xy_m=list(geometry.boat_position), target_xy_m=list(target_position),
            target_offset_m=list(target_offset),
            target_bearing_deg=bearing,
            target_relative_bearing_deg=relative_bearing, target_turn_side=target_side,
            distance_to_target_m=distance, arrival_radius_m=10. if navigation and navigation['detour'] else task['arrival_radius_m'],
            waypoint_speed_limit_knots=limit / KNOT_MPS,
            command_duration_s=self.duration,
            max_turn_deg=min(self.max_turn_rate_deg_s*self.duration, 179.99),
            recent_decisions=list(getattr(self.local, 'history', ())))
        if navigation:
            state['static_avoidance'] = dict(waypoint_xy_m=list(geometry.waypoint_positions[index]),
                waypoint_arrival_radius_m=task['arrival_radius_m'],
                collision_radius_m=self.obstacle_config.collision_radius_m,
                nearby_targets=len(navigation['obstacles']), detour=navigation['detour'])
        previous = index - 1
        if previous < 0 or math.hypot(*geometry.waypoints[previous]) > task['arrival_radius_m']:
            previous = None
        criteria = {}
        for speed in speeds:
            for j, target_course in enumerate(obs['action_space']['courses_deg']):
                clearance = None
                if navigation:
                    clearance = motion_clearance(geometry, boat, speed, target_course,
                        obs['static_obstacles'], obs['execution'], self.duration, self.obstacle_config)
                    if not clearance['safe']:
                        continue
                name = f'speed_{speed:g}__heading_{j:02d}'
                facts = _candidate_facts(obs, geometry, speed*KNOT_MPS,
                    target_course, f'{target_course:.2f} deg', distance, limit, previous)['approximate_effect']
                signed_turn = (target_course-course+180)%360-180
                criteria[name] = dict(speed_knots=speed, absolute_course_deg=target_course,
                    maneuver='straight' if abs(signed_turn) < .005 else ('left' if signed_turn < 0 else 'right'),
                    signed_turn_deg=signed_turn,
                    target_heading_error_deg=abs((bearing-target_course+180)%360-180),
                    distance_after_m=facts['end_checkpoint_distance_m'],
                    distance_after_2_steps_m=facts['same_command_preview']['end_checkpoint_distance_m'],
                    speed_limit_violation=(facts['passing_speed_above_ceiling'] or facts.get('previous_circle_speed_above_ceiling', False)),
                    approach_speed_warning=facts['approach_margin_speed_above_ceiling'],
                    # Preserve this replay-validated model-input order.
                    turn_deg=abs(signed_turn))
                if navigation:
                    # Preserve the validated absolute-turn field at the end.
                    turn = criteria[name].pop('turn_deg')
                    criteria[name]['obstacle_clearance_m'] = clearance['min_clearance_m']
                    criteria[name]['turn_deg'] = turn
                    for steps, field in ((1, 'distance_after_m'), (2, 'distance_after_2_steps_m')):
                        motion = predict_ground_motion(boat['targetSpeed']*KNOT_MPS, course,
                            speed*KNOT_MPS, target_course, obs['execution']['speed_transition_time_s'],
                            obs['execution']['course_transition_time_s'], self.duration*steps,
                            max_turn_rate_deg_s=self.max_turn_rate_deg_s)
                        criteria[name][field] = math.dist(motion['endpoint'], target_offset)
        if not criteria:
            raise StaticObstacleBlocked('当前可达速度和航向均无法满足静态障碍物避碰及制动距离')
        instructions = (
            'Reach the current waypoint circle. Respect speed limits. Choose a maneuver on target_turn_side; '
            'straight when ahead. Prefer smaller target heading error, '
            'then smaller forecast distance and less turning. Keep straight when aligned. '
            'A target behind the boat may require turning while distance initially increases; avoid repeated reversals.')
        if navigation:
            instructions = ('Follow the safe guidance target around static obstacles. '
                            'Original waypoint arrival is checked externally. ' + instructions)
        return round_model_values(dict(model='kev-latest', state=state, questions=dict(action=dict(
            type='choice', instructions=instructions, criteria=criteria))))

    def _call(self, payload):
        if self.client is not None:
            return self.client(payload)
        if not hasattr(self.local, 'opener'):
            self.local.opener = build_opener(ProxyHandler({}))
        request = Request(self.endpoint, data=json.dumps(payload,allow_nan=False).encode(),
                          headers={'Content-Type':'application/json'}, method='POST')
        try:
            with self.local.opener.open(request, timeout=self.timeout) as response:
                return json.load(response)
        except HTTPError as exc:
            raise RuntimeError(f'Kev HTTP {exc.code}: {exc.read(2048).decode(errors="replace")}') from exc
        except (URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
            raise RuntimeError(f'Local Kev request failed ({self.endpoint}): {exc}') from exc

    def __call__(self, packet):
        obs = self.observation(packet)
        if obs['task']['route_complete']:
            return 0., obs['boat']['targetCourse']
        if getattr(self.local,'kev_task_key',None) != self.local.task_key:
            self.local.kev_task_key = copy.deepcopy(self.local.task_key)
            self.local.supervisor = ProgressSupervisor(self.progress_config)
            self.local.navigator = StaticNavigator(self.obstacle_config)
            self.local.static_mode = None
            self.local.kev_waypoint = None
        index = obs['task']['next_waypoint_index']
        if self.local.kev_waypoint != index:
            self.local.kev_waypoint = index
            self.local.history = deque(maxlen=4)
        geometry = WaypointGeometry(obs)
        navigation = self.local.navigator.guide(geometry, obs['static_obstacles'])
        if navigation:
            obs['static_navigation'] = navigation
        static_mode = (index, navigation['detour'], navigation['guidance_xy_m']) if navigation else None
        if static_mode != self.local.static_mode:
            logging.getLogger(__name__).info('Kev static navigation waypoint=%d targets=%d detour=%s guidance=%s',
                index, len(obs['static_obstacles']), bool(navigation and navigation['detour']),
                navigation['guidance_xy_m'] if navigation else None)
            self.local.static_mode = static_mode
        target_position = navigation['guidance_xy_m'] if navigation else geometry.waypoint_positions[index]
        target_offset = tuple(target_position[i]-geometry.boat_position[i] for i in (0, 1))
        distance = math.hypot(*target_offset)
        bearing = math.degrees(math.atan2(target_offset[0], target_offset[1])) % 360
        error = abs((bearing - obs['boat']['targetCourse'] + 180) % 360 - 180)
        previous_mode = self.local.supervisor.mode
        progress_key = (index, tuple(round(v, 2) for v in target_position)) if navigation else index
        obs['navigation_progress'] = self.local.supervisor.update(progress_key, distance, error)
        if previous_mode != self.local.supervisor.mode:
            logging.getLogger(__name__).info('Kev navigation mode=%s waypoint=%d distance=%.1fm heading_error=%.1fdeg',
                self.local.supervisor.mode, index, distance, error)
        payload = self._request(obs)
        started = time.perf_counter()
        # Openers and navigation state are thread-local. A superseded task's
        # slow HTTP request must not hold a client-side lock on the new task.
        result = self._call(payload)
        if isinstance(result,dict) and result.get('truncated'):
            raise RuntimeError('Kev truncated the model input; refusing a partial-state decision')
        answers = result.get('answers') if isinstance(result,dict) else None
        answer = answers.get('action') if isinstance(answers,dict) else None
        if not isinstance(answer,dict) or answer.get('type') != 'choice':
            raise ValueError('Expected Kev Choice answer for action')
        choice = answer.get('choice')
        criteria = payload['questions']['action']['criteria']
        if not isinstance(choice,str) or choice not in criteria:
            raise ValueError('Kev returned a choice outside the available speed/course candidates')
        course = criteria[choice]['absolute_course_deg']
        speed = criteria[choice]['speed_knots']
        self.local.last_decision = dict(choice=choice, candidate_count=len(criteria), selection_mode='single_choice',
            speeds_knots=payload['state']['speeds_knots'], inference_s=time.perf_counter()-started,
            state=payload['state'], options=payload['questions']['action']['criteria'],
            instructions=payload['questions']['action']['instructions'], result=result,
            turn_diagnostics=dict(self.local.turn_diagnostics))
        if navigation:
            self.local.last_decision['static_path_xy_m'] = navigation['path']
        self.local.history.append(dict(waypoint_index=obs['task']['next_waypoint_index'],
            distance_m=round(distance, 2), chosen_speed_knots=speed, chosen_course_deg=course))
        logging.getLogger(__name__).debug('Kev inference_s=%.6f choice=%s',
            self.local.last_decision['inference_s'],choice)
        return speed*(KNOT_MPS if self.speed_unit=='mps' else 1.),course

    def validate_control(self, task, own, records, speed, course):
        """Recheck even held commands against telemetry newer than inference."""
        from types import SimpleNamespace
        from tools.waypoint_geometry import LocalFrame
        frame = LocalFrame(task['params']['planWaypointList'])
        boat = dict(own)
        if self.speed_unit == 'mps':
            boat['targetSpeed'] /= KNOT_MPS
            speed /= KNOT_MPS
            records = [dict(r, targetSpeed=r['targetSpeed']/KNOT_MPS)
                       if isinstance(r.get('targetSpeed'), (int, float)) else r for r in records]
        obstacles = static_targets(records, task['params']['platCode'], frame, self.obstacle_config)
        geometry = SimpleNamespace(boat_position=frame.project(own['longitude'], own['latitude']))
        execution = dict(self.execution, max_turn_rate_deg_s=self.max_turn_rate_deg_s)
        return motion_clearance(geometry, boat, speed, course, obstacles, execution,
                                self.duration, self.obstacle_config)
