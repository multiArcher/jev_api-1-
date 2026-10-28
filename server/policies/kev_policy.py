"""Kev: five speed classes, waypoint lookahead and progress supervision."""
import copy
import json
import logging
import math
import time
from collections import deque
from dataclasses import replace
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
                               static_targets, motion_clearance, ActualDistanceMonitor,
                               ACTUAL_DISTANCE_LOGGER, clear_segment)
from tools.navigation_motion import predict_ground_motion
if __package__.startswith('server.'):
    from ..navigation.route_rejoin import RouteRejoin, RouteRejoinConfig, refine_courses
    from ..navigation.dynamic_avoidance import AvoidanceConfig
    from ..navigation.dynamic_avoidance.kev import (
        DynamicAvoidanceHold, KevDynamicAvoidance, moving_scene, control_check, wire_action)
else:
    from navigation.route_rejoin import RouteRejoin, RouteRejoinConfig, refine_courses
    from navigation.dynamic_avoidance import AvoidanceConfig
    from navigation.dynamic_avoidance.kev import (
        DynamicAvoidanceHold, KevDynamicAvoidance, moving_scene, control_check, wire_action)

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
                 initial_course_deg=None, progress_config=None, obstacle_config=None,
                 dynamic_config=None, route_rejoin_config=None, **kwargs):
        super().__init__(**kwargs)
        self.speeds = [speed for speed in KEV_SPEEDS_KNOTS if speed <= self.max_speed]
        if not self.speeds:
            raise ValueError('Kev max speed must allow at least the 10-knot speed class')
        self.progress_config = progress_config
        self.obstacle_config = obstacle_config or StaticObstacleConfig()
        self.route_rejoin_config = route_rejoin_config or RouteRejoinConfig()
        if urlsplit(endpoint).scheme not in ('http', 'https'):
            raise ValueError('Kev endpoint must be an HTTP(S) URL')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Kev timeout must be positive and finite')
        if not math.isfinite(max_turn_rate_deg_s) or max_turn_rate_deg_s <= 0:
            raise ValueError('Max turn rate must be positive and finite')
        if initial_course_deg is not None and (not math.isfinite(initial_course_deg) or not 0 <= initial_course_deg < 360):
            raise ValueError('Initial course must be in [0, 360)')
        self.max_turn_rate_deg_s = max_turn_rate_deg_s
        static = self.obstacle_config
        self.dynamic_config = replace(dynamic_config or AvoidanceConfig(
            obstacle_radius_m=static.obstacle_radius_m, vessel_length_m=static.vessel_length_m,
            vessel_width_m=static.vessel_width_m, safety_margin_m=static.safety_margin_m,
            acceleration_mps2=static.acceleration_mps2, deceleration_mps2=static.deceleration_mps2),
            max_turn_rate_deg_s=max_turn_rate_deg_s, planning_interval_s=self.duration)
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
        if getattr(self.local, 'route_task_key', None) != self.local.task_key:
            self.local.route_task_key = copy.deepcopy(self.local.task_key)
            self.local.route_rejoin = RouteRejoin(geometry.boat_position, self.route_rejoin_config)
            self.local.route_mode = None
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
        if getattr(self.local, 'dynamic_task_key', None) != self.local.task_key:
            self.local.dynamic_task_key = copy.deepcopy(self.local.task_key)
            self.local.dynamic_guard = KevDynamicAvoidance(self.dynamic_config,
                self.execution['speed_transition_time_s'], self.speed_unit)
            self.local.dynamic_status = 'clear'
            self.local.dynamic_warning = None
            self.local.dynamic_warning_targets = ()
        raw_own = next(r for r in packet['situation'] if r['platformId'] == own_id)
        obs['dynamic_scene'] = self._moving_scene(packet, dict(raw_own,targetCourse=course), packet['situation'])
        return obs

    def _record_dynamic_warning(self,warning):
        """Long-window warnings stay out of the model payload and control veto."""
        targets=tuple(warning['threat_ids']) if warning else ()
        previous=getattr(self.local,'dynamic_warning_targets',())
        self.local.dynamic_warning=warning
        self.local.dynamic_warning_targets=targets
        if targets!=previous:
            if targets:
                logging.getLogger(__name__).warning(
                    'Kev dynamic early warning targets=%s warning_horizon_s=%s action_horizon_s=%s; warning does not veto actions',
                    targets,self.dynamic_config.warning_horizon_s,self.dynamic_config.action_horizon_s)
            elif previous:
                logging.getLogger(__name__).info('Kev dynamic early warning cleared targets=%s',previous)

    def _moving_scene(self, task, own, records):
        waypoint = task['params']['planWaypointList'][0]
        if (own.get('targetCourse') is None and self.initial_course_deg is not None
                and own.get('targetSpeed')==0):
            own=dict(own,targetCourse=self.initial_course_deg)
        return moving_scene(records, own, own_id=task['params']['platCode'],
            origin_lon_lat=(waypoint['routeLon'],waypoint['routeLat']), speed_unit=self.speed_unit,
            stationary_speed_knots=self.obstacle_config.stationary_speed_knots)

    def _candidate(self,obs,geometry,speed,target_course,distance,limit,previous,target_offset,*,force_static=False):
        """Build one ordinary Kev candidate; dynamic expansion reuses this guard."""
        navigation=obs.get('static_navigation')
        route=obs.get('route_navigation')
        boat=obs['boat'];course=boat['targetCourse']
        clearance=None
        if navigation or route or force_static:
            clearance=motion_clearance(geometry,boat,speed,target_course,
                obs['static_obstacles'],obs['execution'],self.duration,self.obstacle_config)
            if not clearance['safe']:
                return None
        facts=_candidate_facts(obs,geometry,speed*KNOT_MPS,target_course,
            f'{target_course:.2f} deg',distance,limit,previous)['approximate_effect']
        signed_turn=(target_course-course+180)%360-180
        item=dict(speed_knots=speed,absolute_course_deg=target_course,
            maneuver='straight' if abs(signed_turn)<.005 else ('left' if signed_turn<0 else 'right'),
            signed_turn_deg=signed_turn,
            target_heading_error_deg=abs((math.degrees(math.atan2(target_offset[0],target_offset[1]))-target_course+180)%360-180),
            distance_after_m=facts['end_checkpoint_distance_m'],
            distance_after_2_steps_m=facts['same_command_preview']['end_checkpoint_distance_m'],
            speed_limit_violation=(facts['passing_speed_above_ceiling'] or facts.get('previous_circle_speed_above_ceiling',False)),
            approach_speed_warning=facts['approach_margin_speed_above_ceiling'],turn_deg=abs(signed_turn))
        if navigation or route:
            turn=item.pop('turn_deg')
            item['obstacle_clearance_m']=clearance['min_clearance_m']
            item['turn_deg']=turn
            for steps,field in ((1,'distance_after_m'),(2,'distance_after_2_steps_m')):
                motion=predict_ground_motion(boat['targetSpeed']*KNOT_MPS,course,
                    speed*KNOT_MPS,target_course,obs['execution']['speed_transition_time_s'],
                    obs['execution']['course_transition_time_s'],self.duration*steps,
                    max_turn_rate_deg_s=self.max_turn_rate_deg_s)
                item[field]=math.dist(motion['endpoint'],target_offset)
                if route:
                    start = route['segment_start_xy_m']
                    end = route['segment_end_xy_m']
                    length = math.dist(start, end)
                    offset = tuple(geometry.boat_position[i] + motion['endpoint'][i] - start[i]
                                   for i in (0, 1))
                    cross = abs((end[0]-start[0])*offset[1] - (end[1]-start[1])*offset[0])/length
                    item['route_cross_track_after_m'] = max(item.get('route_cross_track_after_m', 0.), cross)
            if route:
                item['route_heading_error_deg'] = abs((target_course-route['segment_course_deg']+180)%360-180)
                # Preserve the established final-field convention for Kev.
                item['turn_deg'] = item.pop('turn_deg')
        return item

    @staticmethod
    def _guidance_target(obs, geometry):
        navigation = obs.get('static_navigation')
        if navigation and navigation['detour']:
            return navigation['guidance_xy_m']
        if obs.get('route_navigation'):
            return obs['route_navigation']['guidance_xy_m']
        return navigation['guidance_xy_m'] if navigation else geometry.waypoint_positions[geometry.next_index]

    def _request(self, obs):
        geometry = WaypointGeometry(obs)
        task, boat = obs['task'], obs['boat']
        index = task['next_waypoint_index']
        navigation = obs.get('static_navigation')
        route = obs.get('route_navigation')
        target_position = self._guidance_target(obs, geometry)
        target_offset = tuple(target_position[i] - geometry.boat_position[i] for i in (0, 1))
        distance = math.hypot(*target_offset)
        limit = task['planWaypointList'][index]['routeSpeed']
        course = boat['targetCourse']
        bearing = math.degrees(math.atan2(target_offset[0], target_offset[1])) % 360
        relative_bearing = (bearing-course+180)%360-180
        target_side = 'ahead' if abs(relative_bearing) < .005 else ('left' if relative_bearing < 0 else 'right')
        progress = obs.get('navigation_progress', dict(mode='normal', recovery_cycles=0))
        profile = plan_speed(obs, geometry, self.speeds, self.max_turn_rate_deg_s,
                             recovering=progress['mode'] == 'recovery',
                             apply_heading_cap=not bool(navigation))
        speeds = profile['speeds_knots']
        if navigation and navigation['detour'] and not navigation.get('approaching', False):
            # Half the nominal stage cap, never half the already reduced
            # measured speed: repeated decisions must not keep halving it.
            half_cap=max(speeds)/2
            speeds=[half_cap]
            profile=dict(profile,phase='static_half_speed',speeds_knots=speeds,
                         speed_cap_knots=half_cap)
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
            distance_to_target_m=distance, arrival_radius_m=10. if route or (navigation and navigation['detour']) else task['arrival_radius_m'],
            waypoint_speed_limit_knots=limit / KNOT_MPS,
            command_duration_s=self.duration,
            max_turn_deg=min(self.max_turn_rate_deg_s*self.duration, 179.99),
            recent_decisions=list(getattr(self.local, 'history', ())))
        if navigation:
            state['static_avoidance'] = dict(waypoint_xy_m=list(geometry.waypoint_positions[index]),
                waypoint_arrival_radius_m=task['arrival_radius_m'],
                collision_radius_m=self.obstacle_config.collision_radius_m,
                nearby_targets=len(navigation['obstacles']), detour=navigation['detour'])
        if obs.get('route_rejoin'):
            state['route_rejoin'] = obs['route_rejoin']
        previous = index - 1
        if previous < 0 or math.hypot(*geometry.waypoints[previous]) > task['arrival_radius_m']:
            previous = None
        criteria = {}
        # Include the exact half-cap (e.g. 25 knots), even when it is not one
        # of the five nominal classes. Slower classes remain safety fallbacks.
        candidate_speeds = sorted({s for s in self.speeds if s <= max(speeds)} | set(speeds)) if navigation or route else speeds
        for speed in candidate_speeds:
            for j, target_course in enumerate(obs['action_space']['courses_deg']):
                item=self._candidate(obs,geometry,speed,target_course,distance,limit,previous,target_offset)
                if item is not None:
                    criteria[f'speed_{speed:g}__heading_{j:02d}']=item
        if (navigation or route) and criteria:
            fastest=max(item['speed_knots'] for item in criteria.values())
            criteria={key:item for key,item in criteria.items() if item['speed_knots']==fastest}
            if navigation and navigation.get('approaching'):
                aligned=min(item['target_heading_error_deg'] for item in criteria.values())
                criteria={key:item for key,item in criteria.items()
                          if item['target_heading_error_deg'] <= aligned+1e-6}
            state['speeds_knots']=[fastest]
            state['speed_profile']['speed_cap_knots']=fastest
            if fastest < max(speeds):
                state['speed_profile']['phase']='static_safety_fallback'
        aligned = {}
        if route and route['mode'] == 'following':
            # Once within tolerance, prefer continuing along the line to chasing
            # tiny cross-track errors with a sharp turn when the boat slows down.
            aligned = {key: item for key, item in criteria.items()
                       if item['route_heading_error_deg'] <= self.route_rejoin_config.heading_tolerance_deg
                       and item['route_cross_track_after_m'] <= self.route_rejoin_config.cross_track_tolerance_m}
            if aligned:
                error = min(item['route_heading_error_deg'] for item in aligned.values())
                criteria = {key: item for key, item in aligned.items()
                            if item['route_heading_error_deg'] <= error + 1e-6}
        if route and criteria and (route['mode'] == 'rejoining' or not aligned):
            # Route recovery is an explicit preference, not merely a prompt:
            # do not let an otherwise safe shortcut postpone convergence.
            error = min(item['target_heading_error_deg'] for item in criteria.values())
            criteria = {key: item for key, item in criteria.items()
                        if item['target_heading_error_deg'] <= error + 1e-6}
        if not criteria:
            raise StaticObstacleBlocked('当前可达速度和航向均无法满足静态障碍物避碰及制动距离')
        dynamic=None
        if obs.get('dynamic_scene') is not None:
            cap=max(item['speed_knots'] for item in criteria.values())
            def expand(extra_speeds):
                expanded={}
                for speed in sorted(set(extra_speeds)|{s for s in candidate_speeds if s<=cap}):
                    for j,target_course in enumerate(obs['action_space']['courses_deg']):
                        action=wire_action(speed,target_course,self.speed_unit)
                        item=self._candidate(obs,geometry,action.speed_mps/KNOT_MPS,action.course_deg,
                            distance,limit,previous,target_offset,force_static=True)
                        if item is not None:
                            # Retain the nominal speed, whose wire value was checked above.
                            item['speed_knots']=speed
                            expanded[f'speed_{speed:g}__heading_{j:02d}']=item
                return expanded
            try:
                criteria,dynamic=self.local.dynamic_guard.filter(obs['dynamic_scene'],criteria,expand=expand)
            finally:
                self._record_dynamic_warning(self.local.dynamic_guard.last_warning)
        else:
            self.local.dynamic_guard.planner.reset()
            self._record_dynamic_warning(None)
        self.local.dynamic_status=dynamic['status'] if dynamic else 'clear'
        self.local.dynamic_details=dynamic
        if dynamic:
            self.local.route_rejoin.activate()
            if self.local.route_rejoin.active:
                state['route_rejoin'] = dict(state.get('route_rejoin', {}),
                    waypoint_index=index, mode='avoiding', stable_cycles=0)
            state['dynamic_avoidance']={key:dynamic[key] for key in ('status','targets','threat_ids','horizon_s')}
            state['speeds_knots']=sorted({item['speed_knots'] for item in criteria.values()})
            state['speed_profile']['speed_cap_knots']=max(state['speeds_knots'])
        instructions = (
            'Reach the current waypoint circle. Respect speed limits. Choose a maneuver on target_turn_side; '
            'straight when ahead. Prefer smaller target heading error, '
            'then smaller forecast distance and less turning. Keep straight when aligned. '
            'A target behind the boat may require turning while distance initially increases; avoid repeated reversals.')
        if navigation:
            instructions = ('Follow the safe guidance target around static obstacles. '
                            'Original waypoint arrival is checked externally. ' + instructions)
        if route:
            instructions = ('Follow the forward guidance target to rejoin the original route segment, '
                            'then follow that line. Original waypoint arrival is checked externally. ' + instructions)
        if dynamic and dynamic['status']=='avoiding':
            instructions=('Follow the safe dynamic avoidance side; slow when necessary. '
                          'Static guidance remains the route reference. '+instructions)
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
        if self.local.dynamic_status in ('avoiding','blocked','insufficient_data'):
            self.local.supervisor=ProgressSupervisor(self.progress_config)
        approach_profile = plan_speed(obs, geometry, self.speeds, self.max_turn_rate_deg_s,
            recovering=self.local.supervisor.mode == 'recovery')
        navigation = self.local.navigator.guide(geometry, obs['static_obstacles'],
            boat=obs['boat'], execution=obs['execution'], duration=self.duration,
            speed_knots=max(approach_profile['speeds_knots']))
        if navigation:
            obs['static_navigation'] = navigation
        static_detour = bool(navigation and navigation['detour'])
        if static_detour and not navigation.get('approaching', False):
            self.local.route_rejoin.activate()
        route = self.local.route_rejoin.guide(geometry, obs['boat']['targetSpeed'] * KNOT_MPS,
            obs['boat']['targetCourse'],
            lambda a, b: clear_segment(a, b, obs['static_obstacles'], self.obstacle_config.path_padding_m),
            avoiding=static_detour)
        if route:
            obs['route_rejoin'] = route
            if route['guidance_xy_m'] is not None:
                obs['route_navigation'] = route
                offset = tuple(route['guidance_xy_m'][i]-geometry.boat_position[i] for i in (0, 1))
                bearing = math.degrees(math.atan2(offset[0], offset[1])) % 360
                obs['action_space']['courses_deg'] = refine_courses(obs['action_space']['courses_deg'],
                    obs['boat']['targetCourse'], self.max_turn_rate_deg_s*self.duration,
                    (route['segment_course_deg'], bearing))
        route_mode = (index, self.local.route_rejoin.mode)
        if route_mode != self.local.route_mode:
            logging.getLogger(__name__).info('Kev route rejoin waypoint=%d mode=%s cross_track_m=%s',
                index, self.local.route_rejoin.mode, route.get('cross_track_m') if route else None)
            self.local.route_mode = route_mode
        static_mode = (index, navigation['detour'], navigation['guidance_xy_m']) if navigation else None
        if static_mode != self.local.static_mode:
            logging.getLogger(__name__).info('Kev static navigation waypoint=%d targets=%d detour=%s guidance=%s',
                index, len(obs['static_obstacles']), bool(navigation and navigation['detour']),
                navigation['guidance_xy_m'] if navigation else None)
            self.local.static_mode = static_mode
        target_position = self._guidance_target(obs, geometry)
        target_offset = tuple(target_position[i]-geometry.boat_position[i] for i in (0, 1))
        distance = math.hypot(*target_offset)
        bearing = math.degrees(math.atan2(target_offset[0], target_offset[1])) % 360
        error = abs((bearing - obs['boat']['targetCourse'] + 180) % 360 - 180)
        previous_mode = self.local.supervisor.mode
        if obs.get('dynamic_scene') is not None:
            # Check the prospective nominal action before the progress update:
            # a newly arriving threat must not turn yielding into a stall.
            speed_profile=plan_speed(obs,geometry,self.speeds,self.max_turn_rate_deg_s,
                apply_heading_cap=not bool(navigation))
            nominal=max(speed_profile['speeds_knots'])
            if navigation and navigation['detour'] and not navigation.get('approaching',False):
                nominal/=2
            heading=min(obs['action_space']['courses_deg'],key=lambda c:abs((bearing-c+180)%360-180))
            if self.local.dynamic_guard.needs_avoidance(obs['dynamic_scene'],
                    wire_action(nominal,heading,self.speed_unit)):
                self.local.supervisor=ProgressSupervisor(self.progress_config)
        progress_key = (index, tuple(round(v, 2) for v in target_position)) if navigation else index
        progress_distance = distance
        if obs.get('route_navigation'):
            progress_key = (index, 'route')
            progress_distance = route['progress_distance_m']
        obs['navigation_progress'] = self.local.supervisor.update(progress_key, progress_distance, error,
            require_distance_progress=bool(obs.get('route_navigation')))
        if previous_mode != self.local.supervisor.mode:
            logging.getLogger(__name__).info('Kev navigation mode=%s waypoint=%d distance=%.1fm heading_error=%.1fdeg',
                self.local.supervisor.mode, index, distance, error)
        try:
            payload = self._request(obs)
        except DynamicAvoidanceHold as exc:
            old_status=self.local.dynamic_status
            self.local.dynamic_status=exc.details['status']
            self.local.route_rejoin.activate()
            self.local.last_decision=dict(selection_mode='dynamic_hold',candidate_count=0,options={},
                state=dict(dynamic_avoidance=exc.details),dynamic_avoidance=exc.details)
            if self.local.route_rejoin.active:
                self.local.last_decision['state']['route_rejoin'] = dict(obs.get('route_rejoin') or {},
                    waypoint_index=index, mode='avoiding', stable_cycles=0)
            if self.local.dynamic_warning:
                self.local.last_decision['dynamic_warning']=self.local.dynamic_warning
            if navigation:
                self.local.last_decision['static_path_xy_m']=navigation['path']
            if old_status!=self.local.dynamic_status:
                logging.getLogger(__name__).warning('Kev dynamic wait status=%s targets=%s',
                    self.local.dynamic_status,exc.details['threat_ids'])
            return 0.,obs['boat']['targetCourse']
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
        if self.local.dynamic_details:
            self.local.last_decision['dynamic_avoidance']=self.local.dynamic_details
        if self.local.dynamic_warning:
            self.local.last_decision['dynamic_warning']=self.local.dynamic_warning
        self.local.history.append(dict(waypoint_index=obs['task']['next_waypoint_index'],
            distance_m=round(distance, 2), chosen_speed_knots=speed, chosen_course_deg=course))
        logging.getLogger(__name__).debug('Kev inference_s=%.6f choice=%s',
            self.local.last_decision['inference_s'],choice)
        return speed*(KNOT_MPS if self.speed_unit=='mps' else 1.),course

    def _static_scene(self, task, own, records):
        from types import SimpleNamespace
        from tools.waypoint_geometry import LocalFrame
        frame = LocalFrame(task['params']['planWaypointList'])
        boat = dict(own)
        if self.speed_unit == 'mps':
            boat['targetSpeed'] /= KNOT_MPS
            records = [dict(r, targetSpeed=r['targetSpeed']/KNOT_MPS)
                       if isinstance(r.get('targetSpeed'), (int, float)) else r for r in records]
        obstacles = static_targets(records, task['params']['platCode'], frame, self.obstacle_config)
        geometry = SimpleNamespace(boat_position=frame.project(own['longitude'], own['latitude']))
        return geometry, boat, obstacles

    def validate_position(self, task, own, records):
        """Never count an arrival inside an already occupied collision region."""
        geometry, _, obstacles = self._static_scene(task, own, records)
        for obstacle in obstacles:
            if math.dist(geometry.boat_position, obstacle['xy']) <= obstacle['radius_m']:
                raise StaticObstacleBlocked(f"本艇已处于静态目标{obstacle['id']}的避碰范围内")

    def create_distance_monitor(self, logger, task, max_gap_s):
        return ActualDistanceMonitor(ACTUAL_DISTANCE_LOGGER, task, max_gap_s)

    def record_actual_distance(self, monitor, task, own, records):
        geometry, _, obstacles = self._static_scene(task, own, records)
        monitor.update(geometry.boat_position, obstacles)

    def validate_control(self, task, own, records, speed, course):
        """Recheck even held commands against telemetry newer than inference."""
        geometry, boat, obstacles = self._static_scene(task, own, records)
        if self.speed_unit == 'mps':
            speed /= KNOT_MPS
        execution = dict(self.execution, max_turn_rate_deg_s=self.max_turn_rate_deg_s)
        return motion_clearance(geometry, boat, speed, course, obstacles, execution,
                                self.duration, self.obstacle_config)

    def validate_dynamic_control(self, task, own, records, speed, course):
        """Separate stateless latest-snapshot check; includes zero-speed commands."""
        try:
            scene=self._moving_scene(task,own,records)
        except ValueError as exc:
            return dict(safe=False,min_clearance_m=None,obstacle_id=None,
                        avoidance_type='dynamic',reason=str(exc))
        speed_knots=speed/KNOT_MPS if self.speed_unit=='mps' else speed
        return control_check(scene,speed_knots,course,self.dynamic_config,
            self.execution['speed_transition_time_s'],self.speed_unit)
