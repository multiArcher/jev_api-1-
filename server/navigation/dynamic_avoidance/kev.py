"""Kev integration: snapshot conversion and safe model-candidate filtering.

Only moving or uncertain targets enter the dynamic forecast. Static checks
are supplied by Kev as a separate candidate-builder callback.
"""
from dataclasses import asdict, replace
import math
from tools.waypoint_geometry import KNOT_MPS
from .adapter import scene_from_situation
from .encounter import encounter_window
from .planner import DynamicAvoidancePlanner
from .prediction import assess_action, signed_turn
from .types import Action, finite


class DynamicAvoidanceHold(Exception):
    """Temporary safety wait, not an inference failure or task termination."""
    def __init__(self, details):
        super().__init__('动态避障暂时没有可确认安全的动作')
        self.details = details


def moving_scene(records, own, *, own_id, origin_lon_lat, speed_unit, stationary_speed_knots):
    """Keep unknown motion data; leave valid stationary records to static guard."""
    factor = KNOT_MPS if speed_unit == 'knots' else 1.
    latest = {r.get('platformId'): r for r in records}
    selected = []
    for pid, record in latest.items():
        if pid == own_id:
            continue
        speed = record.get('targetSpeed')
        lon, lat = record.get('longitude'), record.get('latitude')
        if (finite(speed) and abs(speed)*factor <= stationary_speed_knots*KNOT_MPS
                and finite(lon) and finite(lat) and abs(lon) <= 180 and abs(lat) < 90):
            continue
        selected.append(record)
    if not selected:
        return None
    return scene_from_situation([*selected, dict(own, platformId=own_id)],
        origin_lon_lat=origin_lon_lat, speed_unit=speed_unit, own_id=own_id)


def wire_action(speed_knots, course, speed_unit='knots'):
    """Assess the actual two-decimal speed/course sent on the control wire."""
    speed = round(speed_knots*KNOT_MPS if speed_unit == 'mps' else speed_knots, 2)
    return Action(speed if speed_unit == 'mps' else speed*KNOT_MPS, round(course, 2))


def check_models(scene, action, config, speed_transition_s, *, horizon_s=None):
    """Check nominal measured response and physical capability where calibrated."""
    own, targets = scene
    nominal=assess_action(own,targets,action,config,horizon_s=horizon_s)
    slower=min(config.deceleration_mps2,config.controller_deceleration_mps2)
    if slower>=config.deceleration_mps2:
        return nominal
    conservative=assess_action(own,targets,action,replace(config,deceleration_mps2=slower,
        deceleration_curve=tuple((v,min(a,slower)) for v,a in config.deceleration_curve)),horizon_s=horizon_s)
    risks={risk.target_id:risk for risk in nominal.risks}
    for risk in conservative.risks:
        if risk.target_id not in risks or risk.min_clearance_m<risks[risk.target_id].min_clearance_m:
            risks[risk.target_id]=risk
    ordered=tuple(risks[k] for k in sorted(risks))
    return replace(nominal,safe=nominal.safe and conservative.safe,risks=ordered,
        min_clearance_m=min((r.min_clearance_m for r in ordered),default=None),
        uncertain_target_ids=tuple(sorted(set(nominal.uncertain_target_ids)|set(conservative.uncertain_target_ids))),
        diagnostics=tuple(dict.fromkeys(nominal.diagnostics+conservative.diagnostics)))


class KevDynamicAvoidance:
    def __init__(self, config, speed_transition_s, speed_unit='knots'):
        self.config = config
        self.speed_transition_s = speed_transition_s
        self.speed_unit = speed_unit
        self.action_config = replace(config,horizon_s=config.action_horizon_s)
        self.planner = DynamicAvoidancePlanner(self.action_config)
        self.last_warning = None
        self.early_targets = {}

    def effective_config(self,scene,desired,*,track=False):
        horizon,encounters=encounter_window(scene,desired,self.config)
        own,targets=scene
        live={t.target_id:t for t in targets}
        retained={}
        for pid,(direction,window,maneuvering) in self.early_targets.items():
            target=live.get(pid)
            if target is None or target.position_m is None or target.speed_mps is None or target.course_deg is None or target.age_s>self.config.max_target_age_s:
                continue
            angle=math.radians(target.course_deg)
            offset=tuple(target.position_m[i]+target.speed_mps*target.age_s*(math.sin(angle) if i==0 else math.cos(angle))-own.position_m[i] for i in (0,1))
            if sum(offset[i]*direction[i] for i in (0,1))>=-self.config.safe_distance_m-20.:
                retained[pid]=(direction,window,maneuvering)
                horizon=max(horizon,window)
        direction=(math.sin(math.radians(own.course_deg)),math.cos(math.radians(own.course_deg)))
        for encounter in encounters:
            pid=encounter['target_id']
            old=retained.get(pid,(direction,horizon,False))
            retained[pid]=(old[0],max(old[1],horizon),old[2] or encounter['phase']=='maneuver')
        if track:
            self.early_targets=retained
        return replace(self.action_config,horizon_s=horizon),dict(active=bool(retained),
            target_ids=sorted(retained),effective_horizon_s=horizon,encounters=encounters,
            phase='maneuver' if any(v[2] for v in retained.values()) else ('prepare' if retained else 'clear'),
            control_delay_s=self.config.control_delay_s,
            controller_deceleration_mps2=self.config.controller_deceleration_mps2)

    def needs_avoidance(self, scene, desired):
        config,_=self.effective_config(scene,desired)
        return not check_models(scene,desired,config,self.speed_transition_s).safe

    def extra_speeds(self, speeds):
        """Use wire-representable knots, including speeds below Kev's 10 knots."""
        cap = max(speeds)
        return sorted(set(speeds) | {min(cap, 2.5), min(cap, 5.)}
                      | {round(cap*f, 2) for f in (0., .25, .5, .75)})

    def filter(self, scene, criteria, *, expand, prefer_speed=False):
        """Preserve clear baselines and filter all moving targets with one safety planner.

        expand(speeds_knots) must return candidates that passed the original
        static checks, within the baseline speed cap and reachable courses.
        """
        own, targets = scene
        def actions_for(items):
            return {key: wire_action(item['speed_knots'], item['absolute_course_deg'], self.speed_unit)
                    for key, item in items.items()}
        actions = actions_for(criteria)
        preferred = min(criteria, key=lambda key: (criteria[key]['target_heading_error_deg'],
                                                  -criteria[key]['speed_knots']))
        desired = actions[preferred]
        effective,early=self.effective_config(scene,desired,track=True)
        self.planner.config=effective
        warning=check_models(scene,desired,self.config,self.speed_transition_s,
                             horizon_s=self.config.warning_horizon_s)
        warning_targets=sorted({r.target_id for r in warning.risks if not r.safe}
                               | set(warning.uncertain_target_ids))
        self.last_warning=(dict(warning_only=True,warning_horizon_s=self.config.warning_horizon_s,
            action_horizon_s=self.config.action_horizon_s,threat_ids=warning_targets,
            assessment=asdict(warning)) if warning_targets else None)
        cache = {}
        def evaluate(action):
            if action not in cache:
                cache[action] = check_models(scene, action, effective, self.speed_transition_s)
            return cache[action]
        checks = {key: evaluate(action) for key, action in actions.items()}
        desired_check = checks[preferred]
        threats = {r.target_id for check in checks.values() for r in check.risks if not r.safe}
        threats.update(t for check in checks.values() for t in check.uncertain_target_ids)
        if all(check.safe for check in checks.values()):
            if not early['active']:
                self.planner.reset()
                return criteria, None
        if not desired_check.safe or early['active']:
            cap = max(item['speed_knots'] for item in criteria.values())
            criteria = expand(self.extra_speeds([cap]))
            if any(item['speed_knots'] > cap+1e-9 for item in criteria.values()):
                raise ValueError('dynamic expansion exceeded the static speed cap')
            actions = actions_for(criteria)
            checks = {key: evaluate(action) for key, action in actions.items()}
        safe = {key:item for key,item in criteria.items() if checks[key].safe}
        if early['phase']=='prepare' and safe:
            # Far encounter: change speed before changing the nominal route
            # heading, whenever that still leaves a certified safe action.
            error=min(abs(signed_turn(desired.course_deg,actions[k].course_deg)) for k in safe)
            if error<=3.:
                safe={k:item for k,item in safe.items()
                      if abs(signed_turn(desired.course_deg,actions[k].course_deg))<=error+1e-6}
                fastest=max(item['speed_knots'] for item in safe.values())
                safe={k:item for k,item in safe.items() if item['speed_knots']>=fastest-1e-9}
                early['trigger']='prepare_with_route_heading'
            else:
                early.update(phase='maneuver',trigger='no_safe_route_heading_action')
        if prefer_speed and safe:
            # Among useful headings toward the geometric bypass, retain the
            # fastest certified speed. Never enforce a minimum unsafe speed.
            error = min(item['target_heading_error_deg'] for item in safe.values())
            useful = {key:item for key,item in safe.items()
                      if item['target_heading_error_deg'] <= min(90.,error+30.)}
            if useful:
                fastest = max(item['speed_knots'] for item in useful.values())
                safe = {key:item for key,item in useful.items() if item['speed_knots'] >= fastest-1e-9}
        admissible = [actions[key] for key in safe]
        result = self.planner.plan(own, targets, desired, candidates=admissible)
        if result.action is None:
            result = replace(result, assessment=desired_check, threat_ids=tuple(sorted(threats)),
                status='insufficient_data' if desired_check.uncertain_target_ids else 'blocked')
        else:
            result = replace(result, assessment=evaluate(result.action))
        if result.action is not None and early['active'] and abs(signed_turn(desired.course_deg,result.action.course_deg))>3.:
            for pid,(direction,window,_) in self.early_targets.items():
                self.early_targets[pid]=(direction,window,True)
            early['phase']='maneuver'
        details = asdict(result)
        details['targets'] = len(targets)
        details['horizon_s'] = effective.horizon_s
        details['action_horizon_s'] = self.config.action_horizon_s
        details['warning_horizon_s'] = self.config.warning_horizon_s
        details['early_avoidance'] = early
        details['threat_ids'] = sorted(threats | set(early['target_ids']))
        if early['active'] and result.action is not None:
            details['status']='avoiding'
        if result.action is None:
            raise DynamicAvoidanceHold(details)
        if result.status == 'clear' and not desired_check.safe:
            # A replacement action can be clear even though the original desired
            # action was unsafe. Keep avoidance active until recovery.
            details['status'] = 'avoiding'
        if details['status'] == 'avoiding':
            preferred = signed_turn(own.course_deg, result.action.course_deg)
            # Keep Kev's choice within the chosen safe maneuver side. Its
            # preferred action is guaranteed to survive all static constraints.
            safe = {key: item for key, item in safe.items()
                    if (signed_turn(own.course_deg, actions[key].course_deg)*preferred > 0
                        if abs(preferred) > .001 else
                        abs(signed_turn(own.course_deg, actions[key].course_deg)) < .001)}
        # Keep model context bounded to the normal 16 options while retaining
        # the planner's recommendation and a spread of speeds and headings.
        if len(safe) > 16:
            selected = [next(key for key in safe if actions[key] == result.action)]
            groups = {}
            for key in safe:
                groups.setdefault(actions[key].speed_mps, []).append(key)
            for group in groups.values():
                group.sort(key=lambda key: abs(signed_turn(desired.course_deg, actions[key].course_deg)))
            while len(selected) < 16:
                for speed in sorted(groups, reverse=True):
                    group = groups[speed]
                    while group and group[0] in selected:
                        group.pop(0)
                    if group and len(selected) < 16:
                        selected.append(group.pop(0))
            safe = {key: safe[key] for key in selected}
        for key, item in safe.items():
            turn = item.pop('turn_deg')
            item['dynamic_clearance_m'] = checks[key].min_clearance_m
            item['turn_deg'] = turn
        return safe, details


def control_check(scene, speed_knots, course, config, speed_transition_s, speed_unit='knots'):
    if scene is None:
        return dict(safe=True,min_clearance_m=None,obstacle_id=None,avoidance_type='dynamic')
    own, targets = scene
    action=wire_action(speed_knots,course,speed_unit)
    horizon,encounters=encounter_window(scene,action,config)
    check = check_models(scene,action,config,speed_transition_s,horizon_s=horizon)
    nearest = min(check.risks, key=lambda r: r.min_clearance_m, default=None)
    return dict(safe=check.safe, min_clearance_m=check.min_clearance_m,
                obstacle_id=(check.uncertain_target_ids[0] if check.uncertain_target_ids
                             else nearest.target_id if nearest else None),
                avoidance_type='dynamic', action_horizon_s=config.action_horizon_s,
                effective_horizon_s=horizon,early_encounters=encounters,assessment=asdict(check))
