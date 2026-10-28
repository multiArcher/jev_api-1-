"""Kev integration: snapshot conversion and safe model-candidate filtering.

Only moving or uncertain targets enter the dynamic forecast. Static checks
are supplied by Kev as a separate candidate-builder callback.
"""
from dataclasses import asdict, replace
from tools.waypoint_geometry import KNOT_MPS
from .adapter import scene_from_situation
from .planner import DynamicAvoidancePlanner
from .prediction import assess_action, assess_trajectory, forecast, signed_turn
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
    """Require both bounded physical response and configured simulator response."""
    own, targets = scene
    physical = assess_action(own, targets, action, config, horizon_s=horizon_s)
    configured = assess_trajectory(forecast(own, action, config,
        speed_transition_time_s=speed_transition_s, horizon_s=horizon_s), targets, config)
    if physical.safe != configured.safe:
        return configured if not configured.safe else physical
    return min((physical, configured), key=lambda c: (
        -len(c.uncertain_target_ids), c.min_clearance_m if c.min_clearance_m is not None else float('inf')))


class KevDynamicAvoidance:
    def __init__(self, config, speed_transition_s, speed_unit='knots'):
        self.config = config
        self.speed_transition_s = speed_transition_s
        self.speed_unit = speed_unit
        self.action_config = replace(config,horizon_s=config.action_horizon_s)
        self.planner = DynamicAvoidancePlanner(self.action_config)
        self.last_warning = None

    def needs_avoidance(self, scene, desired):
        return not check_models(scene, desired, self.action_config, self.speed_transition_s).safe

    def extra_speeds(self, speeds):
        """Use wire-representable knots, including speeds below Kev's 10 knots."""
        cap = max(speeds)
        return sorted(set(speeds) | {min(cap, 2.5), min(cap, 5.)}
                      | {round(cap*f, 2) for f in (0., .25, .5, .75)})

    def filter(self, scene, criteria, *, expand):
        """Preserve an all-safe baseline; expand only when the desired action is unsafe.

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
                cache[action] = check_models(scene, action, self.action_config, self.speed_transition_s)
            return cache[action]
        checks = {key: evaluate(action) for key, action in actions.items()}
        if all(check.safe for check in checks.values()):
            self.planner.reset()
            return criteria, None
        desired_check = checks[preferred]
        threats = {r.target_id for check in checks.values() for r in check.risks if not r.safe}
        threats.update(t for check in checks.values() for t in check.uncertain_target_ids)
        if not desired_check.safe:
            cap = max(item['speed_knots'] for item in criteria.values())
            criteria = expand(self.extra_speeds([cap]))
            if any(item['speed_knots'] > cap+1e-9 for item in criteria.values()):
                raise ValueError('dynamic expansion exceeded the static speed cap')
            actions = actions_for(criteria)
            checks = {key: evaluate(action) for key, action in actions.items()}
        admissible = [action for key, action in actions.items() if checks[key].safe]
        result = self.planner.plan(own, targets, desired, candidates=admissible)
        if result.action is None:
            result = replace(result, assessment=desired_check, threat_ids=tuple(sorted(threats)),
                status='insufficient_data' if desired_check.uncertain_target_ids else 'blocked')
        else:
            result = replace(result, assessment=evaluate(result.action))
        details = asdict(result)
        details['targets'] = len(targets)
        details['horizon_s'] = self.config.action_horizon_s
        details['action_horizon_s'] = self.config.action_horizon_s
        details['warning_horizon_s'] = self.config.warning_horizon_s
        details['threat_ids'] = sorted(threats)
        if result.action is None:
            raise DynamicAvoidanceHold(details)
        safe = {key: item for key, item in criteria.items() if checks[key].safe}
        if result.status == 'clear' and not desired_check.safe:
            # Desired may be safe under the physical model but fail under the
            # configured fast response. Candidates still passed both checks.
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
    check = check_models(scene, wire_action(speed_knots, course, speed_unit), config, speed_transition_s,
                         horizon_s=config.action_horizon_s)
    nearest = min(check.risks, key=lambda r: r.min_clearance_m, default=None)
    return dict(safe=check.safe, min_clearance_m=check.min_clearance_m,
                obstacle_id=(check.uncertain_target_ids[0] if check.uncertain_target_ids
                             else nearest.target_id if nearest else None),
                avoidance_type='dynamic', action_horizon_s=config.action_horizon_s,assessment=asdict(check))
