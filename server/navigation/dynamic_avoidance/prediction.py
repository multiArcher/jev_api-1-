"""Time-aligned forecasts and swept relative-segment clearance checks."""
from dataclasses import dataclass, replace
import math
from tools.navigation_motion import predict_ground_motion
from .types import Assessment, TargetRisk
from tools.vessel_dynamics import MAX_SPEED_MPS, turn_profile
from tools.acceleration_response import response_applies, measured_speed_profile


def signed_turn(start, end):
    return (end-start+180) % 360-180


def velocity(speed, course):
    angle = math.radians(course)
    return speed*math.sin(angle), speed*math.cos(angle)


@dataclass(frozen=True)
class Trajectory:
    times_s: tuple[float, ...]
    positions_m: tuple[tuple[float, float], ...]
    braking_start_s: float


def forecast(own, action, config, *, speed_transition_time_s=None, horizon_s=None):
    """Hold action for the forecast, then simulate a time-aligned braking tail."""
    if config.control_delay_s:
        delay=config.control_delay_s
        prefix=predict_ground_motion(own.speed_mps,own.course_deg,own.speed_mps,
            own.course_deg+own.turn_rate_deg_s*delay,0.,0.,delay,dt=config.dt_s,
            acceleration_mps2=config.acceleration_mps2,deceleration_mps2=config.deceleration_mps2,
            max_turn_rate_deg_s=config.max_turn_rate_deg_s,
            max_turn_acceleration_deg_s2=config.max_turn_acceleration_deg_s2,
            initial_turn_rate_deg_s=own.turn_rate_deg_s,acceleration_curve=config.acceleration_curve,
            deceleration_curve=config.deceleration_curve,turn_rate_curve=config.turn_rate_curve,
            acceleration_response=config.acceleration_response)
        delayed=replace(own,position_m=tuple(own.position_m[i]+prefix['endpoint'][i] for i in (0,1)),
            speed_mps=prefix['end_speed'],course_deg=prefix['end_course_deg'],
            turn_rate_deg_s=prefix['end_turn_rate_deg_s'])
        rest=forecast(delayed,action,replace(config,control_delay_s=0.),horizon_s=horizon_s)
        times=tuple(prefix['times_s'])+tuple(delay+t for t in rest.times_s[1:])
        positions=tuple(tuple(own.position_m[i]+p[i] for i in (0,1)) for p in prefix['path'])+rest.positions_m[1:]
        return Trajectory(times,positions,delay+rest.braking_start_s)
    requested = min(action.speed_mps, MAX_SPEED_MPS)
    acceleration = config.acceleration_mps2 if requested > own.speed_mps else config.deceleration_mps2
    curve = config.acceleration_curve if requested > own.speed_mps else config.deceleration_curve
    acceleration = min([acceleration]+[p[1] for p in curve])
    speed_time = abs(requested-own.speed_mps)/acceleration
    if response_applies(config.acceleration_response, own.speed_mps, requested, own.course_deg, action.course_deg, own.turn_rate_deg_s):
        _,response_times = measured_speed_profile(own.speed_mps, requested, config.acceleration_mps2, config.acceleration_curve)
        speed_time = max(speed_time,response_times[-1])
    turn = signed_turn(own.course_deg, action.course_deg) if requested > 0 else own.turn_rate_deg_s*abs(own.turn_rate_deg_s)/(2*config.max_turn_acceleration_deg_s2)
    rate = min([config.max_turn_rate_deg_s]+[p[1] for p in config.turn_rate_curve])
    _, turns = turn_profile(turn, own.turn_rate_deg_s, rate, config.max_turn_acceleration_deg_s2)
    turn_time = max(turns, default=0.)
    horizon = config.horizon_s if horizon_s is None else horizon_s
    if isinstance(horizon,bool) or not isinstance(horizon,(int,float)) or not math.isfinite(horizon) or horizon <= 0:
        raise ValueError('forecast horizon must be positive and finite')
    duration = max(horizon, speed_time, turn_time)

    def segment(speed, course, target, heading, length, initial_rate):
        return predict_ground_motion(speed, course, target, heading, 0., 0., length, dt=config.dt_s,
            acceleration_mps2=config.acceleration_mps2, deceleration_mps2=config.deceleration_mps2,
            max_turn_rate_deg_s=config.max_turn_rate_deg_s,
            max_turn_acceleration_deg_s2=config.max_turn_acceleration_deg_s2,
            initial_turn_rate_deg_s=initial_rate, acceleration_curve=config.acceleration_curve,
            deceleration_curve=config.deceleration_curve, turn_rate_curve=config.turn_rate_curve,
            acceleration_response=config.acceleration_response)

    motion = segment(own.speed_mps, own.course_deg, requested, action.course_deg, duration, own.turn_rate_deg_s)
    times = list(motion['times_s'])
    points = [(own.position_m[0]+p[0], own.position_m[1]+p[1]) for p in motion['path']]
    deceleration = min([config.deceleration_mps2]+[p[1] for p in config.deceleration_curve])
    braking_time = motion['end_speed']/deceleration
    if braking_time:
        tail = segment(motion['end_speed'], motion['end_course_deg'], 0., motion['end_course_deg'],
                       braking_time, motion['end_turn_rate_deg_s'])
        end = points[-1]
        points.extend((end[0]+p[0], end[1]+p[1]) for p in tail['path'][1:])
        times.extend(duration+t for t in tail['times_s'][1:])
    return Trajectory(tuple(times), tuple(points), duration)


def relative_segment_minimum(a, b):
    """Minimum distance to zero on a relative-motion segment and its fraction."""
    dx, dy = b[0]-a[0], b[1]-a[1]
    norm = dx*dx+dy*dy
    fraction = max(0., min(1., -(a[0]*dx+a[1]*dy)/norm)) if norm else 0.
    return math.hypot(a[0]+fraction*dx, a[1]+fraction*dy), fraction


def assess_trajectory(path, targets, config):
    risks, uncertain, diagnostics = [], [], []
    for target in targets:
        reliable = (target.position_m is not None and target.speed_mps is not None
                    and (target.course_deg is not None or target.speed_mps == 0)
                    and target.age_s <= config.max_target_age_s)
        if not reliable:
            possible = target.position_m is None
            if not possible:
                bound = max(config.unknown_target_speed_mps, target.speed_mps or 0.)
                for i in range(len(path.times_s)-1):
                    a, b = path.positions_m[i:i+2]
                    distance, _ = relative_segment_minimum(
                        (a[0]-target.position_m[0], a[1]-target.position_m[1]),
                        (b[0]-target.position_m[0], b[1]-target.position_m[1]))
                    reach = config.safe_distance_m + (bound+config.uncertainty_mps)*(
                        target.age_s+path.times_s[i+1]) + config.clearance_epsilon_m
                    if distance <= reach:
                        possible = True
                        break
            diagnostics.append(f'target {target.target_id}: missing/expired motion data; '
                               + ('reachable envelope intersects forecast' if possible else 'outside reachable envelope'))
            if possible:
                uncertain.append(target.target_id)
            continue
        vx, vy = velocity(target.speed_mps, target.course_deg or 0.)
        center = (target.position_m[0]+vx*target.age_s, target.position_m[1]+vy*target.age_s)
        closest, closest_time, clearance = float('inf'), 0., float('inf')
        for i, (t0, t1) in enumerate(zip(path.times_s, path.times_s[1:])):
            a, b = path.positions_m[i:i+2]
            distance, fraction = relative_segment_minimum(
                (a[0]-center[0]-vx*t0, a[1]-center[1]-vy*t0),
                (b[0]-center[0]-vx*t1, b[1]-center[1]-vy*t1))
            if distance < closest:
                closest, closest_time = distance, t0+(t1-t0)*fraction
            # Use the larger interval radius: conservative even between samples.
            radius = config.safe_distance_m + config.uncertainty_mps*(target.age_s+t1)
            clearance = min(clearance, distance-radius)
        risks.append(TargetRisk(target.target_id, clearance, closest, closest_time,
                                clearance > config.clearance_epsilon_m))
    minimum = min((r.min_clearance_m for r in risks), default=None)
    return Assessment(not uncertain and all(r.safe for r in risks), minimum,
                      tuple(risks), tuple(uncertain), tuple(diagnostics))


def assess_action(own, targets, action, config, *, horizon_s=None):
    targets = tuple(targets)
    nominal = forecast(own, action, config, horizon_s=horizon_s)
    first = assess_trajectory(nominal, targets, config)
    if not response_applies(config.acceleration_response, own.speed_mps, min(action.speed_mps,MAX_SPEED_MPS),
                            own.course_deg, action.course_deg, own.turn_rate_deg_s):
        return first
    # The measured response is not a maximum capability. A faster physical
    # response must also pass, over the same primary prediction interval.
    bounded = replace(config,acceleration_response='bounded')
    second = assess_trajectory(forecast(own,action,bounded,horizon_s=max(config.dt_s,nominal.braking_start_s-config.control_delay_s)),targets,bounded)
    risks = {}
    for assessment in (first,second):
        for risk in assessment.risks:
            if risk.target_id not in risks or risk.min_clearance_m < risks[risk.target_id].min_clearance_m:
                risks[risk.target_id] = risk
    ordered = tuple(risks[key] for key in sorted(risks))
    uncertain = tuple(sorted(set(first.uncertain_target_ids)|set(second.uncertain_target_ids)))
    diagnostics = tuple(dict.fromkeys(first.diagnostics+second.diagnostics))
    return Assessment(first.safe and second.safe,
                      min((risk.min_clearance_m for risk in ordered),default=None),
                      ordered,uncertain,diagnostics)
