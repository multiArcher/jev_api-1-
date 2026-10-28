"""Time-aligned forecasts and swept relative-segment clearance checks."""
from dataclasses import dataclass
import math
from tools.navigation_motion import predict_ground_motion
from .types import Assessment, TargetRisk


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
    speed_time = abs(action.speed_mps-own.speed_mps) / (
        config.acceleration_mps2 if action.speed_mps > own.speed_mps else config.deceleration_mps2)
    if speed_transition_time_s is not None:
        if not math.isfinite(speed_transition_time_s) or speed_transition_time_s < 0:
            raise ValueError('speed_transition_time_s must be finite and nonnegative')
        speed_time = speed_transition_time_s if action.speed_mps != own.speed_mps else 0.
    turn = signed_turn(own.course_deg, action.course_deg) if action.speed_mps > 0 else 0.
    # The underlying predictor resolves the exact 180-degree tie to port.
    turn_time = abs(turn)/config.max_turn_rate_deg_s
    horizon = config.horizon_s if horizon_s is None else horizon_s
    if isinstance(horizon,bool) or not isinstance(horizon,(int,float)) or not math.isfinite(horizon) or horizon <= 0:
        raise ValueError('forecast horizon must be positive and finite')
    duration = max(horizon, speed_time, turn_time)

    def segment(speed, course, requested, heading, transition, course_time, length):
        result = predict_ground_motion(speed, course, requested, heading,
            transition, course_time, length, dt=config.dt_s,
            max_turn_rate_deg_s=config.max_turn_rate_deg_s)
        # Mirror the public predictor's integration boundaries: it inserts
        # transition times, so a path index is not necessarily i*dt.
        times = sorted(set([0., length]
            + [i*config.dt_s for i in range(1, math.ceil(length/config.dt_s)) if i*config.dt_s < length]
            + [t for t in (transition, course_time) if 0 < t < length]))
        return times, result['path']

    times, offsets = segment(own.speed_mps, own.course_deg, action.speed_mps,
                            action.course_deg, speed_time, turn_time, duration)
    points = [(own.position_m[0]+p[0], own.position_m[1]+p[1]) for p in offsets]
    braking_time = action.speed_mps/config.deceleration_mps2
    if braking_time:
        heading = action.course_deg if action.speed_mps > 0 else own.course_deg
        tail_times, tail = segment(action.speed_mps, heading, 0., heading,
                                  braking_time, 0., braking_time)
        end = points[-1]
        points.extend((end[0]+p[0], end[1]+p[1]) for p in tail[1:])
        times.extend(duration+t for t in tail_times[1:])
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
    return assess_trajectory(forecast(own, action, config, horizon_s=horizon_s), targets, config)
