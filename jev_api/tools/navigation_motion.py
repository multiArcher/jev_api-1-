"""Shared bounded physical ground-motion forecast in fixed east/north axes."""
import math
from .acceleration_response import response_applies, measured_speed_profile
from .vessel_dynamics import (
    MAX_SPEED_MPS, MAX_ACCELERATION_MPS2, MAX_DECELERATION_MPS2,
    MAX_TURN_RATE_DEG_S, MAX_TURN_ACCELERATION_DEG_S2,
    positive_limit, validate_curve, curve_limit, turn_profile, step_acceleration_limit,
)


def predict_ground_motion(speed_mps, course_deg, target_speed_mps, target_course_deg,
                          speed_time, course_time, duration, dt=.02, *,
                          max_turn_rate_deg_s=None,
                          acceleration_mps2=MAX_ACCELERATION_MPS2,
                          deceleration_mps2=MAX_DECELERATION_MPS2,
                          max_turn_acceleration_deg_s2=MAX_TURN_ACCELERATION_DEG_S2,
                          initial_turn_rate_deg_s=0., acceleration_curve=(),
                          deceleration_curve=(), turn_rate_curve=(), acceleration_response='bounded'):
    """Forecast target commands under vessel limits, retaining angular momentum.

    speed_time/course_time remain accepted for wire/client compatibility only;
    they cannot bypass physical limits. Missing course uses target direction as
    an explicit startup assumption. Curves use SI speed/limit pairs.
    """
    rate_cap = MAX_TURN_RATE_DEG_S if max_turn_rate_deg_s is None else max_turn_rate_deg_s
    for name, value, maximum in (
        ('acceleration_mps2', acceleration_mps2, MAX_ACCELERATION_MPS2),
        ('deceleration_mps2', deceleration_mps2, MAX_DECELERATION_MPS2),
        ('max_turn_rate_deg_s', rate_cap, MAX_TURN_RATE_DEG_S),
        ('max_turn_acceleration_deg_s2', max_turn_acceleration_deg_s2, MAX_TURN_ACCELERATION_DEG_S2)):
        positive_limit(name, value, maximum)
    if not all(math.isfinite(v) for v in (speed_mps, target_speed_mps, target_course_deg, duration, dt, initial_turn_rate_deg_s)) or duration < 0 or dt <= 0:
        raise ValueError('motion inputs must be finite; duration >= 0 and dt > 0')
    if target_speed_mps < 0 or abs(initial_turn_rate_deg_s) > MAX_TURN_RATE_DEG_S:
        raise ValueError('nonnegative target speed and bounded initial turn rate required')
    acceleration_curve = validate_curve(acceleration_curve, acceleration_mps2)
    deceleration_curve = validate_curve(deceleration_curve, deceleration_mps2)
    turn_rate_curve = validate_curve(turn_rate_curve, rate_cap)
    initial = abs(speed_mps)
    requested = min(target_speed_mps, MAX_SPEED_MPS)
    start = target_course_deg if course_deg is None else course_deg
    if not math.isfinite(start):
        raise ValueError('course must be finite or None')
    delta = (target_course_deg-start+180) % 360-180
    if requested == 0:
        # No new turn when stopping, but existing rotation must brake smoothly.
        delta = initial_turn_rate_deg_s*abs(initial_turn_rate_deg_s)/(2*max_turn_acceleration_deg_s2)
    # A conservative constant yaw envelope over the traversed speed range.
    yaw_cap = min(curve_limit(turn_rate_curve, initial, rate_cap),
                  curve_limit(turn_rate_curve, requested, rate_cap),
                  *(limit for speed, limit in turn_rate_curve if min(initial, requested) <= speed <= max(initial, requested)))
    yaw, turn_boundaries = turn_profile(delta, initial_turn_rate_deg_s, yaw_cap, max_turn_acceleration_deg_s2)
    increasing = requested > initial
    cap = acceleration_mps2 if increasing else deceleration_mps2
    curve = acceleration_curve if increasing else deceleration_curve
    transition = abs(requested-initial)/cap
    use_measured = response_applies(acceleration_response, initial, requested, course_deg, target_course_deg, initial_turn_rate_deg_s)
    response_times = []
    if use_measured:
        response_speed, response_times = measured_speed_profile(initial, requested, cap, acceleration_curve)
        transition = response_times[-1]
    boundaries = [duration/2, transition, *turn_boundaries, *response_times]
    times = sorted(set([0., duration] + [i*dt for i in range(1, math.ceil(duration/dt))]
                       + [t for t in boundaries if 0 < t < duration]))
    speed = initial
    x = y = 0.
    path, speeds, courses, rates = [[0., 0.]], [speed], [start % 360], [initial_turn_rate_deg_s]
    midpoint = [0., 0.]
    for t0, t1 in zip(times, times[1:]):
        step = t1-t0
        limit = step_acceleration_limit(curve, speed, requested, cap, step)
        next_speed = response_speed(t1) if use_measured else (min(requested, speed+limit*step) if increasing else max(requested, speed-limit*step))
        # Simpson quadrature of continuously varying heading and linear speed.
        dx = dy = 0.
        for fraction, weight in ((0., 1), (.5, 4), (1., 1)):
            velocity = speed+(next_speed-speed)*fraction
            heading = math.radians(start+yaw(t0+step*fraction)[0])
            dx += weight*velocity*math.sin(heading)*step/6
            dy += weight*velocity*math.cos(heading)*step/6
        x += dx
        y += dy
        speed = next_speed
        offset, omega = yaw(t1)
        path.append([x, y]); speeds.append(speed)
        courses.append((start+offset) % 360); rates.append(omega)
        if abs(t1-duration/2) < 1e-10:
            midpoint = [x, y]
    return dict(midpoint=midpoint, endpoint=[x, y], end_speed=speed,
                end_course_deg=courses[-1], end_turn_rate_deg_s=rates[-1],
                path=path, path_speeds=speeds, path_courses_deg=courses,
                path_turn_rates_deg_s=rates, times_s=times,
                speed_transition_time_s=transition, turn_transition_time_s=max(turn_boundaries, default=0.),
                acceleration_response_used=acceleration_response if use_measured else 'bounded')
