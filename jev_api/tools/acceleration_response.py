"""Nominal measured response, separate from the physical capability envelope."""
from bisect import bisect_right
from functools import lru_cache
import json
import math
from pathlib import Path

from .vessel_dynamics import MAX_SPEED_MPS, MAX_ACCELERATION_MPS2, curve_limit, DEFAULT_ACCELERATION_RESPONSE

ACCELERATION_RESPONSES = (DEFAULT_ACCELERATION_RESPONSE, 'bounded')


def validate_response_mode(mode):
    if mode not in ACCELERATION_RESPONSES:
        raise ValueError(f'unknown acceleration response: {mode!r}')
    return mode


@lru_cache(maxsize=1)
def response_points():
    data = json.loads(Path(__file__).with_name('data').joinpath('acceleration_0_50kn.json').read_text())
    if data['kind'] != 'observed_controller_response' or data['target_speed_knots'] != 50.:
        raise ValueError('invalid measured-response calibration kind or target speed')
    points = tuple(tuple(point) for point in data['points'])
    if len(points) < 2 or points[0][1] != 0. or abs(points[-1][1]-MAX_SPEED_MPS) > 1e-9:
        raise ValueError('measured response must cover 0 to 50 knots')
    for (t0,v0),(t1,v1) in zip(points, points[1:]):
        if not all(math.isfinite(value) for value in (t0,v0,t1,v1)) or t1 <= t0 or v1 <= v0 or (v1-v0)/(t1-t0) > MAX_ACCELERATION_MPS2+1e-9:
            raise ValueError('response points must be monotone and satisfy the acceleration ceiling')
    return points


def response_applies(mode, speed, target, course, target_course, initial_rate=0.):
    validate_response_mode(mode)
    # A two-decimal SI control command may encode 50 knots as 25.72 m/s.
    at_recorded_target = abs(target-MAX_SPEED_MPS) <= .005000001
    straight = course is not None and abs((target_course-course+180)%360-180) <= .005000001 and abs(initial_rate) < 1e-9
    return mode == DEFAULT_ACCELERATION_RESPONSE and target > speed and target <= MAX_SPEED_MPS+1e-9 and at_recorded_target and straight


@lru_cache(maxsize=256)
def measured_speed_profile(speed, target, acceleration, capability_curve=()):
    """Return exact piecewise-linear speed(time), with all physical caps applied.

    Enter the monotone response at the current speed. Reissuing a target does
    not restart the trace. Unmeasured braking and other targets use bounded motion.
    """
    points = response_points()
    edges = [speed]+[value for _,value in points if speed < value < target]+[target]
    speeds = tuple(value for _,value in points)
    times, values = [0.], [speed]
    for lower, upper in zip(edges, edges[1:]):
        if upper <= lower:
            continue
        index = min(max(1,bisect_right(speeds, (lower+upper)/2)), len(points)-1)
        (t0,v0),(t1,v1) = points[index-1:index+1]
        observed_acceleration = (v1-v0)/(t1-t0)
        limit = min(acceleration, observed_acceleration,
                    curve_limit(capability_curve,lower,acceleration),
                    curve_limit(capability_curve,upper,acceleration),
                    *(bound for value,bound in capability_curve if lower <= value <= upper))
        times.append(times[-1]+(upper-lower)/limit); values.append(upper)

    def sample(t):
        index = min(max(1,bisect_right(times,t)), len(times)-1)
        if t >= times[-1]:
            return target
        return values[index-1]+(values[index]-values[index-1])*(max(0.,t)-times[index-1])/(times[index]-times[index-1])

    return sample, times[1:]
