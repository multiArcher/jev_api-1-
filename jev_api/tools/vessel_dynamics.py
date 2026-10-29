"""Shared vessel limits. Curves use (speed_mps, positive_limit) SI pairs.

Only the 11-knot yaw datum is measured/provided; constant limits elsewhere are
an explicit provisional envelope, not an identified hydrodynamic model.
"""
import math

DEFAULT_ACCELERATION_RESPONSE = 'measured_0_50kn'

MAX_SPEED_KNOTS = 50.
MAX_SPEED_MPS = MAX_SPEED_KNOTS * (1852 / 3600)
MAX_ACCELERATION_MPS2 = 1.46
MAX_DECELERATION_MPS2 = 1.75
MAX_TURN_RATE_DEG_S = 12.
MAX_TURN_ACCELERATION_DEG_S2 = 4.


def positive_limit(name, value, maximum):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= maximum:
        raise ValueError(f'{name} must be positive and <= {maximum}')
    return value


def validate_curve(curve, maximum):
    previous = -1.
    for speed, limit in curve:
        if isinstance(speed,bool) or not isinstance(speed,(int,float)) or not math.isfinite(speed) or speed < 0 or speed <= previous:
            raise ValueError('curve speeds must be finite, nonnegative and strictly increasing')
        positive_limit('curve limit', limit, maximum)
        previous = speed
    return tuple(tuple(point) for point in curve)


def curve_limit(curve, speed, default):
    if not curve:
        return default
    if speed <= curve[0][0]:
        return curve[0][1]
    for (x0, y0), (x1, y1) in zip(curve, curve[1:]):
        if speed <= x1:
            return y0 + (y1-y0)*(speed-x0)/(x1-x0)
    return curve[-1][1]


def motion_parameters(execution):
    """Physical settings forwarded by every forecast; old transition keys ignored."""
    keys = ('acceleration_mps2', 'deceleration_mps2', 'max_turn_rate_deg_s',
            'max_turn_acceleration_deg_s2', 'initial_turn_rate_deg_s',
            'acceleration_curve', 'deceleration_curve', 'turn_rate_curve', 'acceleration_response')
    return dict({'acceleration_response':DEFAULT_ACCELERATION_RESPONSE}, **{key: execution[key] for key in keys if key in execution})


def limits_state():
    return dict(max_speed_knots=MAX_SPEED_KNOTS, acceleration_mps2=MAX_ACCELERATION_MPS2,
                deceleration_mps2=MAX_DECELERATION_MPS2, max_turn_rate_deg_s=MAX_TURN_RATE_DEG_S,
                max_turn_acceleration_deg_s2=MAX_TURN_ACCELERATION_DEG_S2,
                calibration='12 deg/s at 11 knots; constant envelope pending speed curves')


def turn_profile(delta, initial_rate=0., rate_limit=MAX_TURN_RATE_DEG_S,
                 acceleration=MAX_TURN_ACCELERATION_DEG_S2):
    """Exact bounded angular trajectory, including braking and reversal.

    Returns (sample(t)->(offset, rate), segment_end_times). A changed command
    preserves initial angular momentum; arriving at a heading ends at zero rate.
    """
    segments = []
    offset, rate, clock = 0., initial_rate, 0.

    def append(duration, alpha):
        nonlocal offset, rate, clock
        if duration > 1e-12:
            segments.append((clock, duration, offset, rate, alpha))
            offset += rate*duration + .5*alpha*duration*duration
            rate += alpha*duration
            clock += duration

    direction = math.copysign(1., delta) if delta else 1.
    # Brake before reversing or when momentum makes the target unavoidable.
    if initial_rate and (initial_rate*direction < 0 or initial_rate**2/(2*acceleration) > abs(delta)
                         or abs(initial_rate) > rate_limit):
        append(abs(initial_rate)/acceleration, -math.copysign(acceleration, initial_rate))
    remaining = delta-offset
    direction = math.copysign(1., remaining) if remaining else 1.
    distance, start_rate = abs(remaining), rate*direction
    peak = min(rate_limit, math.sqrt(max(0., acceleration*distance + .5*start_rate**2)))
    append(max(0., (peak-start_rate)/acceleration), direction*acceleration)
    braking_distance = peak**2/(2*acceleration)
    cruise_distance = max(0., abs(delta-offset)-braking_distance)
    append(cruise_distance/peak if peak else 0., 0.)
    append(peak/acceleration, -direction*acceleration)

    def sample(t):
        for begin, length, position, omega, alpha in segments:
            if t < begin+length:
                elapsed = max(0., t-begin)
                return position+omega*elapsed+.5*alpha*elapsed**2, omega+alpha*elapsed
        return delta, 0.

    return sample, [begin+length for begin, length, *_ in segments]


def step_acceleration_limit(curve, speed, target, maximum, seconds):
    """Conservative curve bound throughout one numerical integration step."""
    if not curve:
        return maximum
    end = min(target,speed+maximum*seconds) if target>speed else max(target,speed-maximum*seconds)
    lower, upper = sorted((speed,end))
    return min(curve_limit(curve,speed,maximum),curve_limit(curve,end,maximum),
               *(limit for point,limit in curve if lower <= point <= upper))
