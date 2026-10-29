"""Local course envelope under an explicitly configured yaw-rate assumption."""
import math

from tools.vessel_dynamics import MAX_TURN_RATE_DEG_S

DEFAULT_MAX_TURN_RATE_DEG_S = MAX_TURN_RATE_DEG_S


def reachable_courses(course_deg, duration_s, max_turn_rate_deg_s):
    """16 target course commands inside the rate envelope, including straight ahead.

    These are targets; acceleration-limited forecasts may not reach them in one cycle.

    Start with 16 evenly spaced samples and replace the closest central sample
    with straight ahead. Even sample counts cannot be both symmetric and include zero.
    Quantize inward so two-decimal commands never exceed the continuous envelope.
    """
    if not all(math.isfinite(v) for v in (course_deg, duration_s, max_turn_rate_deg_s)):
        raise ValueError('Course, duration and max turn rate must be finite')
    if duration_s <= 0 or max_turn_rate_deg_s <= 0:
        raise ValueError('Duration and max turn rate must be positive')
    span = min(duration_s * max_turn_rate_deg_s, 179.99)
    lower = math.ceil((course_deg - span) * 100 - 1e-8)
    upper = math.floor((course_deg + span) * 100 + 1e-8)
    if upper - lower < 15:
        raise ValueError('Turn range too narrow for 16 distinct two-decimal courses; increase control interval')
    samples = [round(lower + (upper-lower)*i/15) for i in range(16)]
    straight = round(course_deg*100)
    if lower <= straight <= upper:
        nearest = min(range(16),key=lambda i:abs(samples[i]-straight))
        samples[nearest] = straight
    return [(value % 36000)/100 for value in samples]


ALIGNMENT_ENTER_DEG = 30.
ALIGNMENT_EXIT_DEG = 15.
ALIGNMENT_PREVIEW_S = 10.
ALIGNMENT_MIN_GAIN_DEG = 2.


def alignment_courses(course_deg, target_course_deg, max_turn_rate_deg_s, duration_s):
    """Sixteen sustained heading targets; physical motion remains rate/alpha bounded.

    Include the exact guidance bearing to avoid a new sequence of tiny local
    commands. Preview duration is a planning horizon, not command hold time.
    """
    courses = reachable_courses(course_deg,max(duration_s,ALIGNMENT_PREVIEW_S),max_turn_rate_deg_s)
    target = round(target_course_deg,2)%360
    nearest = min(range(len(courses)),key=lambda index:abs((courses[index]-target+180)%360-180))
    courses[nearest] = target
    return courses
