"""Task-local route recovery geometry, independent of obstacle prediction/control.

Positions are fixed local east/north metres; courses are clockwise from north.
The caller supplies its own static segment check and retains action safety guards.
"""
from dataclasses import dataclass
import math


def refine_courses(courses, current_course_deg, max_turn_deg, preferred_courses):
    """Include precise route directions within the existing yaw envelope.

    Keep the sample count and straight course. A uniform 16-course grid can
    otherwise omit the segment direction and force persistent tracking drift.
    """
    result = list(courses)
    protected = {round(current_course_deg, 2) % 360}
    for preferred in preferred_courses:
        course = round(preferred % 360, 2) % 360
        if abs((course-current_course_deg+180)%360-180) > min(max_turn_deg, 179.99) + 1e-8:
            continue
        if course not in result:
            choices = [i for i, value in enumerate(result) if value not in protected]
            if not choices:
                continue
            nearest = min(choices, key=lambda i: abs((result[i]-course+180)%360-180))
            result[nearest] = course
        protected.add(course)
    return result


@dataclass(frozen=True)
class RouteRejoinConfig:
    enabled: bool = True
    min_lookahead_m: float = 30.
    lookahead_s: float = 8.
    merge_angle_deg: float = 30.
    cross_track_tolerance_m: float = 10.
    heading_tolerance_deg: float = 5.
    stable_cycles: int = 3

    def __post_init__(self):
        if type(self.enabled) is not bool:
            raise ValueError('enabled must be a bool')
        for name in ('min_lookahead_m', 'lookahead_s', 'merge_angle_deg',
                     'cross_track_tolerance_m', 'heading_tolerance_deg'):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if self.merge_angle_deg >= 90 or self.heading_tolerance_deg > 180:
            raise ValueError('merge angle must be below 90; heading tolerance at most 180')
        if type(self.stable_cycles) is not int or self.stable_cycles < 1:
            raise ValueError('stable_cycles must be a positive integer')


class RouteRejoin:
    def __init__(self, start_xy_m, config=None):
        self.config = config or RouteRejoinConfig()
        self.start_xy_m = tuple(start_xy_m)
        self.active = False
        self.index = None
        self.stable_count = 0
        self.mode = 'inactive'

    def activate(self):
        """Latch actual avoidance; warnings alone must never call this method."""
        if self.config.enabled:
            self.active = True
            self.stable_count = 0
            self.mode = 'avoiding'

    def guide(self, geometry, speed_mps, course_deg, segment_clear, *, avoiding=False):
        """Return diagnostics and an optional safe forward guidance point.

        A missing point means the caller keeps ordinary obstacle navigation.
        Once activated, line following persists across legs until task reset.
        """
        if not self.active:
            return None
        index = geometry.next_index
        if index != self.index:
            self.index = index
            self.stable_count = 0
        start = self.start_xy_m if index == 0 else geometry.waypoint_positions[index - 1]
        end = geometry.waypoint_positions[index]
        length = math.dist(start, end)
        if length < 1e-6:
            self.mode = 'degenerate'
            return dict(mode=self.mode, guidance_xy_m=None, waypoint_index=index,
                        segment_start_xy_m=start, segment_end_xy_m=end)
        direction = tuple((end[i] - start[i]) / length for i in (0, 1))
        offset = tuple(geometry.boat_position[i] - start[i] for i in (0, 1))
        along = sum(offset[i] * direction[i] for i in (0, 1))
        cross = direction[0] * offset[1] - direction[1] * offset[0]
        bearing = math.degrees(math.atan2(direction[0], direction[1])) % 360
        heading_error = abs((bearing - course_deg + 180) % 360 - 180)
        details = dict(waypoint_index=index, segment_start_xy_m=start, segment_end_xy_m=end,
                       segment_course_deg=bearing,
                       along_track_m=along, cross_track_m=cross,
                       heading_error_deg=heading_error,
                       progress_distance_m=max(0., length - along) + abs(cross),
                       guidance_xy_m=None)
        if avoiding:
            self.mode = 'avoiding'
            self.stable_count = 0
        else:
            cfg = self.config
            lookahead = max(cfg.min_lookahead_m, max(0., speed_mps) * cfg.lookahead_s,
                            abs(cross) / math.tan(math.radians(cfg.merge_angle_deg)))
            position = min(length, max(0., along) + lookahead)
            while True:
                point = tuple(start[i] + direction[i] * position for i in (0, 1))
                if segment_clear(geometry.boat_position, point):
                    details['guidance_xy_m'] = point
                    break
                if position >= length:
                    break
                position = min(length, position + cfg.min_lookahead_m)
            if details['guidance_xy_m'] is None:
                self.mode = 'waiting'
                self.stable_count = 0
            else:
                aligned = (0. <= along <= length and abs(cross) <= cfg.cross_track_tolerance_m
                           and heading_error <= cfg.heading_tolerance_deg)
                self.stable_count = self.stable_count + 1 if aligned else 0
                self.mode = 'following' if self.stable_count >= cfg.stable_cycles else 'rejoining'
        details.update(mode=self.mode, stable_cycles=self.stable_count)
        return details
