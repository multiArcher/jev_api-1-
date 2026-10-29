"""Public SI-unit inputs and outputs for the offline avoidance planner."""
from dataclasses import dataclass
import math
from typing import Literal
from tools.vessel_dynamics import positive_limit, validate_curve
from tools.acceleration_response import DEFAULT_ACCELERATION_RESPONSE, validate_response_mode

Point = tuple[float, float]


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def valid_point(point):
    return isinstance(point, (tuple, list)) and len(point) == 2 and all(finite(v) for v in point)


@dataclass(frozen=True)
class Action:
    speed_mps: float
    course_deg: float

    def __post_init__(self):
        if not finite(self.speed_mps) or self.speed_mps < 0:
            raise ValueError('speed_mps must be finite and nonnegative')
        if not finite(self.course_deg):
            raise ValueError('course_deg must be finite')
        object.__setattr__(self, 'course_deg', self.course_deg % 360)


@dataclass(frozen=True)
class Vessel:
    position_m: Point
    speed_mps: float
    course_deg: float
    turn_rate_deg_s: float = 0.

    def __post_init__(self):
        if not valid_point(self.position_m):
            raise ValueError('vessel position must be a finite east/north pair')
        if not finite(self.turn_rate_deg_s) or abs(self.turn_rate_deg_s) > 12.:
            raise ValueError('vessel turn rate must be finite and bounded by 12 deg/s')
        action = Action(self.speed_mps, self.course_deg)
        object.__setattr__(self, 'position_m', tuple(self.position_m))
        object.__setattr__(self, 'course_deg', action.course_deg)


@dataclass(frozen=True)
class Target:
    target_id: int
    position_m: Point | None
    speed_mps: float | None = None
    course_deg: float | None = None
    age_s: float = 0.

    def __post_init__(self):
        if type(self.target_id) is not int:
            raise ValueError('target_id must be an integer')
        if not finite(self.age_s) or self.age_s < 0:
            raise ValueError('target age must be finite and nonnegative')
        if self.position_m is not None:
            if not valid_point(self.position_m):
                raise ValueError('target position must be finite or None')
            object.__setattr__(self, 'position_m', tuple(self.position_m))
        if self.speed_mps is not None and (not finite(self.speed_mps) or self.speed_mps < 0):
            raise ValueError('target speed must be nonnegative or None')
        if self.course_deg is not None:
            if not finite(self.course_deg):
                raise ValueError('target course must be finite or None')
            object.__setattr__(self, 'course_deg', self.course_deg % 360)


@dataclass(frozen=True)
class AvoidanceConfig:
    # Legacy/general forecast length for independent callers. Kev uses the
    # separate warning and action windows below.
    horizon_s: float = 60.
    warning_horizon_s: float = 60.
    action_horizon_s: float = 10.
    control_delay_s: float = 0.
    controller_deceleration_mps2: float = .9
    early_avoidance_margin_s: float = 5.
    dt_s: float = .2
    obstacle_radius_m: float = 80.
    vessel_length_m: float = 11.6
    vessel_width_m: float = 3.4
    safety_margin_m: float = 0.
    uncertainty_mps: float = .5
    max_target_age_s: float = 5.
    unknown_target_speed_mps: float = 30.
    acceleration_response: str = DEFAULT_ACCELERATION_RESPONSE
    acceleration_mps2: float = 1.46
    deceleration_mps2: float = 1.75
    max_turn_rate_deg_s: float = 12.
    max_turn_acceleration_deg_s2: float = 4.
    acceleration_curve: tuple = ()
    deceleration_curve: tuple = ()
    turn_rate_curve: tuple = ()
    heading_step_deg: float = 15.
    # The standalone planner may request a sustained turn across its horizon;
    # actual heading still obeys yaw-rate and angular-acceleration limits.
    planning_interval_s: float = 1.
    clearance_epsilon_m: float = .5
    continuity_slack_deg: float = 15.

    def __post_init__(self):
        zero_allowed = {'control_delay_s', 'safety_margin_m', 'uncertainty_mps', 'max_target_age_s',
                        'clearance_epsilon_m', 'continuity_slack_deg'}
        for name, value in self.__dict__.items():
            if name == 'acceleration_response':
                validate_response_mode(value)
                continue
            if name.endswith('_curve'):
                maximum = dict(acceleration_curve=self.acceleration_mps2, deceleration_curve=self.deceleration_mps2, turn_rate_curve=self.max_turn_rate_deg_s)[name]
                object.__setattr__(self, name, validate_curve(value, maximum))
                continue
            if not finite(value) or value < 0 or (name not in zero_allowed and value == 0):
                raise ValueError(f'invalid {name}')
        for name, maximum in (('acceleration_mps2', 1.46), ('deceleration_mps2', 1.75), ('max_turn_rate_deg_s', 12.), ('max_turn_acceleration_deg_s2', 4.)):
            positive_limit(name, getattr(self, name), maximum)
        positive_limit('controller_deceleration_mps2',self.controller_deceleration_mps2,1.75)
        if self.heading_step_deg > 180:
            raise ValueError('heading_step_deg must be <= 180')
        if self.action_horizon_s > self.warning_horizon_s:
            raise ValueError('action_horizon_s must not exceed warning_horizon_s')

    @property
    def safe_distance_m(self):
        return self.obstacle_radius_m + math.hypot(self.vessel_length_m, self.vessel_width_m)/2 + self.safety_margin_m


@dataclass(frozen=True)
class TargetRisk:
    target_id: int
    min_clearance_m: float
    closest_distance_m: float
    closest_time_s: float
    safe: bool


@dataclass(frozen=True)
class Assessment:
    safe: bool
    min_clearance_m: float | None
    risks: tuple[TargetRisk, ...]
    uncertain_target_ids: tuple[int, ...] = ()
    diagnostics: tuple[str, ...] = ()


@dataclass(frozen=True)
class PlanResult:
    status: Literal['clear', 'avoiding', 'blocked', 'insufficient_data']
    action: Action | None
    assessment: Assessment
    threat_ids: tuple[int, ...]
    reason: str
    emergency_action: Action | None = None
    emergency_assessment: Assessment | None = None
