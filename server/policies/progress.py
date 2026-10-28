"""Per-task progress supervision; no action selection or external packet fields."""
from collections import deque
from dataclasses import dataclass
import math


class NavigationStalled(ValueError):
    """Terminal navigation failure: the control runtime must send zero speed."""
    error_code = 'NAVIGATION_NO_PROGRESS'


@dataclass(frozen=True)
class ProgressConfig:
    window: int = 5
    recovery_limit: int = 20
    min_progress_m: float = 1.0
    min_heading_improvement_deg: float = 2.0
    aligned_deg: float = 30.0

    def __post_init__(self):
        for name in ('window', 'recovery_limit'):
            if type(getattr(self, name)) is not int or getattr(self, name) < 1:
                raise ValueError(f'{name} must be a positive integer')
        for name in ('min_progress_m', 'min_heading_improvement_deg', 'aligned_deg'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive and finite')


class ProgressSupervisor:
    def __init__(self, config=None):
        self.config = config or ProgressConfig()
        self.index = None
        self.samples = deque(maxlen=self.config.window + 1)
        self.mode = 'normal'
        self.recovery_cycles = 0

    def update(self, index, distance_m, heading_error_deg, *, require_distance_progress=False):
        if index != self.index:
            self.index = index
            self.samples.clear()
            self.mode, self.recovery_cycles = 'normal', 0
        self.samples.append((distance_m, heading_error_deg))
        if len(self.samples) < self.samples.maxlen:
            return self.state()
        distance_change = distance_m - self.samples[0][0]
        heading_gain = self.samples[0][1] - heading_error_deg
        if self.mode == 'recovery':
            if distance_change <= -self.config.min_progress_m and heading_error_deg <= self.config.aligned_deg:
                self.mode, self.recovery_cycles = 'normal', 0
                self.samples.clear()
            elif (heading_gain >= self.config.min_heading_improvement_deg or
                  distance_change <= -self.config.min_progress_m):
                self.recovery_cycles = 0
            else:
                self.recovery_cycles += 1
                if self.recovery_cycles >= self.config.recovery_limit:
                    raise NavigationStalled(
                        f'航点{index}低速恢复连续{self.recovery_cycles}轮无改善；'
                        f'距离{distance_m:.1f}米，航向误差{heading_error_deg:.1f}度')
        elif heading_gain < self.config.min_heading_improvement_deg and (
                distance_change >= self.config.min_progress_m or
                (distance_change > -self.config.min_progress_m and
                 (heading_error_deg > self.config.aligned_deg or require_distance_progress))):
            self.mode, self.recovery_cycles = 'recovery', 0
        return self.state()

    def state(self):
        return dict(mode=self.mode, recovery_cycles=self.recovery_cycles)
