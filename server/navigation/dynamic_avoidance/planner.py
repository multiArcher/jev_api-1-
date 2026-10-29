"""Deterministic right-turn-first selection, independent of models or I/O."""
import math
from .types import Action, AvoidanceConfig, PlanResult
from tools.vessel_dynamics import MAX_SPEED_MPS
from .prediction import assess_action, signed_turn


class DynamicAvoidancePlanner:
    """One instance per task. Calls must be sequential; reset between tasks."""
    def __init__(self, config=None):
        self.config = config or AvoidanceConfig()
        self.reset()

    def reset(self):
        self._previous_side = 0

    def _candidates(self, own, desired):
        config = self.config
        cap = min(179., config.max_turn_rate_deg_s*config.horizon_s)
        offsets = {0., cap, -cap}
        for i in range(1, math.floor(cap/config.heading_step_deg)+1):
            offsets.update((i*config.heading_step_deg, -i*config.heading_step_deg))
        desired_turn = signed_turn(own.course_deg, desired.course_deg)
        if abs(desired_turn) <= cap:
            offsets.add(desired_turn)
        speeds = {0., desired.speed_mps, desired.speed_mps*.25,
                  desired.speed_mps*.5, desired.speed_mps*.75,
                  min(own.speed_mps, desired.speed_mps)}
        return tuple(Action(speed, own.course_deg+offset)
                     for speed in sorted(speeds, reverse=True) for offset in sorted(offsets)
                     if speed > 0 or offset == 0)

    def plan(self, own, targets, desired, *, candidates=None):
        """Optionally restrict selection to externally admissible actions."""
        desired = Action(min(desired.speed_mps, MAX_SPEED_MPS), desired.course_deg)
        targets = tuple(targets)
        candidates = None if candidates is None else tuple(a for a in candidates if a.speed_mps <= MAX_SPEED_MPS)
        if len({t.target_id for t in targets}) != len(targets):
            raise ValueError('targets must have unique IDs')
        config = self.config
        initial = assess_action(own, targets, desired, config)
        threats = tuple(sorted({r.target_id for r in initial.risks if not r.safe}
                               | set(initial.uncertain_target_ids)))
        if initial.safe and (candidates is None or desired in candidates):
            self.reset()
            return PlanResult('clear', desired, initial, (), '期望动作在预测范围内安全')
        evaluated = [(action, assess_action(own, targets, action, config))
                     for action in (self._candidates(own, desired) if candidates is None else candidates)]
        safe = [(action, assessment) for action, assessment in evaluated if assessment.safe]

        def side(action):
            turn = signed_turn(own.course_deg, action.course_deg)
            return 1 if turn > .001 else (-1 if turn < -.001 else 0)

        def deviation(action):
            return abs(signed_turn(desired.course_deg, action.course_deg))

        if safe:
            # Hysteresis applies only among similarly deviating safe actions.
            best_deviation = min(deviation(a) for a, _ in safe)
            continuous = [(a, check) for a, check in safe
                          if self._previous_side and side(a) == self._previous_side
                          and deviation(a) <= best_deviation+config.continuity_slack_deg]
            pool = continuous or safe
            action, assessment = min(pool, key=lambda item: (
                {1: 0, 0: 1, -1: 2}[side(item[0])], deviation(item[0]),
                abs(desired.speed_mps-item[0].speed_mps),
                -(item[1].min_clearance_m or 0.)))
            self._previous_side = side(action)
            return PlanResult('avoiding', action, assessment, threats,
                              '选择通过静动态轨迹检查的避让动作，优先右转或保持避让方向')

        # No candidate is certified safe. Expose the least-risk evaluated action
        # separately, including its failures; a stop is never presumed safe.
        if not evaluated:
            return PlanResult('insufficient_data' if initial.uncertain_target_ids else 'blocked',
                              None, initial, threats, '没有同时满足外部约束的候选动作')
        emergency, check = max(evaluated, key=lambda item: (
            -len(item[1].uncertain_target_ids),
            item[1].min_clearance_m if item[1].min_clearance_m is not None else -float('inf'),
            -item[0].speed_mps))
        uncertain = bool(initial.uncertain_target_ids or check.uncertain_target_ids)
        return PlanResult('insufficient_data' if uncertain else 'blocked', None, initial,
                          threats, '没有可确认安全的候选动作；应急建议不代表安全', emergency, check)
