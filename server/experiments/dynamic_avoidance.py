"""Offline scenarios, independent dense verification, and a replanning loop.

PYTHONPATH=jev_api:. python3 -m server.experiments.dynamic_avoidance --output /tmp/dynamic-avoidance.json
"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path

from server.navigation.dynamic_avoidance import (
    Action, AvoidanceConfig, DynamicAvoidancePlanner, Target, Vessel, forecast)


def advance(own, action, seconds, config, *, speed_transition_s=None):
    """Independent midpoint integrator; does not call the planner's predictor."""
    x, y = own.position_m
    difference = (action.course_deg-own.course_deg+180) % 360-180 if action.speed_mps else 0.
    acceleration = config.acceleration_mps2 if action.speed_mps > own.speed_mps else -config.deceleration_mps2
    def speed_at(t):
        if speed_transition_s is not None:
            fraction = min(1., t/speed_transition_s) if speed_transition_s else 1.
            return own.speed_mps+(action.speed_mps-own.speed_mps)*fraction
        value = own.speed_mps+acceleration*t
        return min(value, action.speed_mps) if acceleration > 0 else max(value, action.speed_mps)
    steps = max(1, math.ceil(seconds/.02))
    dt = seconds/steps
    positions = [(0., x, y)]
    speed, course = own.speed_mps, own.course_deg
    for i in range(steps):
        mid = (i+.5)*dt
        v = speed_at(mid)
        turn = math.copysign(min(abs(difference), config.max_turn_rate_deg_s*mid), difference)
        heading = math.radians(own.course_deg+turn)
        x += v*math.sin(heading)*dt
        y += v*math.cos(heading)*dt
        end = (i+1)*dt
        speed = speed_at(end)
        course = (own.course_deg+math.copysign(min(abs(difference), config.max_turn_rate_deg_s*end), difference)) % 360
        positions.append((end, x, y))
    return Vessel((x, y), speed, course), positions


def verify(own, targets, action, config):
    """Independently roll out action and braking at 0.02s, checking each target.

    Returns dense geometric clearance, including the same configured uncertainty
    envelope, rather than reusing planner clearance or relative-segment code.
    """
    transition = abs(action.speed_mps-own.speed_mps)/(
        config.acceleration_mps2 if action.speed_mps > own.speed_mps else config.deceleration_mps2)
    turning = abs((action.course_deg-own.course_deg+180) % 360-180)/config.max_turn_rate_deg_s if action.speed_mps else 0.
    hold = max(config.horizon_s, transition, turning)
    end, points = advance(own, action, hold, config)
    if action.speed_mps:
        _, tail = advance(end, Action(0., end.course_deg), action.speed_mps/config.deceleration_mps2, config)
        points += [(hold+t, x, y) for t, x, y in tail[1:]]
    minimum = float('inf')
    for time, x, y in points:
        for target in targets:
            heading = math.radians(target.course_deg or 0.)
            elapsed = time+target.age_s
            tx = target.position_m[0]+target.speed_mps*math.sin(heading)*elapsed
            ty = target.position_m[1]+target.speed_mps*math.cos(heading)*elapsed
            distance = math.hypot(x-tx, y-ty)
            minimum = min(minimum, distance-config.safe_distance_m-config.uncertainty_mps*elapsed)
    return minimum


def shifted(target, seconds):
    heading = math.radians(target.course_deg or 0.)
    return Target(target.target_id, (
        target.position_m[0]+target.speed_mps*math.sin(heading)*seconds,
        target.position_m[1]+target.speed_mps*math.cos(heading)*seconds),
        target.speed_mps, target.course_deg)


def run():
    config = AvoidanceConfig()
    own = Vessel((0., 0.), 10., 0.)
    desired = Action(10., 0.)
    scenarios = {
        'head_on': (Target(1, (0., 600.), 10., 180.),),
        'crossing': (Target(1, (300., 300.), 10., 270.),),
        'overtaking': (Target(1, (0., 200.), 3., 0.),),
        'right_blocked': (Target(1, (0., 600.), 10., 180.), Target(2, (90., 100.), 0., 0.)),
    }
    results = {}
    for name, targets in scenarios.items():
        result = DynamicAvoidancePlanner(config).plan(own, targets, desired)
        assert result.action is not None, (name, result)
        clearance = verify(own, targets, result.action, config)
        assert clearance > 0, (name, clearance)
        path = forecast(own, result.action, config)
        results[name] = dict(result=asdict(result), independent_min_clearance_m=clearance,
            times_s=path.times_s, own_path_m=path.positions_m,
            target_paths_m={str(target.target_id): [shifted(target, t).position_m for t in path.times_s]
                            for target in targets})

    # Fresh telemetry each second, one task-local planner, continuously moving
    # target; this loop is deliberately independent of all backend/control code.
    planner = DynamicAvoidancePlanner(config)
    boat, targets = own, scenarios['head_on']
    samples, actual_minimum = [], float('inf')
    for tick in range(75):
        result = planner.plan(boat, targets, desired)
        assert result.action is not None, (tick, result)
        checked = verify(boat, targets, result.action, config)
        assert checked > 0, (tick, checked)
        before = boat
        boat, trace = advance(boat, result.action, 1., config)
        for t, x, y in trace:
            actual_minimum = min(actual_minimum, *(math.dist((x, y), shifted(target, t).position_m)
                                                  -config.safe_distance_m for target in targets))
        samples.append(dict(time_s=tick, own=asdict(before), targets=[asdict(t) for t in targets],
                            status=result.status, action=asdict(result.action),
                            independent_forecast_clearance_m=checked))
        targets = tuple(shifted(target, 1.) for target in targets)
    assert actual_minimum > 0, actual_minimum
    results['closed_loop'] = dict(samples=samples, actual_min_clearance_m=actual_minimum)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('/tmp/dynamic-avoidance.json'))
    args = parser.parse_args()
    results = run()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(results, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(f'Verified {len(results)-1} scenarios and 75 closed-loop steps: {args.output}')


if __name__ == '__main__':
    main()
