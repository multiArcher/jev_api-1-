"""Offline scenarios, independent dense verification, and a replanning loop.

PYTHONPATH=jev_api:. python3 -m server.experiments.dynamic_avoidance --output /tmp/dynamic-avoidance.json
"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
from tools.vessel_dynamics import turn_profile, step_acceleration_limit, MAX_SPEED_MPS
from tools.acceleration_response import response_applies, measured_speed_profile

from server.navigation.dynamic_avoidance import (
    Action, AvoidanceConfig, DynamicAvoidancePlanner, Target, Vessel, forecast)


def advance(own, action, seconds, config, *, speed_transition_s=None):
    """Independent midpoint position integration of the shared physical limits.

    The legacy speed_transition_s argument cannot bypass acceleration limits.
    """
    x, y = own.position_m
    requested = min(action.speed_mps, MAX_SPEED_MPS)
    difference = (action.course_deg-own.course_deg+180) % 360-180 if requested else own.turn_rate_deg_s*abs(own.turn_rate_deg_s)/(2*config.max_turn_acceleration_deg_s2)
    rate = min([config.max_turn_rate_deg_s]+[p[1] for p in config.turn_rate_curve])
    yaw, _ = turn_profile(difference, own.turn_rate_deg_s, rate, config.max_turn_acceleration_deg_s2)
    steps = max(1, math.ceil(seconds/.02))
    dt = seconds/steps
    positions = [(0., x, y)]
    speed, course = own.speed_mps, own.course_deg
    increasing = requested > speed
    cap = config.acceleration_mps2 if increasing else config.deceleration_mps2
    curve = config.acceleration_curve if increasing else config.deceleration_curve
    use_measured = response_applies(config.acceleration_response,speed,requested,own.course_deg,action.course_deg,own.turn_rate_deg_s)
    if use_measured:
        response_speed,_ = measured_speed_profile(speed,requested,cap,curve)
    for i in range(steps):
        limit = step_acceleration_limit(curve,speed,requested,cap,dt)
        next_speed = response_speed((i+1)*dt) if use_measured else (min(requested,speed+limit*dt) if increasing else max(requested,speed-limit*dt))
        mid = (i+.5)*dt
        v = (speed+next_speed)/2
        heading = math.radians(own.course_deg+yaw(mid)[0])
        x += v*math.sin(heading)*dt
        y += v*math.cos(heading)*dt
        end = (i+1)*dt
        speed = next_speed
        course = (own.course_deg+yaw(end)[0]) % 360
        positions.append((end, x, y))
    return Vessel((x,y), speed, course, yaw(seconds)[1]), positions


def verify(own, targets, action, config):
    """Independently roll out action and braking at 0.02s, checking each target.

    Returns dense geometric clearance, including the same configured uncertainty
    envelope, rather than reusing planner clearance or relative-segment code.
    """
    requested=min(action.speed_mps,MAX_SPEED_MPS)
    curve=config.acceleration_curve if requested>own.speed_mps else config.deceleration_curve
    cap=config.acceleration_mps2 if requested>own.speed_mps else config.deceleration_mps2
    transition=abs(requested-own.speed_mps)/min([cap]+[p[1] for p in curve])
    if response_applies(config.acceleration_response,own.speed_mps,requested,own.course_deg,action.course_deg,own.turn_rate_deg_s):
        _,response_times = measured_speed_profile(own.speed_mps,requested,cap,curve)
        transition=max(transition,response_times[-1])
    _,turns=turn_profile((action.course_deg-own.course_deg+180)%360-180 if action.speed_mps else own.turn_rate_deg_s*abs(own.turn_rate_deg_s)/(2*config.max_turn_acceleration_deg_s2),own.turn_rate_deg_s,min([config.max_turn_rate_deg_s]+[p[1] for p in config.turn_rate_curve]),config.max_turn_acceleration_deg_s2)
    turning=max(turns,default=0.)
    hold = max(config.horizon_s, transition, turning)
    end, points = advance(own, action, hold, config)
    if action.speed_mps:
        _, tail = advance(end, Action(0., end.course_deg), end.speed_mps/min([config.deceleration_mps2]+[p[1] for p in config.deceleration_curve]), config)
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
