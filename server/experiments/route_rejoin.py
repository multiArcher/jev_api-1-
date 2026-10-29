"""Avoidance -> forward route recovery -> sustained tracking, without live control.

Use --mock for fully offline validation; otherwise only local Kev HTTP is used.
Independent 0.02s motion integration checks actual static/dynamic distances.
"""
import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path

from server.experiments.dynamic_avoidance import advance, shifted
from server.experiments.kev_dynamic_avoidance import telemetry
from server.experiments.static_avoidance import closest_heading
from server.navigation.dynamic_avoidance import Action, Target, Vessel
from server.policies.kev_policy import KevPolicy
from tools.waypoint_geometry import EARTH_RADIUS, KNOT_MPS


def run_scene(name, *, mock=True, response='configured', steps=260, output_path=None,
              endpoint='http://127.0.0.1:18028/v1/systemone'):
    targets = {
        'static': (Target(4001, (1200., 0.), 0., 0.),),
        'dynamic': (Target(2001, (600., 0.), 20.*KNOT_MPS, 270.),),
        'mixed': (Target(2001, (600., 0.), 20.*KNOT_MPS, 270.),
                  Target(4001, (1200., 0.), 0., 0.)),
    }[name]
    task = dict(simId='route-rejoin', id=name, code='trackingMode', params=dict(
        platCode=1010, currentRoute=0, planWaypointNum=1, planWaypointList=[dict(
            routeLon=math.degrees(3500./EARTH_RADIUS), routeLat=0., routeSpeed=50., routeRadius=0.)]))
    policy = KevPolicy(endpoint=endpoint, timeout=30., client=closest_heading if mock else None)
    boat = Vessel((0., 0.), (50. if name == 'static' else 20.) * KNOT_MPS, 90.)
    minima = dict(static=None, dynamic=None)
    samples = []
    first_rejoin = first_following = None
    for tick in range(steps):
        own = dict(telemetry(1010, boat.position_m, boat.speed_mps, boat.course_deg), _turn_rate_deg_s=boat.turn_rate_deg_s)
        records = [own, *(telemetry(t.target_id, t.position_m, t.speed_mps, t.course_deg) for t in targets)]
        packet = dict(task, situation=records)
        command = policy(packet)
        if policy.local.route_complete:
            break
        static = policy.validate_control(task, own, records, *command)
        dynamic = policy.validate_dynamic_control(task, own, records, *command)
        assert static['safe'] and dynamic['safe'], (name, tick, static, dynamic)
        decision = policy.local.last_decision
        route = decision['state'].get('route_rejoin')
        if route and route['mode'] == 'rejoining' and first_rejoin is None:
            first_rejoin = tick
        if route and route['mode'] == 'following' and first_following is None:
            first_following = tick
        samples.append(dict(time_s=tick, own=asdict(boat), targets=[asdict(t) for t in targets],
            cross_track_m=abs(boat.position_m[1]),
            route_heading_error_deg=abs((boat.course_deg-90.+180.)%360.-180.),
            command=dict(speed_knots=command[0], course_deg=command[1]),
            static_check=static, dynamic_check=dynamic, decision=decision))
        boat, trace = advance(boat, Action(command[0]*KNOT_MPS, command[1]), 1.,
            replace(policy.dynamic_config,acceleration_response='bounded') if response=='physical' else policy.dynamic_config,
            speed_transition_s=.5 if response == 'configured' else None)
        for target in targets:
            kind = 'static' if target.speed_mps <= .1*KNOT_MPS else 'dynamic'
            clearance = min(math.dist((x, y), shifted(target, time).position_m)
                -policy.obstacle_config.collision_radius_m for time, x, y in trace)
            minima[kind] = clearance if minima[kind] is None else min(minima[kind], clearance)
            assert clearance > 0., (name, response, tick, kind, clearance)
        targets = tuple(shifted(t, 1.) for t in targets)
    tail = samples[-20:]
    max_cross = max(s['cross_track_m'] for s in tail)
    max_heading = max(s['route_heading_error_deg'] for s in tail)
    result = dict(scenario=name, response=response, mock=mock, steps=len(samples),
                  first_rejoin_s=first_rejoin, first_following_s=first_following,
                  rejoin_duration_s=None if first_rejoin is None or first_following is None else first_following-first_rejoin,
                  last_20s_max_cross_track_m=max_cross, last_20s_max_heading_error_deg=max_heading,
                  min_clearance_m=minima, route_complete=policy.local.route_complete, samples=samples)
    # Preserve the trace even if an acceptance assertion fails.
    if output_path is not None:
        output_path.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    assert first_rejoin is not None and first_following is not None, (name, response, 'never rejoined')
    assert len(tail) == 20 and tail[0]['time_s'] > first_following, (name, 'tracking duration too short')
    assert max_cross <= policy.route_rejoin_config.cross_track_tolerance_m, (name, response, max_cross)
    assert max_heading <= policy.route_rejoin_config.heading_tolerance_deg, (name, response, max_heading,
        [(s['time_s'], round(s['cross_track_m'], 2), s['own']['course_deg'],
          s['decision']['state'].get('route_rejoin', {}).get('mode')) for s in tail])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mock', action='store_true')
    parser.add_argument('--kev-endpoint', default='http://127.0.0.1:18028/v1/systemone')
    parser.add_argument('--scenario', choices=('all', 'static', 'dynamic', 'mixed'), default='all')
    parser.add_argument('--response', choices=('configured', 'physical', 'both'), default='both')
    parser.add_argument('--steps', type=int, default=260)
    parser.add_argument('--output', type=Path, default=Path('/tmp/kev-route-rejoin'))
    args = parser.parse_args()
    if args.steps < 1:
        parser.error('--steps must be positive')
    args.output.mkdir(parents=True, exist_ok=True)
    summary = []
    for response in (('configured', 'physical') if args.response == 'both' else (args.response,)):
        for name in (('static', 'dynamic', 'mixed') if args.scenario == 'all' else (args.scenario,)):
            result = run_scene(name, mock=args.mock, response=response, steps=args.steps, endpoint=args.kev_endpoint,
                               output_path=args.output/f'{name}-{response}.json')
            row = {k: v for k, v in result.items() if k != 'samples'}
            summary.append(row)
            print(json.dumps(row, ensure_ascii=False), flush=True)
    (args.output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')


if __name__ == '__main__':
    main()
