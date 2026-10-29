"""Local following rollout: no control packets or field endpoints.

PYTHONPATH=jev_api:. python3 -m server.experiments.target_following --mock
Omit --mock to use an already-running Kev service. The mock tests plumbing,
not model quality. Vessel integration is independent of candidate prediction.
"""
import argparse
import json
import math
from pathlib import Path
import time

from server.policies.kev_policy import KevPolicy
from tools.waypoint_geometry import KNOT_MPS

ORIGIN = (120., 30.)


def record(pid, position, speed_knots, course):
    scale = 6378137.*math.pi/180
    return dict(platformId=pid, longitude=ORIGIN[0]+position[0]/(scale*math.cos(math.radians(ORIGIN[1]))),
                latitude=ORIGIN[1]+position[1]/scale, targetSpeed=speed_knots, targetCourse=course)


def mock_choice(payload):
    """Deterministic test fixture; production always asks Kev to select."""
    state = payload['state']
    reference = state['reference_distance_m']
    lower = state['minFollowingDistanceM']
    def score(item):
        end = item['preview_range_m'][2]
        return (max(0., lower-item['range_m'][0])*1000 +
                max(0., -item['radial_braking_margin_m'])*100 +
                abs(end-reference) + .3*abs(item['range_rate_mps']) +
                .01*item['target_heading_error_deg'])
    items = payload['questions']['action']['criteria']
    return dict(answers=dict(action=dict(type='choice', choice=min(items, key=lambda k: score(items[k])))))


def integrate(position, speed, course, action, seconds=1., *, initial_turn_rate_deg_s=0.,
              return_turn_rate=False, config=None):
    """Dense independent midpoint position integration under physical limits.

    Speeds at this compatibility interface are in knots; Vessel uses SI units.
    """
    from server.experiments.dynamic_avoidance import advance
    from server.navigation.dynamic_avoidance import Vessel, Action, AvoidanceConfig
    end, trace = advance(Vessel(position,speed*KNOT_MPS,course,initial_turn_rate_deg_s),
                         Action(action[0]*KNOT_MPS,action[1]),seconds,config or AvoidanceConfig())
    result = (end.position_m,end.speed_mps/KNOT_MPS,end.course_deg,[(x,y) for _,x,y in trace])
    return (*result,end.turn_rate_deg_s) if return_turn_rate else result


def run_scenario(name, policy, steps):
    packet = dict(simId='offline', id='following-'+name, code='targetFollowing', params=dict(
        platCode=1010, targetPlatformId=3001, minFollowingDistanceM=150., maxFollowingDistanceM=250.))
    boat, tracked = (0.,0.), (400.,0.) if name == 'catch_up' else (200.,0.)
    speed, course = 10.,90.
    yaw_rate = 0.
    rows = []
    minimum = float('inf')
    in_band = 0
    for tick in range(steps):
        target_speed = 0. if name == 'target_stop' and tick >= steps//3 else 10.
        target_course = 0. if name == 'target_turn' and tick >= steps//3 else 90.
        own = dict(record(1010, boat, speed, course), _turn_rate_deg_s=yaw_rate)
        target = record(3001, tracked, target_speed, target_course)
        started = time.perf_counter()
        action = policy(dict(packet, situation=(own,target)))
        elapsed = time.perf_counter()-started
        boat, speed, course, points, yaw_rate = integrate(boat,speed,course,action, initial_turn_rate_deg_s=yaw_rate, return_turn_rate=True, config=policy.dynamic_config)
        tv = (target_speed*KNOT_MPS*math.sin(math.radians(target_course)),
              target_speed*KNOT_MPS*math.cos(math.radians(target_course)))
        for index, point in enumerate(points):
            t = index/(len(points)-1)
            distance = math.dist(point,(tracked[0]+tv[0]*t,tracked[1]+tv[1]*t))
            minimum = min(minimum,distance)
        tracked = (tracked[0]+tv[0],tracked[1]+tv[1])
        distance = math.dist(boat,tracked)
        in_band += int(150. <= distance <= 250.)
        rows.append(dict(time_s=tick+1, own_xy_m=boat, target_xy_m=tracked,
            target_speed_knots=target_speed, target_course_deg=target_course,
            speed_knots=speed, course_deg=course, distance_m=distance,
            decision_s=elapsed, selection_mode=policy.local.last_decision['selection_mode']))
    summary = dict(steps=steps, initial_distance_m=400. if name=='catch_up' else 200.,
        final_distance_m=rows[-1]['distance_m'], min_distance_m=minimum,
        in_band_fraction=in_band/steps, below_minimum=minimum < 150.,
        max_decision_s=max(r['decision_s'] for r in rows),
        final_speed_knots=speed, safety_hold_steps=sum(r['selection_mode']=='following_safety_hold' for r in rows))
    return dict(summary=summary,trajectory=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mock', action='store_true')
    parser.add_argument('--steps',type=int,default=60)
    parser.add_argument('--kev-endpoint',default='http://127.0.0.1:18028/v1/systemone')
    parser.add_argument('--output',type=Path,default=Path('/tmp/target-following.json'))
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error('--steps must be positive')
    result = dict(mode='mock' if args.mock else 'kev', scenarios={})
    for name in ('catch_up','target_turn','target_stop'):
        policy = KevPolicy(client=mock_choice if args.mock else None, endpoint=args.kev_endpoint)
        result['scenarios'][name] = run_scenario(name,policy,args.steps)
        print(json.dumps(dict(scenario=name,**result['scenarios'][name]['summary']),allow_nan=False),flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
