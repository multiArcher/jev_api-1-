"""Replay a field snapshot with controller delay and slower braking; no UDP I/O."""
import argparse
from dataclasses import asdict,replace
import json
import math
from pathlib import Path

from server.experiments.field_tasks import load_task,_telemetry
from server.experiments.dynamic_avoidance import advance,shifted
from server.experiments.static_avoidance import closest_heading
from server.navigation.dynamic_avoidance import Action,Target,Vessel
from server.policies.kev_policy import KevPolicy
from tools.waypoint_geometry import KNOT_MPS,xy


def replay(*,mock=True,delay_steps=1,deceleration_mps2=1.,steps=180,snapshot="rejoin_near_p2"):
    fixture=json.loads((Path(__file__).parents[1]/f'tests/fixtures/{snapshot}.json').read_text())
    task=load_task('six_waypoint_northwest_20261028');task['params']['next_waypoint_index']=1
    origin=tuple(task['params']['planWaypointList'][0][k] for k in ('routeLon','routeLat'))
    initial=fixture['own'];position=xy((initial['longitude'],initial['latitude']),origin)
    own=Vessel(position,initial['speed_knots']*KNOT_MPS,initial['course_deg'])
    targets=tuple(shifted(Target(t['id'],tuple(t['position_xy_m']),t['speed_mps'],t['course_deg']),t['age_s']) for t in fixture['targets'])
    policy=KevPolicy(client=closest_heading if mock else None)
    physics=replace(policy.dynamic_config,deceleration_mps2=deceleration_mps2,acceleration_response='bounded')
    queue=[Action(own.speed_mps,own.course_deg)]*delay_steps
    minimum=float('inf');arrived=None;samples=[]
    centers=[xy((p['routeLon'],p['routeLat']),origin) for p in task['params']['planWaypointList']]
    for tick in range(steps):
        records=[dict(_telemetry(origin,*own.position_m,own.speed_mps,own.course_deg,1010),_turn_rate_deg_s=own.turn_rate_deg_s)]
        records.extend(_telemetry(origin,*t.position_m,t.speed_mps,t.course_deg,t.target_id) for t in targets)
        packet=dict(task,situation=records)
        speed,heading=policy(packet)
        action=Action(speed*KNOT_MPS,heading)
        check=policy.validate_dynamic_control(packet,records[0],records,speed,heading)
        assert check['safe'],(tick,'dynamic veto',check)
        queue.append(action);applied=queue.pop(0)
        samples.append(dict(time_s=tick,own=asdict(own),targets=[asdict(t) for t in targets],
            command=asdict(action),applied=asdict(applied),decision=policy.local.last_decision))
        before=own
        own,trace=advance(own,applied,1.,physics)
        for tm,x,y in trace:
            for target in targets:
                clearance=math.dist((x,y),shifted(target,tm).position_m)-policy.obstacle_config.collision_radius_m
                minimum=min(minimum,clearance)
                assert clearance>0,(tick,'collision',clearance)
            # Speed evolution is monotonic within one command interval.
            v=min(applied.speed_mps,before.speed_mps+physics.acceleration_mps2*tm) if applied.speed_mps>=before.speed_mps else max(applied.speed_mps,before.speed_mps-physics.deceleration_mps2*tm)
            for center,point in zip(centers,task['params']['planWaypointList']):
                if math.dist((x,y),center)<=100.:
                    assert v<=point['routeSpeed']*KNOT_MPS+1e-6,(tick,'waypoint speed',v/KNOT_MPS)
        targets=tuple(shifted(t,1.) for t in targets)
        if policy.current_route()>=2:
            arrived=tick
            break
    assert arrived is not None,'P2 was not reached'
    return dict(snapshot=snapshot,mock=mock,delay_s=delay_steps,deceleration_mps2=deceleration_mps2,
                min_clearance_m=minimum,p2_arrival_s=arrived,samples=samples)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--real',action='store_true')
    parser.add_argument('--snapshot',choices=('rejoin_near_p2','rejoin_approach_p2'),default='rejoin_near_p2')
    parser.add_argument('--output',type=Path,default=Path('/tmp/moving-rejoin.json'))
    args=parser.parse_args()
    result=replay(mock=not args.real,snapshot=args.snapshot)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='samples'}))


if __name__=='__main__':main()
