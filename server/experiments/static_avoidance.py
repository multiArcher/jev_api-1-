"""Static cruise/late-turn regression, with local Kev or a deterministic client.

Only the model HTTP endpoint is used; no live control packets are sent.
"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path

from server.experiments.dynamic_avoidance import advance
from server.navigation.dynamic_avoidance import Action, AvoidanceConfig, Vessel
from server.policies.kev_policy import KevPolicy
from tools.waypoint_geometry import EARTH_RADIUS, KNOT_MPS


def closest_heading(payload):
    options=payload['questions']['action']['criteria']
    key=min(options,key=lambda k:(options[k]['target_heading_error_deg'],-options[k]['speed_knots']))
    return dict(answers=dict(action=dict(type='choice',choice=key)))


def record(pid,position,speed,course):
    return dict(platformId=pid,longitude=math.degrees(position[0]/EARTH_RADIUS),
        latitude=math.degrees(position[1]/EARTH_RADIUS),targetSpeed=speed/KNOT_MPS,targetCourse=course)


def run(*,endpoint='http://127.0.0.1:18028/v1/systemone',steps=100,mock=False):
    task=dict(simId='static-cruise',id='late-turn',code='trackingMode',params=dict(
        platCode=1010,currentRoute=0,planWaypointNum=1,planWaypointList=[
            dict(routeLon=math.degrees(3500/EARTH_RADIUS),routeLat=0.,routeSpeed=50.,routeRadius=0.)]))
    obstacle=(1200.,0.)
    policy=KevPolicy(endpoint=endpoint,timeout=30.,client=closest_heading if mock else None)
    boat=Vessel((0.,0.),50*KNOT_MPS,90.)
    physics=AvoidanceConfig()
    minimum=float('inf');first_turn=None;samples=[]
    for tick in range(steps):
        own=dict(record(1010,boat.position_m,boat.speed_mps,boat.course_deg),_turn_rate_deg_s=boat.turn_rate_deg_s)
        records=[own,record(4001,obstacle,0.,0.)]
        speed,course=policy(dict(task,situation=records))
        check=policy.validate_control(task,own,records,speed,course)
        assert check['safe'], (tick,check)
        if first_turn is None and abs((course-90+180)%360-180)>.01:
            first_turn=dict(time_s=tick,center_distance_m=math.dist(boat.position_m,obstacle))
        samples.append(dict(time_s=tick,own=asdict(boat),speed_knots=speed,course_deg=course,
                            decision=policy.local.last_decision))
        boat,trace=advance(boat,Action(speed*KNOT_MPS,course),1.,physics,speed_transition_s=.5)
        minimum=min(minimum,*(math.dist((x,y),obstacle)-policy.obstacle_config.collision_radius_m
                              for _,x,y in trace))
        assert minimum>0, (tick,minimum)
    assert first_turn is not None and first_turn['center_distance_m'] < 550., first_turn
    assert boat.position_m[0]>obstacle[0]+200., boat
    return dict(mock=mock,steps=steps,first_turn=first_turn,min_clearance_m=minimum,
                min_command_speed_knots=min(s['speed_knots'] for s in samples),samples=samples)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kev-endpoint',default='http://127.0.0.1:18028/v1/systemone')
    parser.add_argument('--steps',type=int,default=100)
    parser.add_argument('--mock',action='store_true')
    parser.add_argument('--output',type=Path,default=Path('/tmp/static-late-turn.json'))
    args=parser.parse_args()
    result=run(endpoint=args.kev_endpoint,steps=args.steps,mock=args.mock)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='samples'}))


if __name__=='__main__':
    main()
