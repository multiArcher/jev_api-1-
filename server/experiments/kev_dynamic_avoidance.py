"""Separated static/dynamic Kev integration: offline closed-loop validation.

Use --mock for a deterministic client; otherwise calls only the local Kev HTTP
endpoint. No live task or control packets are sent. Inference pauses world time.
"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path

from server.experiments.dynamic_avoidance import advance, shifted
from server.experiments.static_avoidance import closest_heading
from server.navigation.dynamic_avoidance import Action, Target, Vessel
from server.policies.kev_policy import KevPolicy
from tools.waypoint_geometry import EARTH_RADIUS, KNOT_MPS


def telemetry(pid,position,speed,course):
    return dict(platformId=pid,longitude=math.degrees(position[0]/EARTH_RADIUS),
        latitude=math.degrees(position[1]/EARTH_RADIUS),targetSpeed=speed/KNOT_MPS,
        targetCourse=course,_received_age_s=0.)


def run_scene(name,targets,*,steps=50,mock=True,endpoint='http://127.0.0.1:18028/v1/systemone',response='configured'):
    task=dict(simId='separate-avoidance',id=name,code='trackingMode',params=dict(
        platCode=1010,currentRoute=0,planWaypointNum=1,planWaypointList=[
            dict(routeLon=math.degrees(3000/EARTH_RADIUS),routeLat=0.,routeSpeed=50.,routeRadius=0.)]))
    policy=KevPolicy(endpoint=endpoint,timeout=30.,client=closest_heading if mock else None)
    boat=Vessel((0.,0.),20*KNOT_MPS,90.)
    minima=dict(static=None,dynamic=None)
    samples=[]
    for tick in range(steps):
        own=telemetry(1010,boat.position_m,boat.speed_mps,boat.course_deg)
        records=[own,*(telemetry(t.target_id,t.position_m,t.speed_mps,t.course_deg) for t in targets)]
        speed,course=policy(dict(task,situation=records))
        static=policy.validate_control(task,own,records,speed,course)
        dynamic=policy.validate_dynamic_control(task,own,records,speed,course)
        assert static['safe'] and dynamic['safe'],(name,response,tick,static,dynamic)
        samples.append(dict(time_s=tick,own=asdict(boat),targets=[asdict(t) for t in targets],
            command=dict(speed_knots=speed,course_deg=course),static_check=static,dynamic_check=dynamic,
            decision=policy.local.last_decision))
        boat,trace=advance(boat,Action(speed*KNOT_MPS,course),1.,policy.dynamic_config,
            speed_transition_s=.5 if response=='configured' else None)
        for target in targets:
            kind='static' if target.speed_mps<=.1*KNOT_MPS else 'dynamic'
            clearance=min(math.dist((x,y),shifted(target,t).position_m)
                -policy.obstacle_config.collision_radius_m for t,x,y in trace)
            minima[kind]=clearance if minima[kind] is None else min(minima[kind],clearance)
            assert clearance>0,(name,response,tick,kind,clearance)
        targets=tuple(shifted(t,1.) for t in targets)
    return dict(scenario=name,response=response,mock=mock,steps=steps,min_clearance_m=minima,
        model_calls=sum('inference_s' in s['decision'] for s in samples),samples=samples)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mock',action='store_true')
    parser.add_argument('--kev-endpoint',default='http://127.0.0.1:18028/v1/systemone')
    parser.add_argument('--steps',type=int,default=50)
    parser.add_argument('--response',choices=('configured','physical','both'),default='both')
    parser.add_argument('--scenario',choices=('all','head_on','crossing','overtaking','mixed_near','mixed_far'),default='all')
    parser.add_argument('--output',type=Path,default=Path('/tmp/kev-separated-avoidance'))
    args=parser.parse_args()
    if args.steps<1:parser.error('--steps must be positive')
    scenarios={
        'head_on':(Target(2001,(600.,0.),20*KNOT_MPS,270.),),
        'crossing':(Target(2001,(300.,300.),20*KNOT_MPS,180.),),
        'overtaking':(Target(2001,(200.,0.),5*KNOT_MPS,90.),),
        'mixed_near':(Target(2001,(600.,0.),20*KNOT_MPS,270.),Target(4001,(60.,-85.),0.,0.)),
        'mixed_far':(Target(2001,(600.,0.),20*KNOT_MPS,270.),Target(4001,(1200.,0.),0.,0.)),
    }
    args.output.mkdir(parents=True,exist_ok=True)
    summary=[]
    for response in (('configured','physical') if args.response=='both' else (args.response,)):
        for name,targets in scenarios.items():
            if args.scenario!='all' and name!=args.scenario:
                continue
            result=run_scene(name,targets,steps=args.steps,mock=args.mock,endpoint=args.kev_endpoint,response=response)
            (args.output/f'{name}-{response}.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
            row={k:v for k,v in result.items() if k!='samples'}
            summary.append(row)
            print(json.dumps(row),flush=True)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')


if __name__=='__main__':main()
