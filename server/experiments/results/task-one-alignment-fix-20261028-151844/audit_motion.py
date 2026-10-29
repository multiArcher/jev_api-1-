"""Replay recorded commands to audit motion limits and dense static clearance."""
from pathlib import Path
import hashlib
import json
import math
import sys
import numpy as np
ROOT=Path.cwd();sys.path.insert(0,str(ROOT/'jev_api'))
from tools.navigation_motion import predict_ground_motion
from tools.waypoint_geometry import LocalFrame,KNOT_MPS
BASE=Path(__file__).parent
folder=BASE/'task_one'
rows=[json.loads(line) for line in (folder/'trajectory.jsonl').read_text().splitlines()]
summary=json.loads((folder/'summary.json').read_text())
task=json.loads((folder/'task.json').read_text());frame=LocalFrame(task['params']['planWaypointList'])
records=json.loads((folder/'static-obstacles.json').read_text())
centers=np.array([frame.project(row['longitude'],row['latitude']) for row in records])
radius=summary['static_collision_radius_m']
minimum=np.full(len(records),np.inf)
omega=0.;peak_accel=peak_decel=peak_rate=peak_alpha=0.;max_position_error=max_speed_error=max_course_error=0.;measured_steps=0
for before,after in zip(rows,rows[1:]):
    duration=after['time_s']-before['time_s']
    if duration<=0:raise ValueError('non-increasing recorded simulation time')
    motion=predict_ground_motion(before['actual_speed_knots']*KNOT_MPS,before['actual_course_deg'],
        before['command']['speed']*KNOT_MPS,before['command']['course'],0.,0.,duration,
        initial_turn_rate_deg_s=omega,acceleration_response='measured_0_50kn')
    omega=motion['end_turn_rate_deg_s']
    measured_steps+=motion['acceleration_response_used']=='measured_0_50kn'
    times=np.array(motion['times_s']);valid=np.diff(times)>1e-10
    acceleration=np.diff(motion['path_speeds'])[valid]/np.diff(times)[valid]
    angular_acceleration=np.diff(motion['path_turn_rates_deg_s'])[valid]/np.diff(times)[valid]
    peak_accel=max(peak_accel,float(acceleration.max(initial=0.)))
    peak_decel=max(peak_decel,float((-acceleration).max(initial=0.)))
    peak_rate=max(peak_rate,max(map(abs,motion['path_turn_rates_deg_s'])))
    peak_alpha=max(peak_alpha,float(np.abs(angular_acceleration).max(initial=0.)))
    path=np.array(motion['path'])+np.array((before['x'],before['y']))
    max_position_error=max(max_position_error,math.dist(path[-1],(after['x'],after['y'])))
    max_speed_error=max(max_speed_error,abs(motion['end_speed']-after['actual_speed_knots']*KNOT_MPS))
    max_course_error=max(max_course_error,abs((motion['end_course_deg']-after['actual_course_deg']+180)%360-180))
    begin=path[:-1];direction=np.diff(path,axis=0);norm=np.sum(direction**2,axis=1)
    for index,center in enumerate(centers):
        fraction=np.divide(np.sum((center-begin)*direction,axis=1),norm,out=np.zeros_like(norm),where=norm>0)
        closest=begin+np.clip(fraction,0.,1.)[:,None]*direction
        clearance=float(np.linalg.norm(closest-center,axis=1).min()-radius)
        minimum[index]=min(minimum[index],clearance)
manifest=json.loads((BASE/'configuration.json').read_text())
unchanged=all(hashlib.sha256(Path(path).read_bytes()).hexdigest()==value for path,value in manifest['source_sha256'].items())
result=dict(source_hashes_unchanged=unchanged,model_decision='real local Kev',simulation='offline, world paused during inference',
    acceleration_response='measured_0_50kn',measured_response_steps=measured_steps,
    maximum_acceleration_mps2=peak_accel,maximum_braking_mps2=peak_decel,
    maximum_turn_rate_deg_s=peak_rate,maximum_angular_acceleration_deg_s2=peak_alpha,
    maximum_replay_position_error_m=max_position_error,maximum_replay_speed_error_mps=max_speed_error,
    maximum_replay_course_error_deg=max_course_error,
    minimum_static_clearance_m=float(minimum.min()),
    per_obstacle_minimum_clearance_m={str(record['platformId']):float(value) for record,value in zip(records,minimum)},
    constraints_passed=peak_accel<=1.46+1e-7 and peak_decel<=1.75+1e-7 and peak_rate<=12.+1e-7 and peak_alpha<=4.+1e-7,
    replay_matches=unchanged and max_position_error<1e-6 and max_speed_error<1e-7 and max_course_error<1e-6,
    scope='Command replay and numerical geometry audit; not an independent vessel dynamics model or real-vessel measurement')
(BASE/'motion-audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False))
