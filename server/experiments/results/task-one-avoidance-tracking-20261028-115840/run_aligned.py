from pathlib import Path
import hashlib
import json
import math
import sys
from dataclasses import asdict
from urllib.request import urlopen

ROOT=Path.cwd()
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'jev_api'))
from server.experiments.field_tasks import load_task,TestConfig,run_task
from tools.waypoint_geometry import LocalFrame
from tools.vessel_dynamics import limits_state

task=load_task('task_one')
frame=LocalFrame(task['params']['planWaypointList'])
p1,p2=[frame.project(point['routeLon'],point['routeLat']) for point in task['params']['planWaypointList'][:2]]
dx,dy=p2[0]-p1[0],p2[1]-p1[1]
length=math.hypot(dx,dy)
heading=math.degrees(math.atan2(dx,dy))%360
config=TestConfig(steps=6000,dt=1.,initial_position_xy_m=(-150*dx/length,-150*dy/length),initial_course_deg=heading)
output=Path(__file__).parent/'aligned-entry'
output.mkdir(parents=True,exist_ok=True)
obstacles=json.loads((ROOT/'server/experiments/tasks/static_situation.json').read_text())
with urlopen('http://127.0.0.1:18028/v1/models',timeout=10) as response:
    (output/'model-service.json').write_bytes(response.read())
manifest=dict(scenario='首航段反向延长线外150米进场；初始航向沿P1到P2方向',config=asdict(config),limits=limits_state(),acceleration_response='measured_0_50kn',static_obstacles=len(obstacles),source_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in [ROOT/'server/policies/kev_policy.py',ROOT/'jev_api/tools/navigation_motion.py',ROOT/'jev_api/tools/data/acceleration_0_50kn.json',ROOT/'server/experiments/tasks/task_one.json',ROOT/'server/experiments/tasks/static_situation.json']})
(output/'configuration.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(manifest,ensure_ascii=False),flush=True)
result=run_task('task_one',output,config,static_obstacles=obstacles,checkpoint_every=3)
print(json.dumps(result,ensure_ascii=False),flush=True)
raise SystemExit(0 if result['passed'] else 1)
