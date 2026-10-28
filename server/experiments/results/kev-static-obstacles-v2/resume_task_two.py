"""Continue the real task after its recorded inference-timeout stop.

This is a separate continuation, not evidence of an uninterrupted full run.
"""
import json
from pathlib import Path
import sys
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT))
from server.experiments import field_tasks as ft

OUT=Path(__file__).resolve().parent
rows=[json.loads(line) for line in (OUT/'task_two/trajectory.jsonl').open()]
last=rows[-1]
assert last['actual_speed_knots']<=.1 and last['command']['speed']==0
original=ft.load_task('task_two')
original['id']+='-after-timeout'
original['params']['currentRoute']=last['completed_waypoints']
obstacles=json.loads((ROOT/'server/experiments/tasks/static_situation.json').read_text())
config=ft.TestConfig(steps=6000,initial_position_xy_m=(last['x'],last['y']),
                    initial_course_deg=last['actual_course_deg'],
                    kev_endpoint='http://127.0.0.1:18029/v1/systemone',kev_timeout=30.)
with patch.object(ft,'load_task',return_value=original):
    result=ft.run_task('task_two_resume',OUT,config,static_obstacles=obstacles)
result['initial_completed_waypoints']=last['completed_waypoints']
result['continuation_from']='task_two/trajectory.jsonl'
result['continuation_reason']='Original local inference service timed out; resume from its confirmed stop.'
(OUT/'task_two_resume/summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False,indent=2))
