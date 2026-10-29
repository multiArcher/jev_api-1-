"""Render final recorded results, without model or control calls."""
import json,math,csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
root=Path(__file__).resolve().parent
out=root/'task_two'
summary=json.loads((out/'summary.json').read_text())
task=json.loads((out/'task.json').read_text())
obstacles=json.loads((out/'static-obstacles.json').read_text())
rows=[]
for line in (out/'trajectory.jsonl').open():
 r=json.loads(line)
 rows.append({k:r[k] for k in ('time_s','x','y','actual_speed_knots','completed_waypoints','target_index','command')})
font=Path('/usr/share/fonts/truetype/SimHei.ttf')
if font.exists():font_manager.fontManager.addfont(str(font));plt.rcParams['font.family']='SimHei'
plt.rcParams.update({'axes.unicode_minus':False,'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
wp=task['params']['planWaypointList'];lon0=wp[0]['routeLon'];lat0=wp[0]['routeLat']
def xy(lon,lat):return ((lon-lon0)*math.pi/180*6378137*math.cos(math.radians(lat0)),(lat-lat0)*math.pi/180*6378137)
centers=[xy(o['longitude'],o['latitude']) for o in obstacles]
net=[min(math.dist((r['x'],r['y']),c) for c in centers)-summary['static_collision_radius_m'] for r in rows]
times=[r['time_s'] for r in rows]
fig,axes=plt.subplots(3,1,figsize=(14,10),sharex=True)
fig.subplots_adjust(top=.87,bottom=.09,left=.09,right=.97,hspace=.25)
status='通过' if summary['passed'] else '未通过'
fig.suptitle(f'任务二：45航点＋24静态目标｜{status}',fontsize=21,y=.97)
fig.text(.5,.918,f"到达 {summary['reached_waypoints']}/45　碰撞 {summary['static_collision_count']}次　航点限速违规 {summary['speed_violation_count']}次　停车确认：{'是' if summary['stop_confirmed'] else '否'}",ha='center',fontsize=12)
axes[0].step(times,[r['completed_waypoints'] for r in rows],where='post',color='#218558',lw=2)
axes[0].set(ylabel='已完成航点数量',ylim=(0,46),yticks=(0,10,20,30,40,45))
for c in summary['checkpoints']:
 axes[0].axvline(c['time_s'],ls=':',color='#a0a7ac',lw=.8)
axes[1].plot(times,[r['actual_speed_knots'] for r in rows],color='#087eae',label='实际航速',lw=1.5)
axes[1].step(times,[r['command']['speed'] for r in rows],where='post',color='#8855a6',label='指令航速',alpha=.65,lw=.8)
axes[1].step(times,[wp[r['target_index']]['routeSpeed'] for r in rows],where='post',color='#ce4e42',ls=':',label='航点圈内限速',lw=1.2)
axes[1].set(ylabel='航速（节）',ylim=(0,55));axes[1].legend(loc='upper right',ncol=3,fontsize=9)
axes[2].plot(times,net,color='#087eae',lw=1.5)
axes[2].axhline(0,color='#ce4e42',ls='--',lw=1)
axes[2].set(ylabel='采样位置最小净空（米）',xlabel='仿真时间（秒）')
axes[2].text(.98,.94,f"整段运动检查最小净空：{summary['minimum_static_clearance_m']:.2f}米",ha='right',va='top',transform=axes[2].transAxes,color='#218558',fontsize=12,bbox={'facecolor':'white','edgecolor':'none','alpha':.9})
for axis in axes:axis.grid(alpha=.17)
fig.savefig(out/'metrics-cn.png',dpi=170);plt.close(fig)
metrics=[('总体通过','是' if summary['passed'] else '否'),('完成航点',f"{summary['reached_waypoints']}/45"),('静态目标数量',summary['static_obstacle_count']),('碰撞次数',summary['static_collision_count']),('全程最小净空',f"{summary['minimum_static_clearance_m']:.3f}米"),('航点限速违规',summary['speed_violation_count']),('实际最大转向速率',f"{summary['max_actual_turn_rate_deg_s']:.2f}度/秒"),('停车确认',summary['stop_confirmed']),('最终航速',f"{summary['final_speed_knots']:.2f}节"),('仿真时长',f"{summary['simulated_seconds']:.0f}秒"),('本机运行时间',f"{summary['elapsed_s']:.1f}秒"),('结束原因',summary['reason']),('错误',summary['error'])]
lines=['# 任务二追加障碍物闭环测试（2026-10-28）','',
 '上次停止于23/45航点的临时记录已清理，本次无法恢复当时的精确状态，故使用当前代码从头完整重跑。',
 '保留原45航点与16个静态目标，精确重建上次8个新增目标的位置，共24个。未修改航点或避障参数以迎合测试结果。','',
 '| 指标 | 结果 |','| --- | --- |']+[f'| {k} | {v} |' for k,v in metrics]
lines+=['','## 每十航点阶段图','', '| 已完成航点 | 仿真时刻 | 累计最小净空 | 碰撞次数 | 航点限速违规 | 图像 |','| --- | --- | --- | --- | --- | --- |']
for c in summary['checkpoints']:
 n=c['milestone'];lines.append(f"| {n} | {c['time_s']:.0f}秒 | {c['minimum_static_clearance_m']:.3f}米 | {c['static_collision_count']} | {c['speed_violation_count']} | [查看](checkpoints/waypoints_{n:02d}.png) |")
lines+=['','![最终轨迹](checkpoints/final.png)','', '![统计曲线](metrics-cn.png)','',
 '默认起点在首航点西侧50米，处于100米到达圈内，第1个航点初始化计为到达。',
 '净空为中心间距扣除约86.04米安全中心距离后的余量。最小值检查每步运动路径的全部线段；曲线展示逐秒采样位置净空。',
 '本机真实 Kev 决策，仿真使用配置的速度响应和转向速率；推理期间暂停仿真时间，未加入风流、侧滑和完整船体水动力。没有向现场发送任务或控制报文。','',
 '## 复现命令','', '从项目根目录运行：','', '```bash',
 '.venv-kev/bin/python server/local_models/serve_kev.py --device cuda',
 '.venv/bin/python server/experiments/field_tasks.py --task two --steps 12000 --checkpoint-every 10 --static-situation server/experiments/results/task-two-augmented-20261028/static-situation-augmented.json --output /tmp/kev-task-two-repeat',
 '```']
(out/'REPORT.md').write_text('\n'.join(lines)+'\n')
with (out/'trajectory-compact.csv').open('w',newline='') as f:
 writer=csv.writer(f);writer.writerow(['time_s','east_m','north_m','actual_speed_knots','completed_waypoints','command_speed_knots','command_course_deg'])
 for r in rows:writer.writerow([r['time_s'],r['x'],r['y'],r['actual_speed_knots'],r['completed_waypoints'],r['command']['speed'],r['command']['course']])
print(out/'metrics-cn.png');print(out/'REPORT.md')
