import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

root=Path('server/experiments/results/task-one-following-100-150-20261028')
names={'straight':'匀速直线','s_curve':'S形','crossing_variable':'交叉折返与变速'}
labels={'straight':'Straight, 10 knots','s_curve':'S turns, 10 knots','crossing_variable':'Crossing turns, 6–18 knots'}
results={name:json.loads((root/name/'summary.json').read_text()) for name in names}
metadata={name:json.loads((root/name/'configuration.json').read_text()) for name in names}
traces={name:[json.loads(s) for s in (root/name/'trajectory.jsonl').read_text().splitlines()] for name in names}
fig,axes=plt.subplots(3,1,figsize=(13,13),layout='constrained')
for ax,(name,s) in zip(axes,results.items()):
 c=metadata[name];rows=traces[name]
 own=[c['own_start_xy_m']]+[r['own_xy_m'] for r in rows]
 target=[c['target_start_xy_m']]+[r['target_xy_m'] for r in rows]
 route=c['map_route_xy_m']
 ax.plot([p[0] for p in route],[p[1] for p in route],ls='--',color='.7',lw=1,label='Task 1 reference route')
 for obstacle in c['obstacles_xy_m']:
  ax.add_patch(Circle(obstacle['xy'],c['collision_radius_m'],facecolor='#dd8578',edgecolor='#b44532',alpha=.4))
 ax.plot([p[0] for p in target],[p[1] for p in target],color='#e07b13',lw=2.5,label='Target')
 ax.plot([p[0] for p in own],[p[1] for p in own],color='#2167b5',lw=1.8,label='Tracking vessel (real Kev)')
 for points,color in ((target,'#e07b13'),(own,'#2167b5')):
  ax.scatter(*points[0],c=color,s=45,marker='o',zorder=5)
  ax.scatter(*points[-1],c=color,s=55,marker='X',zorder=5)
  for i in range(0,len(rows),max(1,len(rows)//4)):
   ax.annotate(f"{rows[i]['time_s']:.0f}s",points[i+1],color=color,fontsize=8,xytext=(3,5),textcoords='offset points')
 allpoints=own+target
 ax.set_xlim(min(p[0] for p in allpoints)-90,max(p[0] for p in allpoints)+90)
 ax.set_ylim(min(p[1] for p in allpoints)-90,max(p[1] for p in allpoints)+90)
 ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2)
 ax.set_title(f"{labels[name]} | {s['simulated_seconds']:.0f}s | first band entry {s['first_band_entry_control_s']}s | "
              f"after entry: {s['post_entry_min_control_distance_m']:.1f}–{s['post_entry_max_control_distance_m']:.1f}m")
 ax.set_xlabel('East from task 1 origin (m)');ax.set_ylabel('North (m)');ax.legend(fontsize=8,loc='best')
fig.suptitle('Task 1 scene: progressive target-following tests\nAllowed range 100–150 m; initial range 300 m; straight-line start offset 75 m',fontsize=15)
fig.savefig(root/'routes-overview.png',dpi=150);plt.close(fig)
lines=['# 任务一地图目标追踪测试结果','','三种路线均调用当前运行的真实本地 Kev 服务，未用模拟选择器替代。允许中心距离 **100–150米**，每种初始距离 **300米**；直线场景本艇起点侧向偏离目标运动直线 **75米**。配置由任务样例读取，各场景保存了当次任务快照。','','## 结果','','| 场景 | 模拟时长 | 首次入圈 | 入圈后控制采样距离 | 入圈后区间内比例 | 最终距离 | 安全等待次数 |','| --- | ---: | ---: | ---: | ---: | ---: | ---: |']
for name,s in results.items():
 lines.append(f"| {names[name]} | {s['simulated_seconds']:.0f}s | {s['first_band_entry_control_s']:.0f}s | {s['post_entry_min_control_distance_m']:.2f}–{s['post_entry_max_control_distance_m']:.2f}m | {s['post_entry_control_in_band_fraction']:.1%} | {s['final_distance_m']:.2f}m | {s['safety_hold_steps']} |")
lines+=['','首次入圈和入圈后统计按2秒控制采样点计算；各场景 `summary.json` 另含0.02秒积分采样得到的全程最小距离与全程区间内比例。初始300米在允许范围以外，是有意设置的追赶条件；不能把初始超距直接当作跟踪失败。','','![三种路线汇总](routes-overview.png)','','## 分场景图像与记录','']
for name,s in results.items():
 lines += [f"- **{names[name]}**：[完整路线／距离／速度图]({name}/routes.png) · [统计]({name}/summary.json) · [逐轮轨迹]({name}/trajectory.jsonl)。"]
c=results['crossing_variable']
lines += ['',f"交叉场景目标沿8字路线完成约 **{c['target_completed_loops']:.2f}圈**，控制采样目标速度范围 **{c['target_speed_min_knots']:.2f}–{c['target_speed_max_knots']:.2f}节**。其目标最大转向角速度预检约8.93度/秒，速度变化率约0.194米/秒²，目标位置和速度连续变化，无瞬移。",'','## 观察与限制','','直线和S形均能从超距状态追回，但入圈后存在短暂超过150米、反复切换速度的现象。当前结果说明具备追赶能力，尚不能认定稳定维持距离区间。','']
for name,s in results.items():
 lines.append(f"- {names[name]}：全程最小中心距离{s['min_distance_m']:.2f}米；本艇到记录静态安全圈的最小净距{s['min_static_clearance_m']:.2f}米；目标对应净距{s['target_min_static_clearance_m']:.2f}米；最长决策{s['max_decision_s']:.2f}秒；速度指令变化{s['speed_command_changes']}次。")
lines += ['','所有16个记录静态目标均参与检查，但目标路线预留了较大静态净距，本组主要评估移动目标追踪。追踪目标是两艇中心距离区间，轨迹无需逐点重合。','','地图数据为任务一原始12个航点及记录障碍物，没有海岸线、水深或完整Unity地形。船舶按0.5秒速度响应、45度/秒转向上限，用独立0.02秒中点积分更新。模型推理期间暂停模拟时间，未覆盖真实推理延迟下的实时船舶运动，不代表实艇验证。未向现场发送控制报文。','','实验说明与复现命令见 [README.md](README.md)。旧范围或旧共线起点结果已保留为历史记录，不计入本报告。','']
(root/'REPORT.md').write_text('\n'.join(lines))
print(json.dumps({name:{key:value for key,value in s.items() if key in ('first_band_entry_control_s','post_entry_control_in_band_fraction','min_distance_m','safety_hold_steps')} for name,s in results.items()},ensure_ascii=False))
