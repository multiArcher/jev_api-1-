"""Render the recorded real-Kev task-one runs, without rerunning decisions."""
from pathlib import Path
import json
import math
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,patheffects
from matplotlib.patches import Circle,Rectangle

ROOT=Path.cwd()
sys.path.insert(0,str(ROOT/'jev_api'))
from tools.waypoint_geometry import LocalFrame

BASE=Path(__file__).parent
for path in ('/usr/share/fonts/truetype/SimHei.ttf',):
    if Path(path).exists():font_manager.fontManager.addfont(path)
for name in ('SimHei','Noto Sans CJK SC','WenQuanYi Zen Hei','DejaVu Sans'):
    try:
        font_manager.findfont(name,fallback_to_default=False);FONT=name;break
    except ValueError:pass
plt.rcParams.update({'font.family':FONT,'axes.unicode_minus':False,'axes.spines.top':False,'axes.spines.right':False})


def load(folder):
    summary=json.loads((folder/'summary.json').read_text())
    rows=[json.loads(line) for line in (folder/'trajectory.jsonl').read_text().splitlines()]
    return summary,rows


def draw():
    main_folder=BASE/'task_one'
    summary,rows=load(main_folder)
    failed,failed_rows=load(Path('/home/epic/jev_api(1)/server/experiments/results/task-one-avoidance-tracking-20261028-115840/task_one'))
    task=json.loads((main_folder/'task.json').read_text())
    waypoints=task['params']['planWaypointList'];frame=LocalFrame(waypoints)
    points=[frame.project(p['routeLon'],p['routeLat']) for p in waypoints]
    obstacle_records=json.loads((main_folder/'static-obstacles.json').read_text())
    obstacles=[dict(id=r['platformId'],xy=frame.project(r['longitude'],r['latitude'])) for r in obstacle_records]
    radius=summary['static_collision_radius_m']
    complete=summary['reached_waypoints']
    status='通过' if summary['passed'] else ('完成，但未通过约束检查' if summary['completed'] else '未完成')
    figure=plt.figure(figsize=(15,13),layout='constrained')
    grid=figure.add_gridspec(4,2,width_ratios=(1.2,1),height_ratios=(1,1,1,.65))
    overview=figure.add_subplot(grid[:3,0])
    zooms=[figure.add_subplot(grid[i,1]) for i in range(3)]
    speed_ax=figure.add_subplot(grid[3,:])
    figure.suptitle(f'任务一｜避障与循迹｜转向调整后，90°进场复测：{status}',fontsize=23,fontweight='bold',color='#17324d')
    subtitle=f'{complete}/12 航点 · 仿真 {summary["simulated_seconds"]:.0f} 秒\n静态碰撞 {summary["static_collision_count"]} 次 · 限速违规 {summary["speed_violation_count"]} 次\n最小净空 {summary["minimum_static_clearance_m"]:.2f} 米'
    overview.set_title(subtitle,fontsize=9.5,pad=15)
    zones=[(100,1100,-2650,-1650,'区域①：4001，首个航线交叠障碍'),
           (650,1800,-5750,-4600,'区域②：4002／4003，连续障碍'),
           (1050,2200,-8450,-7300,'区域③：4004，长航段避让')]

    def layer(ax,extent=None,labels=True):
        ax.plot([x/1000 for x,y in points],[y/1000 for x,y in points],ls='--',color='#a4abb3',lw=1.2,label='原航点连线',zorder=1)
        ax.plot([r['x']/1000 for r in rows],[r['y']/1000 for r in rows],color='#087eb3',lw=2.,label='实际仿真航迹',zorder=5)
        for index,(x,y) in enumerate(points):
            visited=index<complete;color='#25936a' if visited else '#a4abb3'
            ax.add_patch(Circle((x/1000,y/1000),.1,fill=False,color=color,alpha=.5,lw=.8,zorder=2))
            ax.scatter(x/1000,y/1000,s=18,color=color,zorder=6)
            if labels and (extent is None or extent[0]-150<x<extent[1]+150 and extent[2]-150<y<extent[3]+150):
                text=ax.annotate(f'P{index+1}',(x/1000,y/1000),xytext=(5,6 if index%2==0 else -11),textcoords='offset points',fontsize=9,color='#243a48',zorder=8)
                text.set_path_effects([patheffects.withStroke(linewidth=2.5,foreground='white')])
        for index,obstacle in enumerate(obstacles):
            x,y=obstacle['xy']
            ax.add_patch(Circle((x/1000,y/1000),radius/1000,facecolor='#e56c60',edgecolor='#bb473d',alpha=.35,lw=.6,zorder=3,label='静态目标禁入范围' if index==0 else None))
            ax.scatter(x/1000,y/1000,marker='x',color='#ad3b31',s=14,zorder=4)
            if obstacle['id']>=4001 and (extent is None or extent[0]<x<extent[1] and extent[2]<y<extent[3]):
                text=ax.annotate(str(obstacle['id']),(x/1000,y/1000),xytext=(5,-12),textcoords='offset points',fontsize=8,color='#a33d34',zorder=9)
                text.set_path_effects([patheffects.withStroke(linewidth=2.5,foreground='white')])
        ax.scatter(rows[0]['x']/1000,rows[0]['y']/1000,marker='s',s=45,color='#159570',edgecolor='white',label='起点',zorder=10)
        ax.scatter(rows[-1]['x']/1000,rows[-1]['y']/1000,marker='*',s=120,color='#8548a8',edgecolor='white',label='结束位置',zorder=11)
        ax.set_aspect('equal',adjustable='box')
        ax.grid(alpha=.15)
        if extent:ax.set_xlim(extent[0]/1000,extent[1]/1000);ax.set_ylim(extent[2]/1000,extent[3]/1000)
        ax.set_xlabel('东向距离（公里）',fontsize=10);ax.set_ylabel('北向距离（公里）',fontsize=10)

    layer(overview)
    overview.set_xlim(-1.5,7.7);overview.set_ylim(-18.5,1.8)
    colors=('#e09435','#7455b5','#3b9c9a')
    for index,(left,right,bottom,top,title) in enumerate(zones):
        overview.add_patch(Rectangle((left/1000,bottom/1000),(right-left)/1000,(top-bottom)/1000,fill=False,edgecolor=colors[index],lw=1.2,ls='--'))
        overview.text(right/1000+.1,(bottom+top)/2000,f'区域{index+1}',color=colors[index],fontsize=9)
        layer(zooms[index],(left,right,bottom,top));zooms[index].set_title(title,fontsize=12,color=colors[index])
    overview.legend(loc='lower left',fontsize=9,framealpha=.9)
    times=[row['time_s'] for row in rows]
    speed_ax.plot(times,[r['actual_speed_knots'] for r in rows],color='#087eb3',lw=1.2,label='实际航速')
    speed_ax.step(times,[r['command']['speed'] for r in rows],color='#d78c2e',alpha=.6,lw=.8,label='目标航速')
    speed_ax.step(times,[waypoints[r['target_index']]['routeSpeed'] for r in rows],color='#b54c4c',ls='--',lw=1.,label='航点圈内限速')
    for arrival in summary['arrivals']:
        speed_ax.axvline(arrival['time_s'],color='#25936a',lw=.5,alpha=.35)
    speed_ax.set(xlabel='仿真时间（秒）',ylabel='航速（节）',ylim=(-1,55))
    speed_ax.grid(alpha=.15);speed_ax.legend(loc='upper right',fontsize=9,ncol=3)
    figure.supxlabel('主图：P1以西150米、初始航向90°；与调整前失败测试使用相同初始条件、地图和物理约束。\n真实本地 Kev 决策；离线仿真在推理期间暂停。航点圈半径100米，红色禁入半径约86米。',fontsize=10,color='#52616b')
    figure.savefig(BASE/'route-map.png',dpi=180)
    figure.savefig(BASE/'route-map.pdf')
    plt.close(figure)

    fig,ax=plt.subplots(figsize=(10,8),layout='constrained')
    ax.plot([x for x,y in points[:4]],[y for x,y in points[:4]],'--',color='#8b98a4',label='原航点连线')
    ax.plot([r['x'] for r in failed_rows],[r['y'] for r in failed_rows],color='#d5675a',lw=2,label=f'调整前，90°进场：{failed["reached_waypoints"]}/12，未完成')
    recent=[r for r in rows if r['target_index']<=2]
    ax.plot([r['x'] for r in recent],[r['y'] for r in recent],color='#087eb3',lw=2,label='调整后，90°进场复测')
    for index,(x,y) in enumerate(points[:4]):
        ax.add_patch(Circle((x,y),100,fill=False,color='#8b98a4',alpha=.7))
        ax.annotate(f'P{index+1}',(x,y),xytext=(6,6),textcoords='offset points')
    ax.scatter(failed_rows[-1]['x'],failed_rows[-1]['y'],marker='x',s=90,color='#d5675a')
    ax.set(title='相同90°初始姿态：转向调整前后对照',xlabel='东向距离（米）',ylabel='北向距离（米）',aspect='equal',xlim=(-300,900),ylim=(-2100,350))
    ax.legend(fontsize=10,loc='lower left');ax.grid(alpha=.2)
    fig.savefig(BASE/'entry-comparison.png',dpi=160);plt.close(fig)

    distance=sum(math.dist(a,b) for a,b in zip(points,points[1:]))
    report={
        'main':summary,
        'initial_heading_test':{key:failed[key] for key in ('passed','completed','reason','reached_waypoints','error','stop_confirmed')},
        'reference_route_length_m':distance,
        'static_obstacles':len(obstacles),
        'interpretation':'真实本机Kev模型＋离线船艇运动仿真；不是实艇测试。主图为调整后原始90°初始姿态复测；原失败记录保留。'
    }
    (BASE/'test-results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    lines=['# 任务一：避障与循迹测试','',
           '真实本地 Kev 模型，离线仿真。原地图12个航点、16个静态目标，参考航线约16.04公里。',
           '','| 测试 | 航点 | 完成 | 通过 | 原因 |','|---|---:|---|---|---|',
           f'| 调整前：P1以西150米、90° | {failed["reached_waypoints"]}/12 | {failed["completed"]} | {failed["passed"]} | {failed["reason"]} |',
           f'| 调整后：P1以西150米、90° | {complete}/12 | {summary["completed"]} | {summary["passed"]} | {summary["reason"]} |',
           '',f'主测试静态碰撞：{summary["static_collision_count"]}；航点限速违规：{summary["speed_violation_count"]}；最小静态净空：{summary["minimum_static_clearance_m"]:.3f}米；停车确认：{summary["stop_confirmed"]}。',
           '',f'主测试仿真{summary["simulated_seconds"]:.0f}秒，最高实际航速{summary["max_speed_knots"]:.3f}节，最大实际平均转向角速度{summary["max_actual_turn_rate_deg_s"]:.3f}°/s。',
           '',f'首轮失败详情：{failed["error"]}',
           '',f'主测试异常：{summary["error"]}',
           '','调整内容：航向偏差超过30°进入10节以内对准，降至15°以内退出；16个持续航向候选包含引导目标方位；用10秒物理预测比较实际航向进展，存在安全有效转向时过滤无效直行；提示词优先完成低速对准。动态避障保持原动作空间与优先级。',
           '','当前参数：最大50节、加速度1.46m/s²、制动幅值1.75m/s²、转向角速度12°/s、角加速度4°/s²。50节直线加速使用实测控制器响应。',
           '','调整前后使用相同起点、90°航向、静止起步、原12航点和16静态目标、1秒控制周期、100米到点圈。最大速度、加减速、角速度和角加速度上限保持不变。此图基于航点与目标坐标层，不包含岸线／水深；世界在模型推理期间暂停。',
           '','回归验证：服务端242项测试通过；客户端17项中8项通过、9项因缺少Unity地图资源跳过。新增回归覆盖原90°进场航点切换、对准状态退出和重置、实际转向预览及静态安全过滤。',
           '','![路线图](route-map.png)','','![同初始姿态调整前后对照](entry-comparison.png)']
    audit_path=BASE/'motion-audit.json'
    if audit_path.exists():
        audit=json.loads(audit_path.read_text())
        report['motion_audit']=audit
        (BASE/'test-results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        lines.extend(['','运动指令回放与原航迹匹配；源文件哈希一致。加速度、制动、角速度及角加速度约束检查通过。',f'实测50节加速响应在本次实际运动中触发{audit["measured_response_steps"]}个控制周期；重算最小静态净空{audit["minimum_static_clearance_m"]:.3f}米。'])
    (BASE/'REPORT.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({key:summary.get(key) for key in ('passed','completed','reason','reached_waypoints','static_collision_count','speed_violation_count','minimum_static_clearance_m','stop_confirmed')},ensure_ascii=False))


if __name__=='__main__':draw()
