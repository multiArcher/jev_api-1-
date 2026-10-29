"""Offline real-Kev target tasks. No UDP; synthetic target route, simulated clock.

PYTHONPATH=jev_api:. .venv/bin/python -m server.experiments.target_tasks_animation
"""
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from server.experiments.dynamic_avoidance import advance
from server.experiments.target_following import record
from server.navigation import target_tasks
from server.navigation.dynamic_avoidance import Action, Vessel
from server.policies.kev_policy import KevPolicy
from tools.waypoint_geometry import KNOT_MPS

ROOT = Path(__file__).with_name('results') / 'target-tasks-20261029'
SCENARIOS = (
    ('capped_50', '伴随：限速20.58节，目标50节', 'accompanyMode', 20.58, 50.),
    ('unlimited_50', '伴随：上限50节，目标50节', 'accompanyMode', 0., 50.),
    ('accompany_30', '伴随：上限50节，目标30节', 'accompanyMode', 0., 30.),
    ('follow_30', '跟随：上限50节，目标30节', 'followMode', 0., 30.),
)


def dump(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def run(spec):
    name, title, code, cap, target_cap = spec
    folder = ROOT/name
    folder.mkdir(parents=True, exist_ok=True)
    policy = KevPolicy(duration=2., timeout=15.)
    params = dict(platCode=1010, targetPlatCode=1001, maxSpeed=cap)
    if code == 'accompanyMode':
        params.update(offsetAlongMinM=-160., offsetAlongMaxM=-140.,
                      offsetAcrossMinM=70., offsetAcrossMaxM=90., syncHeading=1)
    else:
        params.update(followDistanceMinM=150., followDistanceMaxM=200.)
    packet = dict(simId='offline', id=name, code=code, params=params)
    own = Vessel((-300., -80.), 10.*KNOT_MPS, 90.)
    target = Vessel((0., 0.), 10.*KNOT_MPS, 90.)
    t = 0.
    clock = SimpleNamespace(monotonic=lambda: t, perf_counter=time.perf_counter)
    pending = Action(own.speed_mps, own.course_deg)
    status, reason = 'running', ''
    rows = []
    dense_min = math.dist(own.position_m, target.position_m)
    first_band = None
    def snapshot(command=None):
        reports = [dict(record(pid, v.position_m, v.speed_mps/KNOT_MPS, v.course_deg),
                        _received_age_s=0., _turn_rate_deg_s=v.turn_rate_deg_s)
                   for pid, v in ((1010, own), (1001, target))]
        report = target_tasks.progress(packet, *reports)
        return reports, dict(time_s=t, own_xy_m=own.position_m, target_xy_m=target.position_m,
            own_speed_knots=own.speed_mps/KNOT_MPS, target_speed_knots=target.speed_mps/KNOT_MPS,
            own_course_deg=own.course_deg, target_course_deg=target.course_deg,
            distance_m=math.dist(own.position_m, target.position_m), report=report,
            command=command, status=status)
    rows.append(snapshot()[1])
    dump(folder/'configuration.json', dict(title=title, task=packet, dt_s=2., duration_s=160.,
        initial_own_xy_m=own.position_m, initial_target_xy_m=target.position_m,
        initial_speed_knots=10., initial_course_deg=90., target_cap_knots=target_cap,
        target_route='Eastbound; target30 scenarios turn toward120deg at90s. Synthetic route, not field replay.',
        model='Real local Kev HTTP', execution='2s command delay; dense independent0.02s integration; no UDP',
        safety_radius_m=policy.dynamic_config.safe_distance_m, motion_limits=policy.execution))
    with patch.object(target_tasks, 'time', clock):
        while t < 160.:
            reports, _ = snapshot()
            if status == 'running':
                try:
                    policy.validate_position(packet, reports[0], reports)
                    command = policy(dict(packet, situation=reports))
                except Exception as exc:
                    status, reason = 'failed', f'{type(exc).__name__}: {exc}'
                    command = (0., own.course_deg)
            else:
                command = (0., own.course_deg)
            own, trace = advance(own, pending, 2., policy.dynamic_config)
            target, target_trace = advance(target, Action(target_cap*KNOT_MPS,
                120. if target_cap == 30. and t >= 90. else 90.), 2., policy.dynamic_config)
            dense_min = min(dense_min, *(math.hypot(a[1]-b[1], a[2]-b[2])
                                        for a, b in zip(trace, target_trace)))
            pending = Action(command[0]*KNOT_MPS, command[1])
            t += 2.
            row = snapshot(command)[1]
            if status == 'running' and row['report'].get('inPosition', row['report'].get('inRange')):
                if first_band is None:
                    first_band = t
            rows.append(row)
            if t % 20. == 0:
                print(json.dumps(dict(scenario=name, time_s=t, status=status,
                    speed_knots=round(row['own_speed_knots'],2), distance_m=round(row['distance_m'],2))), flush=True)
            if status == 'failed' and own.speed_mps < .01:
                break
    summary = dict(scenario=name, title=title, status=status, failure=reason,
        simulated_s=t, first_band_entry_s=first_band, min_distance_m=dense_min,
        final_distance_m=rows[-1]['distance_m'], final_report=rows[-1]['report'],
        max_own_speed_knots=max(r['own_speed_knots'] for r in rows),
        zero_speed_commands=sum(r['command'] is not None and r['command'][0] == 0. for r in rows),
        in_band_samples=sum(bool(r['report'].get('inPosition',r['report'].get('inRange'))) for r in rows))
    dump(folder/'summary.json', summary)
    dump(folder/'trajectory.json', rows)
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    return summary


def render():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Polygon, Circle
    from server.experiments.animate_following_field import hull
    plt.rcParams['font.family'] = 'Noto Sans CJK JP'
    plt.rcParams['axes.unicode_minus'] = False
    loaded = [(spec, json.loads((ROOT/spec[0]/'trajectory.json').read_text())) for spec in SCENARIOS]
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), layout='constrained')
    artists = []
    for ax, (spec, rows) in zip(axes.flat, loaded):
        points = [r[k] for r in rows for k in ('own_xy_m','target_xy_m')]
        ax.set_xlim(min(p[0] for p in points)-150,max(p[0] for p in points)+150)
        ax.set_ylim(min(p[1] for p in points)-200,max(p[1] for p in points)+200)
        ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2)
        ax.set_xlabel('东向距离（m）');ax.set_ylabel('北向距离（m）')
        a, = ax.plot([],[],color='#2670ba',label='本艇')
        b, = ax.plot([],[],color='#df871d',label='目标')
        boat=Polygon(hull(rows[0]['own_xy_m'],90,25),color='#2670ba')
        tracked=Polygon(hull(rows[0]['target_xy_m'],90,25),color='#df871d')
        zone=Polygon([(0,0)]*4,color='#389b57',alpha=.25,label='要求位置/距离')
        ring=Circle((0,0),86.044,fill=False,ls=':',color='#ba4848',label='避碰范围')
        inner=Circle((0,0),150.,fill=False,color='#389b57',ls='--')
        outer=Circle((0,0),200.,fill=False,color='#389b57')
        for item in (inner,outer):item.set_visible(spec[2]=='followMode')
        for item in (boat,tracked,zone,ring,inner,outer):ax.add_patch(item)
        text=ax.text(.02,.98,'',transform=ax.transAxes,va='top',fontsize=9,
                     bbox=dict(facecolor='white',alpha=.85,edgecolor='none'))
        ax.legend(loc='lower left',fontsize=8)
        artists.append((a,b,boat,tracked,zone,ring,inner,outer,text))
    def update(i):
        for ax,(spec,rows),items in zip(axes.flat,loaded,artists):
            j=min(i,len(rows)-1);r=rows[j];a,b,boat,tracked,zone,ring,inner,outer,text=items
            for line,key in ((a,'own_xy_m'),(b,'target_xy_m')):
                line.set_data([q[key][0] for q in rows[:j+1]],[q[key][1] for q in rows[:j+1]])
            boat.set_xy(hull(r['own_xy_m'],r['own_course_deg'],25))
            tracked.set_xy(hull(r['target_xy_m'],r['target_course_deg'],25))
            ring.center=r['target_xy_m'];h=math.radians(r['target_course_deg']);x,y=r['target_xy_m']
            if spec[2]=='accompanyMode':
                zone.set_xy([(x+aa*math.sin(h)+cc*math.cos(h),y+aa*math.cos(h)-cc*math.sin(h))
                             for aa,cc in ((-160,70),(-140,70),(-140,90),(-160,90))])
            else:
                zone.set_visible(False);inner.center=(x,y);outer.center=(x,y)
            ax.set_title(spec[1],fontsize=11)
            flag=r['report'].get('inPosition',r['report'].get('inRange'))
            text.set_text(f"t={r['time_s']:.0f}s | {'失败后停车' if r['status']=='failed' else ('进入要求范围' if flag else '追踪中')}\n"
                          f"本艇 {r['own_speed_knots']:.1f}节 / 目标 {r['target_speed_knots']:.1f}节\n距离 {r['distance_m']:.1f}m")
        fig.suptitle('当前代码 · 真实本地Kev模型 · 合成航线追踪仿真（无UDP发送）\n统一时间轴：每帧2秒，8倍速播放；50节目标为直线，30节目标90秒后转向',fontsize=12)
    movie=FuncAnimation(fig,update,frames=max(len(rows) for _,rows in loaded),interval=250,repeat=False)
    movie.save(ROOT/'tracking-comparison.gif',writer=PillowWriter(fps=4),dpi=85)
    update(max(len(rows) for _,rows in loaded)-1)
    fig.savefig(ROOT/'tracking-routes.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(13,8),layout='constrained')
    for ax,(spec,rows) in zip(axes.flat,loaded):
        times=[r['time_s'] for r in rows]
        ax.plot(times,[r['own_speed_knots'] for r in rows],label='本艇实际航速')
        ax.plot(times,[r['target_speed_knots'] for r in rows],label='目标实际航速')
        ax.plot(times,[r['command'][0] if r['command'] else float('nan') for r in rows],ls=':',label='本艇指令航速')
        ax.set_title(spec[1]);ax.set_xlabel('仿真时间（s）');ax.set_ylabel('航速（节）');ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.savefig(ROOT/'tracking-speeds.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(13,8),layout='constrained')
    for ax,(spec,rows) in zip(axes.flat,loaded):
        times=[r['time_s'] for r in rows]
        ax.plot(times,[r['distance_m'] for r in rows],label='两艇距离')
        if spec[2]=='followMode':
            ax.axhspan(150,200,color='#389b57',alpha=.15,label='要求150–200m')
        else:
            ax.plot(times,[math.hypot(r['report']['alongOutM'],r['report']['acrossOutM']) for r in rows],
                    label='超出伴随区域的距离')
        ax.axhline(86.044,color='#ba4848',ls=':',label='避碰范围')
        ax.set_title(spec[1]);ax.set_xlabel('仿真时间（s）');ax.set_ylabel('距离（m）');ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.savefig(ROOT/'tracking-distances.png',dpi=150);plt.close(fig)


if __name__ == '__main__':
    ROOT.mkdir(parents=True,exist_ok=True)
    dump(ROOT/'summary.json',[run(spec) for spec in SCENARIOS])
    render()
