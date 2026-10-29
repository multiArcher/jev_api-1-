"""Replay recorded following results as GIF; no inference or simulation rerun.

PYTHONPATH=jev_api:. .venv/bin/python -m server.experiments.animate_following_field
Each frame corresponds to one saved control sample (2 simulated seconds).
"""
import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle, Polygon

DEFAULT_ROOT=Path('server/experiments/results/task-one-following-100-150-20261028')
NAMES={'straight':'Straight line','s_curve':'S turns','crossing_variable':'Crossing turns, variable speed'}


def load_samples(folder):
    config=json.loads((folder/'configuration.json').read_text())
    rows=[json.loads(s) for s in (folder/'trajectory.jsonl').read_text().splitlines()]
    course=(90. if config['scenario']!='crossing_variable' else math.degrees(math.atan2(300.,360.)))
    initial=dict(time_s=0.,own_xy_m=config['own_start_xy_m'],target_xy_m=config['target_start_xy_m'],
        distance_m=math.dist(config['own_start_xy_m'],config['target_start_xy_m']),
        own_speed_knots=12. if config['scenario']=='crossing_variable' else 10.,own_course_deg=course,
        target_speed_knots=12. if config['scenario']=='crossing_variable' else 10.,target_course_deg=course)
    return config,[initial]+rows


def hull(position,course,length=16.):
    angle=math.radians(course);fx,fy=math.sin(angle),math.cos(angle)
    sx,sy=math.cos(angle),-math.sin(angle);x,y=position
    return [(x+length*fx,y+length*fy),(x-length*.65*fx+length*.45*sx,y-length*.65*fy+length*.45*sy),
            (x-length*.65*fx-length*.45*sx,y-length*.65*fy-length*.45*sy)]


def animate(folder,*,fps=5,tail_seconds=40.):
    c,rows=load_samples(folder);lower=c['min_following_distance_m'];upper=c['max_following_distance_m']
    own=[r['own_xy_m'] for r in rows];target=[r['target_xy_m'] for r in rows]
    points=own+target;pad=upper+35
    fig=plt.figure(figsize=(9,6.5),layout='constrained');gs=fig.add_gridspec(2,1,height_ratios=(3.2,1.))
    ax=fig.add_subplot(gs[0]);dist=fig.add_subplot(gs[1])
    route=c['map_route_xy_m'];ax.plot([p[0] for p in route],[p[1] for p in route],ls='--',c='.75',lw=.7)
    for obstacle in c['obstacles_xy_m']:
        ax.add_patch(Circle(obstacle['xy'],c['collision_radius_m'],color='#b94e3d',alpha=.25))
    ax.set_xlim(min(p[0] for p in points)-pad,max(p[0] for p in points)+pad)
    ax.set_ylim(min(p[1] for p in points)-pad,max(p[1] for p in points)+pad)
    ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.17)
    ax.set_xlabel('East from task 1 origin (m)');ax.set_ylabel('North (m)')
    own_past,=ax.plot([],[],color='#2167b5',lw=1,alpha=.22)
    target_past,=ax.plot([],[],color='#e07b13',lw=1,alpha=.22)
    own_tail,=ax.plot([],[],color='#2167b5',lw=2,label='Tracking vessel')
    target_tail,=ax.plot([],[],color='#e07b13',lw=2,label='Moving target')
    small=Circle(target[0],lower,fill=False,color='#398259',lw=1.2,ls='--')
    large=Circle(target[0],upper,fill=False,color='#398259',lw=1.2)
    ax.add_patch(small);ax.add_patch(large)
    boat=Polygon(hull(own[0],rows[0]['own_course_deg']),color='#2167b5',zorder=6)
    tracked=Polygon(hull(target[0],rows[0]['target_course_deg']),color='#e07b13',zorder=6)
    ax.add_patch(boat);ax.add_patch(tracked)
    connector,=ax.plot([],[],lw=1.2,zorder=4)
    ax.legend(loc='upper right',fontsize=8)
    title=fig.suptitle('')
    dist.axhspan(lower,upper,color='#daeee0',label=f'Allowed {lower:g}–{upper:g}m')
    history,=dist.plot([],[],color='#2167b5',lw=1.5)
    cursor=dist.axvline(0,color='.5',lw=1)
    now,=dist.plot([],[],marker='o',markersize=5)
    dist.set_xlim(0,rows[-1]['time_s']);dist.set_ylim(min(lower-15,min(r['distance_m'] for r in rows)-10),max(r['distance_m'] for r in rows)+15)
    dist.set_xlabel('Simulated time (s)');dist.set_ylabel('Range (m)');dist.grid(alpha=.17)
    dist.legend(fontsize=8,loc='upper right')
    def update(index):
        r=rows[index];t=r['time_s'];start=next((j for j in range(index+1) if rows[j]['time_s']>=t-tail_seconds),0)
        for artist,points_,begin in ((own_past,own,0),(target_past,target,0),(own_tail,own,start),(target_tail,target,start)):
            artist.set_data([p[0] for p in points_[begin:index+1]],[p[1] for p in points_[begin:index+1]])
        small.center=target[index];large.center=target[index]
        boat.set_xy(hull(own[index],r['own_course_deg']));tracked.set_xy(hull(target[index],r['target_course_deg']))
        color='#398259' if lower<=r['distance_m']<=upper else '#ba4035'
        connector.set_data([own[index][0],target[index][0]],[own[index][1],target[index][1]]);connector.set_color(color)
        history.set_data([x['time_s'] for x in rows[:index+1]],[x['distance_m'] for x in rows[:index+1]])
        cursor.set_xdata([t,t]);now.set_data([t],[r['distance_m']]);now.set_color(color)
        title.set_text(f"{NAMES[c['scenario']]} | t={t:.0f}/{rows[-1]['time_s']:.0f}s | range={r['distance_m']:.1f}m\n"
            f"Vessel {r['own_speed_knots']:.1f} kn / target {r['target_speed_knots']:.1f} kn | "
            f"allowed {lower:g}–{upper:g}m | {c['dt_s']*fps:g}x replay")
        return ()
    animation=FuncAnimation(fig,update,frames=len(rows),interval=1000/fps,blit=False,repeat=False)
    output=folder/'replay.gif';animation.save(output,writer=PillowWriter(fps=fps),dpi=105)
    update(len(rows)//2);fig.savefig(folder/'replay-preview.png',dpi=105);plt.close(fig)
    print(json.dumps(dict(scenario=c['scenario'],gif=str(output.resolve()),frames=len(rows),bytes=output.stat().st_size)),flush=True)
    return output


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,default=DEFAULT_ROOT)
    p.add_argument('--scenario',choices=['all',*NAMES],default='all');p.add_argument('--fps',type=int,default=5)
    args=p.parse_args()
    if args.fps<=0:p.error('fps must be positive')
    for name in NAMES if args.scenario=='all' else (args.scenario,):animate(args.root/name,fps=args.fps)


if __name__=='__main__':main()
