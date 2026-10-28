"""Show collision circles and simulated detours from the real field routes."""
import json
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[3]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'jev_api'))
from server.policies.static_obstacles import StaticObstacleConfig
from tools.waypoint_geometry import LocalFrame


def read_run(name):
    folder=OUT/name
    task=json.loads((folder/'task.json').read_text())
    frame=LocalFrame(task['params']['planWaypointList'])
    rows=[json.loads(line) for line in (folder/'trajectory.jsonl').open()]
    obstacles=json.loads((folder/'static-obstacles.json').read_text())
    centers={r['platformId']:frame.project(r['longitude'],r['latitude']) for r in obstacles}
    points=[frame.project(p['routeLon'],p['routeLat']) for p in task['params']['planWaypointList']]
    return rows,centers,points


fig,axes=plt.subplots(1,3,figsize=(16,5.5))
radius=StaticObstacleConfig().collision_radius_m
for ax,name,ids,title in zip(axes,
        ('task_one','task_one','occupied_waypoint_probe'),
        ((4001,),(4002,4003),(4999,4001)),
        ('Task one: obstacle 4001','Task one: obstacles 4002 / 4003','Obstacle at waypoint: arrival radius stays 100 m')):
    rows,centers,points=read_run(name)
    cx=sum(centers[i][0] for i in ids)/len(ids)
    cy=sum(centers[i][1] for i in ids)/len(ids)
    if name=='occupied_waypoint_probe':cx,cy=centers[4999]
    ax.plot([p[0]-cx for p in points],[p[1]-cy for p in points],'--',color='gray',linewidth=1,label='Waypoint order')
    ax.plot([r['x']-cx for r in rows],[r['y']-cy for r in rows],color='#0078b8',linewidth=2,label='Real Kev simulation')
    for pid in ids:
        center=(centers[pid][0]-cx,centers[pid][1]-cy)
        ax.add_patch(Circle(center,radius,color='#dc5252',alpha=.22,label='80 m + hull envelope' if pid==ids[0] else None))
        ax.scatter(*center,s=20,color='#b82e2e')
        ax.annotate(str(pid),center,xytext=(5,5),textcoords='offset points',fontsize=8)
    if name=='occupied_waypoint_probe':
        ax.add_patch(Circle((0,0),100,fill=False,edgecolor='#16884e',linestyle='--',label='Original arrival circle'))
        ax.scatter(rows[-1]['x']-cx,rows[-1]['y']-cy,marker='x',color='#16884e',s=45,label='Stopped')
    ax.set(xlim=(-350,350),ylim=(-350,350),aspect='equal',xlabel='East offset (m)',ylabel='North offset (m)',title=title)
    ax.grid(alpha=.2);ax.legend(fontsize=7,loc='best')
fig.tight_layout();fig.savefig(OUT/'static-avoidance-detail.png',dpi=180);plt.close(fig)
print(OUT/'static-avoidance-detail.png')
