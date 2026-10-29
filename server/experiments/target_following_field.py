"""Progressive target-following trials on task one's recorded geographic scene.

The map is the supplied waypoint/recorded-obstacle layer, not a coastline raster.
Only local Kev HTTP is called; no UDP control packets are sent.
"""
import argparse
import json
import math
from pathlib import Path
import time

from server.policies.kev_policy import KevPolicy
from server.experiments.target_following import integrate, mock_choice
from tools.waypoint_geometry import LocalFrame, KNOT_MPS, EARTH_RADIUS

TASKS = Path(__file__).with_name('tasks')
DEFAULT_TASK = Path(__file__).resolve().parents[1]/'examples'/'target_following.json'
DURATIONS = {'straight': 120., 's_curve': 240., 'crossing_variable': 420.}
LABELS = {'straight': 'Constant-speed straight line', 's_curve': 'Constant-speed S turns',
          'crossing_variable': 'Crossing figure-eight, variable speed'}


def to_record(frame, pid, position, speed, course):
    lon, lat = frame.origin
    return dict(platformId=pid,
        longitude=lon+math.degrees(position[0]/(EARTH_RADIUS*math.cos(math.radians(lat)))),
        latitude=lat+math.degrees(position[1]/EARTH_RADIUS), targetSpeed=speed,
        targetCourse=course%360, _received_age_s=0.)


def target_speed(name, t):
    return 12.+6.*math.sin(2*math.pi*t/100.) if name == 'crossing_variable' else 10.


def target_course(name, t, theta=0.):
    if name == 's_curve':
        return 90.+40.*math.sin(2*math.pi*t/80.)
    if name == 'crossing_variable':
        return math.degrees(math.atan2(300.*math.cos(theta),360.*math.cos(2*theta)))%360
    return 90.


def advance_target(name, t, seconds, position, theta, center):
    count = max(1,math.ceil(seconds/.02))
    dt = seconds/count
    points = [position]
    x,y = position
    for i in range(count):
        mid = t+(i+.5)*dt
        speed = target_speed(name,mid)*KNOT_MPS
        if name == 'crossing_variable':
            def angular(value):
                return speed/math.hypot(300.*math.cos(value),360.*math.cos(2*value))
            a=angular(theta);b=angular(theta+a*dt/2);c=angular(theta+b*dt/2);d=angular(theta+c*dt)
            theta += (a+2*b+2*c+d)*dt/6
            x,y = center[0]+300.*math.sin(theta),center[1]+180.*math.sin(2*theta)
        else:
            heading = math.radians(target_course(name,mid))
            x+=speed*math.sin(heading)*dt
            y+=speed*math.cos(heading)*dt
        points.append((x,y))
    return (x,y),theta,points


def dump(path,value):
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')


def recovery_metrics(rows, lower, upper):
    """Control-sample recovery metrics; the intentional initial excess is separate."""
    first = next((i for i,r in enumerate(rows) if lower <= r['distance_m'] <= upper),None)
    result = dict(first_band_entry_control_s=None,post_entry_control_in_band_fraction=None,
                  post_entry_min_control_distance_m=None,post_entry_max_control_distance_m=None,
                  speed_command_changes=sum(abs(a['command'][0]-b['command'][0])>.01
                                            for a,b in zip(rows,rows[1:])))
    if first is not None:
        later=rows[first:]
        result.update(first_band_entry_control_s=rows[first]['time_s'],
            post_entry_control_in_band_fraction=sum(lower<=r['distance_m']<=upper for r in later)/len(later),
            post_entry_min_control_distance_m=min(r['distance_m'] for r in later),
            post_entry_max_control_distance_m=max(r['distance_m'] for r in later))
    return result


def plot_result(folder):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    metadata=json.loads((folder/'configuration.json').read_text())
    summary=json.loads((folder/'summary.json').read_text())
    rows=[json.loads(line) for line in (folder/'trajectory.jsonl').read_text().splitlines()]
    own=[metadata['own_start_xy_m']]+[r['own_xy_m'] for r in rows]
    target=[metadata['target_start_xy_m']]+[r['target_xy_m'] for r in rows]
    route=metadata['map_route_xy_m']
    obstacles=metadata['obstacles_xy_m']
    scale=1000.
    fig=plt.figure(figsize=(14,10),layout='constrained')
    gs=fig.add_gridspec(3,3,height_ratios=(1,1,.9))
    ax=fig.add_subplot(gs[:2,:2]);overview=fig.add_subplot(gs[:2,2])
    distance=fig.add_subplot(gs[2,:2]);speeds=fig.add_subplot(gs[2,2])
    xs=[p[0] for p in own+target];ys=[p[1] for p in own+target]
    pad=180.
    bounds=(min(xs)-pad,max(xs)+pad,min(ys)-pad,max(ys)+pad)
    for panel in (ax,overview):
        panel.plot([p[0]/scale for p in route],[p[1]/scale for p in route],color='.65',linestyle='--',marker='.',label='Task 1 reference route')
        for obstacle in obstacles:
            panel.add_patch(Circle((obstacle['xy'][0]/scale,obstacle['xy'][1]/scale),
                metadata['collision_radius_m']/scale,facecolor='#e87b69',edgecolor='#ae4431',alpha=.5))
        panel.plot([p[0]/scale for p in target],[p[1]/scale for p in target],color='#e07b13',lw=2,label='Moving target')
        panel.plot([p[0]/scale for p in own],[p[1]/scale for p in own],color='#2167b5',lw=1.8,label='Tracking vessel (Kev)')
        panel.set_aspect('equal',adjustable='box');panel.grid(alpha=.2)
        panel.set_xlabel('East from task 1 origin (km)');panel.set_ylabel('North (km)')
    ax.set_xlim(bounds[0]/scale,bounds[1]/scale);ax.set_ylim(bounds[2]/scale,bounds[3]/scale)
    for points,color in ((target,'#e07b13'),(own,'#2167b5')):
        ax.scatter(points[0][0]/scale,points[0][1]/scale,c=color,marker='o',s=60,zorder=5)
        ax.scatter(points[-1][0]/scale,points[-1][1]/scale,c=color,marker='X',s=70,zorder=5)
        for i in range(0,len(rows),max(1,len(rows)//5)):
            point=points[i+1]
            ax.annotate(f"{rows[i]['time_s']:.0f}s",(point[0]/scale,point[1]/scale),fontsize=8,color=color,xytext=(3,4),textcoords='offset points')
    if metadata['scenario']=='straight':
        target_y=target[0][1]/scale
        ax.axhline(target_y,color='#e07b13',ls=':',alpha=.45,lw=1)
        x,y=own[0][0]/scale,own[0][1]/scale
        ax.plot([x,x],[target_y,y],color='#6139a0',ls=':',lw=1.2)
        ax.annotate(f"{abs(metadata['initial_cross_track_m']):g} m lateral offset",(x,(target_y+y)/2),
                    xytext=(6,0),textcoords='offset points',fontsize=8,color='#6139a0')
    ax.legend(loc='best',fontsize=9);ax.set_title('Motion routes — circles: start, X: end')
    overview.add_patch(Rectangle((bounds[0]/scale,bounds[2]/scale),
        (bounds[1]-bounds[0])/scale,(bounds[3]-bounds[2])/scale,fill=False,edgecolor='#6139a0',lw=1.5))
    overview.set_title('Task 1 geographic layer\nRecorded obstacles + 12 waypoints')
    times=[0.]+[r['time_s'] for r in rows]
    ranges=[math.dist(own[0],target[0])]+[r['distance_m'] for r in rows]
    lower,upper=metadata['min_following_distance_m'],metadata['max_following_distance_m']
    distance.axhspan(lower,upper,color='#daeee0',label=f'Allowed {lower:g}–{upper:g} m')
    distance.plot(times,ranges,color='#2167b5');distance.set_xlabel('Simulated time (s)');distance.set_ylabel('Target distance (m)');distance.grid(alpha=.2)
    distance.legend(fontsize=8)
    speeds.plot([r['time_s'] for r in rows],[r['target_speed_knots'] for r in rows],color='#e07b13',label='Target')
    speeds.plot([r['time_s'] for r in rows],[r['own_speed_knots'] for r in rows],color='#2167b5',label='Vessel')
    speeds.set_xlabel('Simulated time (s)');speeds.set_ylabel('Speed (knots)');speeds.grid(alpha=.2);speeds.legend(fontsize=8)
    fig.suptitle(f"{LABELS[metadata['scenario']]} | {metadata['mode']} | {summary['simulated_seconds']:.0f}s\n"
                f"In band: {summary['in_band_fraction']:.1%}; range: {summary['min_distance_m']:.1f}–{summary['max_distance_m']:.1f}m; "
                f"safety holds: {summary['safety_hold_steps']}",fontsize=14)
    result=folder/'routes.png';fig.savefig(result,dpi=155);plt.close(fig)
    return result.resolve()


def run(name,output,*,endpoint,dt=2.,duration=None,mock=False,task_config=DEFAULT_TASK,initial_distance=300.,straight_offset=75.):
    task=json.loads((TASKS/'task_one.json').read_text())
    static=json.loads((TASKS/'static_situation.json').read_text())
    frame=LocalFrame(task['params']['planWaypointList'])
    route=[frame.project(p['routeLon'],p['routeLat']) for p in task['params']['planWaypointList']]
    anchor=route[2]
    center=(anchor[0]+500.,anchor[1]+100.)
    target=center if name=='crossing_variable' else anchor
    theta=0.
    course=target_course(name,0.,theta);speed=target_speed(name,0.)
    yaw_rate=0.
    packet=json.loads(Path(task_config).read_text())
    from server.navigation.target_following import validate_task
    validate_task(packet)
    packet.update(simId='task-one-following',id=name)
    lower=packet['params']['minFollowingDistanceM'];upper=packet['params']['maxFollowingDistanceM']
    initial_range=initial_distance
    lateral=straight_offset if name=='straight' else 0.
    if not math.isfinite(initial_range) or initial_range<=upper or not math.isfinite(lateral) or abs(lateral)>=initial_range:
        raise ValueError('initial distance must exceed maximum following distance and lateral offset')
    along=math.sqrt(initial_range**2-lateral**2)
    angle=math.radians(course)
    own=(target[0]-along*math.sin(angle)-lateral*math.cos(angle),
         target[1]-along*math.cos(angle)+lateral*math.sin(angle))
    policy=KevPolicy(endpoint=endpoint,duration=dt,client=mock_choice if mock else None)
    folder=Path(output)/name;folder.mkdir(parents=True,exist_ok=True)
    metadata=dict(scenario=name,mode='mock' if mock else 'real_kev',dt_s=dt,
        task_config_source=str(Path(task_config).resolve()),min_following_distance_m=lower,max_following_distance_m=upper,
        initial_distance_m=initial_range,initial_cross_track_m=lateral,
        duration_s=duration or DURATIONS[name],map_source=str(TASKS/'task_one.json'),
        obstacle_source=str(TASKS/'static_situation.json'),map_origin_lon_lat=frame.origin,
        map_route_xy_m=route,obstacles_xy_m=[dict(id=r['platformId'],xy=frame.project(r['longitude'],r['latitude'])) for r in static],
        target_start_xy_m=target,own_start_xy_m=own,collision_radius_m=policy.obstacle_config.collision_radius_m,
        vessel_motion='50kn max; accel 1.46m/s², braking 1.75m/s², yaw 12deg/s, yaw acceleration 4deg/s²; independent 0.02s position integration',
        target_motion='10kn straight; 10kn sinusoidal course; arc-length figure-eight with 6–18kn sinusoidal speed',
        timing='Offline sequential inference; simulated world paused during HTTP requests, not real-time UDP')
    dump(folder/'configuration.json',metadata);dump(folder/'task.json',packet)
    dump(folder/'map-task-one.json',task);dump(folder/'static-situation.json',static)
    end=metadata['duration_s'];t=0.;minimum=math.inf;maximum=0.;band=0;count=0;holds=0
    min_static=math.inf;target_min_static=math.inf;below=0;above=0;max_decision=0.;rows=[]
    def static_clearance(point):
        return min(math.dist(point,o['xy'])-metadata['collision_radius_m'] for o in metadata['obstacles_xy_m'])
    with (folder/'trajectory.jsonl').open('w') as log:
        while t < end-1e-9:
            step=min(dt,end-t)
            target_velocity=target_speed(name,t);target_heading=target_course(name,t,theta)
            records=(dict(to_record(frame,packet['params']['platCode'],own,speed,course),_turn_rate_deg_s=yaw_rate),
                     to_record(frame,packet['params']['targetPlatformId'],target,target_velocity,target_heading),*static)
            started=time.perf_counter();action=policy(dict(packet,situation=records));elapsed=time.perf_counter()-started
            max_decision=max(max_decision,elapsed)
            own,speed,course,own_points,yaw_rate=integrate(own,speed,course,action,step,initial_turn_rate_deg_s=yaw_rate,return_turn_rate=True,config=policy.dynamic_config)
            target,theta,target_points=advance_target(name,t,step,target,theta,center)
            for a,b in zip(own_points,target_points):
                d=math.dist(a,b);minimum=min(minimum,d);maximum=max(maximum,d)
                band+=int(lower<=d<=upper);below+=int(d<lower);above+=int(d>upper);count+=1
                min_static=min(min_static,static_clearance(a));target_min_static=min(target_min_static,static_clearance(b))
            t+=step
            decision=policy.local.last_decision
            holds+=int(decision['selection_mode']=='following_safety_hold')
            row=dict(time_s=t,own_xy_m=own,target_xy_m=target,distance_m=math.dist(own,target),
                own_speed_knots=speed,own_course_deg=course,target_speed_knots=target_speed(name,t),
                target_course_deg=target_course(name,t,theta),target_theta_rad=theta,command=list(action),
                inference_s=elapsed,selection_mode=decision['selection_mode'],recovery_phase=decision['state']['recovery_phase'],
                choice=decision.get('choice'),candidate_count=decision['candidate_count'])
            rows.append(row);log.write(json.dumps(row,allow_nan=False)+'\n');log.flush()
            if len(rows)%10==0:
                print(json.dumps(dict(progress=name,simulated_s=t,duration_s=end,distance_m=row['distance_m'],holds=holds)),flush=True)
    summary=dict(scenario=name,mode=metadata['mode'],simulated_seconds=t,steps=len(rows),
        min_distance_m=minimum,max_distance_m=maximum,final_distance_m=rows[-1]['distance_m'],
        in_band_fraction=band/count,below_min_fraction=below/count,above_max_fraction=above/count,
        safety_hold_steps=holds,min_static_clearance_m=min_static,target_min_static_clearance_m=target_min_static,
        max_decision_s=max_decision,target_completed_loops=theta/(2*math.pi),
        target_speed_min_knots=min(r['target_speed_knots'] for r in rows),
        target_speed_max_knots=max(r['target_speed_knots'] for r in rows))
    summary.update(recovery_metrics(rows,lower,upper))
    dump(folder/'summary.json',summary)
    image=plot_result(folder)
    print(json.dumps(dict(completed=name,summary=summary,image=str(image)),allow_nan=False),flush=True)
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario',choices=['all',*DURATIONS],default='all')
    p.add_argument('--output',type=Path,default=Path('server/experiments/results/task-one-following-100-150-20261028'))
    p.add_argument('--task-config',type=Path,default=DEFAULT_TASK)
    p.add_argument('--kev-endpoint',default='http://127.0.0.1:18028/v1/systemone')
    p.add_argument('--dt',type=float,default=2.)
    p.add_argument('--duration',type=float)
    p.add_argument('--mock',action='store_true')
    p.add_argument('--initial-distance-m',type=float,default=300.)
    p.add_argument('--straight-offset-m',type=float,default=75.)
    args=p.parse_args()
    if not math.isfinite(args.dt) or args.dt<=0 or (args.duration is not None and (not math.isfinite(args.duration) or args.duration<=0)):
        p.error('dt and duration must be positive and finite')
    for name in DURATIONS if args.scenario=='all' else (args.scenario,):
        run(name,args.output,endpoint=args.kev_endpoint,dt=args.dt,duration=args.duration,mock=args.mock,task_config=args.task_config,
            initial_distance=args.initial_distance_m,straight_offset=args.straight_offset_m)


if __name__=='__main__':main()
