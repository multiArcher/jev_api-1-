"""Static target geometry, visibility paths and swept-motion collision checks.

Only internal metre coordinates are used. Unknown target types and friendly
stationary vessels remain physical obstacles; moving targets are not predicted.
"""
from dataclasses import dataclass
import heapq
import logging
import math
from pathlib import Path
import time

from tools.navigation_motion import predict_ground_motion
from tools.acceleration_response import response_applies, measured_speed_profile
from tools.vessel_dynamics import motion_parameters, turn_profile, positive_limit
from tools.waypoint_geometry import KNOT_MPS, LocalFrame


ACTUAL_DISTANCE_LOGGER = logging.getLogger(__name__ + '.actual_distance')
ACTUAL_DISTANCE_LOGGER.propagate = False
ACTUAL_DISTANCE_LOGGER.addHandler(logging.NullHandler())


def configure_actual_distance_log(path):
    """Open once per service start; tasks share this file without truncating it."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    handler = logging.FileHandler(path, mode='w', encoding='utf-8')
    handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
    for old in list(ACTUAL_DISTANCE_LOGGER.handlers):
        ACTUAL_DISTANCE_LOGGER.removeHandler(old)
        old.close()
    ACTUAL_DISTANCE_LOGGER.addHandler(handler)
    ACTUAL_DISTANCE_LOGGER.setLevel(logging.INFO)
    return path


class StaticObstacleBlocked(ValueError):
    error_code = 'STATIC_OBSTACLE_BLOCKED'


@dataclass(frozen=True)
class StaticObstacleConfig:
    obstacle_radius_m: float = 80.
    vessel_length_m: float = 11.6
    vessel_width_m: float = 3.4
    safety_margin_m: float = 0.
    path_padding_m: float = 10.
    stationary_speed_knots: float = .1
    horizon_s: float = 4.
    deceleration_mps2: float = 1.75
    acceleration_mps2: float = 1.46

    def __post_init__(self):
        positive_limit('acceleration_mps2', self.acceleration_mps2, 1.46)
        positive_limit('deceleration_mps2', self.deceleration_mps2, 1.75)
        for key,value in self.__dict__.items():
            if not math.isfinite(value) or value < 0 or (
                    key not in ('safety_margin_m','stationary_speed_knots') and value == 0):
                raise ValueError(f'{key} must be finite and positive (margins may be zero)')

    @property
    def collision_radius_m(self):
        return self.obstacle_radius_m + math.hypot(self.vessel_length_m,self.vessel_width_m)/2 + self.safety_margin_m


def segment_distance(point,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    norm=dx*dx+dy*dy
    t=max(0.,min(1.,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/norm)) if norm else 0.
    return math.hypot(point[0]-a[0]-t*dx,point[1]-a[1]-t*dy)


class ActualDistanceMonitor:
    """Task-local telemetry measurements, independent of model predictions."""
    def __init__(self, logger, task, max_gap_s=2.5):
        self.logger, self.task = logger, task
        self.max_gap_s = max_gap_s
        self.previous = {}
        self.active = {}

    def update(self, position, obstacles, now=None):
        now = time.monotonic() if now is None else now
        present = set()
        for obstacle in obstacles:
            oid, center, radius = obstacle['id'], obstacle['xy'], obstacle['radius_m']
            present.add(oid)
            distance = math.dist(position, center)
            previous = self.previous.get(oid)
            swept = distance
            interpolated = False
            if previous and 0 < now-previous[0] <= self.max_gap_s and previous[2] == center:
                swept = segment_distance(center, previous[1], position)
                interpolated = True
            self.previous[oid] = (now, position, center)
            if distance > 125.:
                if oid in self.active:
                    self._finish(oid, 'left_monitoring_range')
                continue
            entry = self.active.setdefault(oid, dict(min_distance_m=float('inf'), violations=0))
            entry['min_distance_m'] = min(entry['min_distance_m'], swept)
            entry['radius_m'] = radius
            unsafe = swept <= radius
            entry['violations'] += int(unsafe)
            log = self.logger.warning if unsafe else self.logger.info
            log('static obstacle actual distance task=%s platCode=%s obstacle=%s '
                'center_distance_m=%.3f segment_min_distance_m=%.3f safe_distance_m=%.3f '
                'clearance_m=%.3f safe=%s interpolated=%s obstacle_age_s=%s',
                self.task.get('id'), self.task['params']['platCode'], oid,
                distance, swept, radius, swept-radius, not unsafe, interpolated, obstacle.get('age_s'))
        for oid in set(self.active)-present:
            self._finish(oid, 'target_no_longer_static')
        for oid in set(self.previous)-present:
            del self.previous[oid]

    def _finish(self, oid, reason):
        entry = self.active.pop(oid)
        log = self.logger.warning if entry['violations'] else self.logger.info
        log('static obstacle actual summary task=%s platCode=%s obstacle=%s '
            'min_distance_m=%.3f safe_distance_m=%.3f min_clearance_m=%.3f '
            'unsafe_samples=%d reason=%s', self.task.get('id'), self.task['params']['platCode'],
            oid, entry['min_distance_m'], entry['radius_m'],
            entry['min_distance_m']-entry['radius_m'], entry['violations'], reason)

    def close(self):
        for oid in list(self.active):
            self._finish(oid, 'task_ended')


def static_targets(records,own_id,frame,config):
    """Retain last-known stationary targets even after a detection stops updating."""
    targets={}
    for record in records:
        if record.get('platformId') == own_id:
            continue
        lon,lat,speed=record.get('longitude'),record.get('latitude'),record.get('targetSpeed')
        if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
                   for v in (lon,lat,speed)):
            continue
        if not -180 <= lon <= 180 or not -90 <= lat <= 90 or abs(speed) > config.stationary_speed_knots:
            continue
        pid=record.get('platformId')
        if type(pid) is not int:
            continue
        targets[pid]=dict(id=pid,xy=frame.project(lon,lat),radius_m=config.collision_radius_m,
                         age_s=record.get('_received_age_s',0.))
    return [targets[k] for k in sorted(targets)]


def clear_segment(a,b,obstacles,padding=0.):
    for obstacle in obstacles:
        radius=obstacle['radius_m']+padding
        clearance=segment_distance(obstacle['xy'],a,b)
        if clearance >= radius-1e-6:
            continue
        # A tracking error may enter the planning padding, never the collision
        # circle. Allow a path that monotonically exits that padding.
        start=math.dist(a,obstacle['xy'])
        outward=sum((a[i]-obstacle['xy'][i])*(b[i]-a[i]) for i in (0,1)) >= 0
        if start > obstacle['radius_m']+1e-4 and outward and clearance >= start-1e-5:
            continue
        return False
    return True


class StaticNavigator:
    """Cache a geometric detour; Kev still selects each speed/course action."""
    def __init__(self,config):
        self.config=config
        self.key=None
        self.path=[]

    def guide(self,geometry,obstacles,*,boat=None,execution=None,duration=1.,speed_knots=50.):
        start=geometry.boat_position
        target=geometry.waypoint_positions[geometry.next_index]
        for obstacle in obstacles:
            if math.dist(start,obstacle['xy']) <= obstacle['radius_m']:
                raise StaticObstacleBlocked(f"本艇已处于静态目标{obstacle['id']}的避碰范围内")
        padding=self.config.path_padding_m
        nearby=[o for o in obstacles if math.dist(start,o['xy']) <= 500 or
                segment_distance(o['xy'],start,target) <= o['radius_m']+200]
        if not nearby:
            return None
        signature=(geometry.next_index,tuple((o['id'],o['xy'],o['radius_m']) for o in nearby))
        direct=clear_segment(start,target,obstacles,padding)
        if not direct:
            approach=self._approach(start,target,nearby,boat,execution,duration,speed_knots)
            if approach is not None:
                # Keep an explicit straight leg. Visibility shortcuts must not
                # skip this leg merely because a distant detour vertex is visible.
                self.key=signature
                self.path=[approach]+self._plan(approach,target,geometry.radius,nearby,obstacles)
                return dict(guidance_xy_m=approach,detour=True,approaching=True,
                    nearest_clearance_m=min(math.dist(start,o['xy'])-o['radius_m'] for o in nearby),
                    slow=False,path=list(self.path),obstacles=nearby)
        if direct:
            self.key,self.path=signature,[target]
        elif self.key==signature and self.path:
            # Keep the chosen side until a farther path vertex is visible.
            visible=[i for i in range(len(self.path)) if clear_segment(start,self.path[i],obstacles,padding)]
            if visible:
                self.path=self.path[max(visible):]
            else:
                self.path=[]
        else:
            self.path=[]
        if not self.path:
            self.key=signature
            self.path=self._plan(start,target,geometry.radius,nearby,obstacles)
        guidance=self.path[0]
        detour=math.dist(guidance,target)>1.
        nearest=min(math.dist(start,o['xy'])-o['radius_m'] for o in nearby)
        return dict(guidance_xy_m=guidance,detour=detour,nearest_clearance_m=nearest,
                    approaching=False,slow=False,path=list(self.path),obstacles=nearby)

    def _approach(self,start,target,obstacles,boat,execution,duration,speed_knots):
        """Place the turn-in point on the direct leg, outside the safety reserve.

        Reserve both the prediction+stopping distance and the turn geometry,
        plus two control periods. Faster or slower-turning boats start earlier.
        """
        length=math.dist(start,target)
        if length < 1e-6:
            return None
        direction=tuple((target[i]-start[i])/length for i in (0,1))
        entries=[]
        for obstacle in obstacles:
            offset=tuple(obstacle['xy'][i]-start[i] for i in (0,1))
            along=sum(offset[i]*direction[i] for i in (0,1))
            lateral2=max(0.,sum(v*v for v in offset)-along*along)
            radius=obstacle['radius_m']+self.config.path_padding_m
            if lateral2 < radius*radius:
                entry=along-math.sqrt(radius*radius-lateral2)
                if entry < length and along+math.sqrt(radius*radius-lateral2) > 0:
                    entries.append((entry,radius))
        if not entries:
            return None
        entry,radius=min(entries)
        current=(boat['targetSpeed'] if boat else speed_knots)*KNOT_MPS
        requested=speed_knots*KNOT_MPS
        speed=max(current,requested)
        execution=execution or {}
        turn_rate=min([execution.get('max_turn_rate_deg_s',12.)]+[p[1] for p in execution.get('turn_rate_curve',())])
        angular_acceleration=execution.get('max_turn_acceleration_deg_s2',4.)
        initial_rate=abs(boat.get('_turn_rate_deg_s',0.)) if boat else 0.
        turn_radius=speed/math.radians(turn_rate)
        # Include the time to brake existing yaw and build up a new turn.
        turn_lead=math.sqrt(radius*radius+2*turn_radius*radius)-radius
        turn_lead += speed*(turn_rate+initial_rate)/angular_acceleration
        accel=min([self.config.acceleration_mps2 if requested>current else self.config.deceleration_mps2]
                  +[p[1] for p in execution.get('acceleration_curve' if requested>current else 'deceleration_curve',())])
        braking=min([self.config.deceleration_mps2]+[p[1] for p in execution.get('deceleration_curve',())])
        transition=abs(requested-current)/accel
        horizon=max(self.config.horizon_s,2*duration,transition)
        distance=.5*(current+requested)*transition+requested*(horizon-transition)
        stop_lead=distance+requested*requested/(2*braking)
        desired_course=math.degrees(math.atan2(direction[0],direction[1]))%360
        if boat and response_applies(execution.get('acceleration_response','measured_0_50kn'),current,requested,
                                     boat.get('targetCourse'),desired_course,boat.get('_turn_rate_deg_s',0.)):
            response_speed,times=measured_speed_profile(current,requested,self.config.acceleration_mps2,
                                                        tuple(execution.get('acceleration_curve',())))
            boundaries=[0.,*times]
            distance=sum((response_speed(begin)+response_speed(end))*.5*(end-begin)
                         for begin,end in zip(boundaries,boundaries[1:]))
            distance+=requested*max(0.,max(self.config.horizon_s,2*duration)-times[-1])
            stop_lead=max(stop_lead,distance+requested*requested/(2*braking))
        lead=max(stop_lead,turn_lead)+2*speed*duration+self.config.path_padding_m
        travel=entry-lead
        if travel <= max(2.,speed*duration):
            return None
        approach=tuple(start[i]+direction[i]*travel for i in (0,1))
        return approach if clear_segment(start,approach,obstacles,self.config.path_padding_m) else None

    def _plan(self,start,target,arrival,nearby,obstacles):
        pad=self.config.path_padding_m
        if clear_segment(start,target,obstacles,pad):
            return [target]
        goals=[]
        if all(math.dist(target,o['xy']) > o['radius_m']+pad for o in obstacles):
            goals.append(target)
        # An occupied waypoint is a safe arrival region, not a demand to reach
        # its center. These points remain strictly inside the unchanged circle.
        goal_radius=max(0.,arrival-2.)
        for i in range(32):
            angle=2*math.pi*i/32
            point=(target[0]+goal_radius*math.cos(angle),target[1]+goal_radius*math.sin(angle))
            if all(math.dist(point,o['xy']) > o['radius_m']+pad for o in obstacles):
                goals.append(point)
        if not goals:
            raise StaticObstacleBlocked('航路点到达圈内没有满足静态避碰距离的可用通过位置')
        nodes=[start]+goals
        goal_ids=set(range(1,len(nodes)))
        sides=20
        for obstacle in nearby:
            # Circumscribe the padded circle: adjacent polygon chords must
            # also stay outside it, rather than cutting across the circle.
            radius=(obstacle['radius_m']+pad)/math.cos(math.pi/sides)+.05
            for i in range(sides):
                angle=2*math.pi*i/sides
                p=(obstacle['xy'][0]+radius*math.cos(angle),obstacle['xy'][1]+radius*math.sin(angle))
                if all(math.dist(p,o['xy']) > o['radius_m']+pad-1e-5 for o in obstacles):
                    nodes.append(p)
        costs={0:0.};previous={};queue=[(0.,0)];visited=set();end=None
        while queue:
            cost,index=heapq.heappop(queue)
            if index in visited:
                continue
            if index in goal_ids:
                end=index;break
            visited.add(index)
            for j,p in enumerate(nodes):
                if j in visited or j==index:
                    continue
                candidate=cost+math.dist(nodes[index],p)
                if candidate >= costs.get(j,float('inf')) or not clear_segment(nodes[index],p,obstacles,pad):
                    continue
                costs[j]=candidate;previous[j]=index;heapq.heappush(queue,(candidate,j))
        if end is None:
            raise StaticObstacleBlocked('静态障碍物阻断了到当前航路点的可用绕行路径')
        route=[]
        while end:
            route.append(nodes[end]);end=previous[end]
        return route[::-1]


def motion_clearance(geometry,boat,speed_knots,course,obstacles,execution,duration,config):
    """Reject if either the nominal response or faster physical response is unsafe."""
    first = _motion_clearance(geometry,boat,speed_knots,course,obstacles,execution,duration,config)
    mode = execution.get('acceleration_response','measured_0_50kn')
    if not obstacles or not response_applies(mode,boat['targetSpeed']*KNOT_MPS,min(speed_knots,50.)*KNOT_MPS,
                                            boat['targetCourse'],course,boat.get('_turn_rate_deg_s',0.)):
        return first
    second = _motion_clearance(geometry,boat,speed_knots,course,obstacles,
                               dict(execution,acceleration_response='bounded'),duration,config)
    return min((first,second),key=lambda item:item['min_clearance_m'])


def _motion_clearance(geometry,boat,speed_knots,course,obstacles,execution,duration,config):
    """Check entire swept polylines plus stopping tails, not just endpoints.

    Use the shared longitudinal and angular acceleration bounds, including
    continuous rotation and the complete physical braking tail.
    """
    if not obstacles:
        return dict(safe=True,min_clearance_m=None,obstacle_id=None)
    current=boat['targetSpeed']*KNOT_MPS
    requested=min(speed_knots,50.)*KNOT_MPS
    parameters=motion_parameters(execution)
    parameters.update(acceleration_mps2=config.acceleration_mps2, deceleration_mps2=config.deceleration_mps2,
                      initial_turn_rate_deg_s=boat.get('_turn_rate_deg_s',parameters.get('initial_turn_rate_deg_s',0.)))
    accel=min([config.acceleration_mps2 if requested>current else config.deceleration_mps2]
              +[p[1] for p in parameters.get('acceleration_curve' if requested>current else 'deceleration_curve',())])
    transition=abs(requested-current)/accel
    if response_applies(parameters['acceleration_response'],current,requested,boat['targetCourse'],course,parameters.get('initial_turn_rate_deg_s',0.)):
        _,response_times=measured_speed_profile(current,requested,config.acceleration_mps2,parameters.get('acceleration_curve',()))
        transition=max(transition,response_times[-1])
    turn=(course-boat['targetCourse']+180)%360-180 if requested else boat.get('_turn_rate_deg_s',0.)*abs(boat.get('_turn_rate_deg_s',0.))/8.
    rate=min([parameters.get('max_turn_rate_deg_s',12.)]+[p[1] for p in parameters.get('turn_rate_curve',())])
    _,turns=turn_profile(turn,parameters.get('initial_turn_rate_deg_s',0.),rate,parameters.get('max_turn_acceleration_deg_s2',4.))
    horizon=max(config.horizon_s,2*duration,transition,max(turns,default=0.))
    motion=predict_ground_motion(current,boat['targetCourse'],requested,course,0.,0.,horizon,dt=.1,**parameters)
    path=[(geometry.boat_position[0]+p[0],geometry.boat_position[1]+p[1]) for p in motion['path']]
    deceleration=min([config.deceleration_mps2]+[p[1] for p in parameters.get('deceleration_curve',())])
    braking_time=motion['end_speed']/deceleration
    if braking_time:
        parameters['initial_turn_rate_deg_s']=motion['end_turn_rate_deg_s']
        tail=predict_ground_motion(motion['end_speed'],motion['end_course_deg'],0.,motion['end_course_deg'],
                                   0.,0.,braking_time,dt=.1,**parameters)
        end=path[-1]
        path.extend((end[0]+p[0],end[1]+p[1]) for p in tail['path'][1:])
    smallest=float('inf');nearest=None
    for obstacle in obstacles:
        clearance=min(segment_distance(obstacle['xy'],a,b) for a,b in zip(path,path[1:]))-obstacle['radius_m']
        if clearance < smallest:
            smallest,nearest=clearance,obstacle['id']
    return dict(safe=smallest>.5,min_clearance_m=smallest,obstacle_id=nearest)
