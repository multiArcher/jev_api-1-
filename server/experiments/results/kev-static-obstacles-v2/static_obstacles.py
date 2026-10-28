"""Static target geometry, visibility paths and swept-motion collision checks.

Only internal metre coordinates are used. Unknown target types and friendly
stationary vessels remain physical obstacles; moving targets are not predicted.
"""
from dataclasses import dataclass
import heapq
import math

from tools.navigation_motion import predict_ground_motion
from tools.waypoint_geometry import KNOT_MPS, LocalFrame


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

    def guide(self,geometry,obstacles):
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
        if clear_segment(start,target,obstacles,padding):
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
        slow=any(math.dist(start,o['xy'])-o['radius_m'] < 300 and
                 (detour or sum((o['xy'][i]-start[i])*(guidance[i]-start[i]) for i in (0,1)) > 0)
                 for o in nearby)
        # Brake sufficiently early for a high-speed approach; actual obstacle
        # checks below include bounded acceleration and a full stopping tail.
        return dict(guidance_xy_m=guidance,detour=detour,nearest_clearance_m=nearest,
                    slow=slow,path=list(self.path),obstacles=nearby)

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
    """Check entire swept polylines plus stopping tails, not just endpoints.

    Test both the current simulator response and the supplied longitudinal
    acceleration bounds. Yaw still uses the configured rate-only approximation.
    """
    if not obstacles:
        return dict(safe=True,min_clearance_m=None,obstacle_id=None)
    current=boat['targetSpeed']*KNOT_MPS
    requested=speed_knots*KNOT_MPS
    transition=abs(requested-current)/(config.acceleration_mps2 if requested>current else config.deceleration_mps2)
    smallest=float('inf');nearest=None
    for speed_time in set((execution['speed_transition_time_s'],transition)):
        horizon=max(config.horizon_s,2*duration,speed_time)
        motion=predict_ground_motion(current,boat['targetCourse'],requested,course,
            speed_time,execution['course_transition_time_s'],horizon,dt=.1,
            max_turn_rate_deg_s=execution.get('max_turn_rate_deg_s'))
        path=[(geometry.boat_position[0]+p[0],geometry.boat_position[1]+p[1]) for p in motion['path']]
        stopping=motion['end_speed']**2/(2*config.deceleration_mps2)
        heading=math.radians(motion.get('end_course_deg',course))
        path.append((path[-1][0]+stopping*math.sin(heading),path[-1][1]+stopping*math.cos(heading)))
        for obstacle in obstacles:
            clearance=min(segment_distance(obstacle['xy'],a,b) for a,b in zip(path,path[1:]))-obstacle['radius_m']
            if clearance < smallest:
                smallest,nearest=clearance,obstacle['id']
    return dict(safe=smallest>.5,min_clearance_m=smallest,obstacle_id=nearest)
