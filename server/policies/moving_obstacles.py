"""Moving occupancy for the shared geometric navigator, never collision certification.

Safety remains the dynamic planner's time-dependent prediction. These overlapping
circles keep its route reference outside the slow vessel's upcoming corridor.
"""
import math
if __package__.startswith('server.'):
    from ..navigation.dynamic_avoidance.types import finite
else:
    from navigation.dynamic_avoidance.types import finite


def slow_obstacle_geometry(scene, geometry, cruise_mps, config, *, initial_position=None):
    if scene is None:
        return [], [], False
    own, targets = scene
    goal = geometry.waypoint_positions[geometry.next_index]
    start = geometry.waypoint_positions[geometry.next_index-1] if geometry.next_index else (initial_position or own.position_m)
    dx, dy = goal[0]-start[0], goal[1]-start[1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return [], [], False
    direction = (dx/length, dy/length)
    bearing = math.degrees(math.atan2(*direction)) % 360
    obstacles, ids, lateral_clear = [], [], True
    radius = config.safe_distance_m + config.uncertainty_mps*config.action_horizon_s
    for target in targets:
        if (target.position_m is None or not finite(target.speed_mps) or
                not finite(target.course_deg) or target.age_s > config.max_target_age_s or
                target.speed_mps >= cruise_mps-.5 or
                abs((target.course_deg-bearing+180)%360-180) > 30.):
            continue
        heading = math.radians(target.course_deg)
        velocity = (target.speed_mps*math.sin(heading), target.speed_mps*math.cos(heading))
        center = tuple(target.position_m[i]+velocity[i]*target.age_s for i in (0,1))
        offset = tuple(center[i]-own.position_m[i] for i in (0,1))
        along = sum(offset[i]*direction[i] for i in (0,1))
        cross = abs(offset[0]*direction[1]-offset[1]*direction[0])
        # Keep geometry until the entire moving corridor is safely astern.
        if along < -radius-20-target.speed_mps*20. or along > max(700.,cruise_mps*30.) or cross > radius+100.:
            continue
        ids.append(target.target_id)
        lateral_clear &= cross >= config.safe_distance_m-10.
        intervals = max(2, math.ceil(target.speed_mps*20./radius))
        for sample in range(intervals+1):
            seconds = 20.*sample/intervals
            point = tuple(center[i]+seconds*velocity[i] for i in (0,1))
            # Geometry may be conservative, but must not turn a predicted
            # future occupancy into a fatal "inside static obstacle" error.
            if math.dist(own.position_m, point) <= radius:
                continue
            obstacles.append(dict(id=f'moving:{target.target_id}:{sample}',xy=point,
                                  radius_m=radius,age_s=target.age_s))
    return obstacles, ids, bool(ids and lateral_clear)


def rejoin_clearance(scene, geometry, config, *, initial_position, entry_speed_mps=5.1444444444, braking_mps2=.9):
    """Gate crossing back toward moving occupancy, without a passing phase.

    A visible waypoint beyond a swept disk is not permission to merge. Keep
    the present side until there is longitudinal room for the complete merge,
    including relative closing, yaw build-up and two control periods.
    """
    if scene is None:
        return None
    own, targets = scene
    goal = geometry.waypoint_positions[geometry.next_index]
    start = geometry.waypoint_positions[geometry.next_index-1] if geometry.next_index else initial_position
    length = math.dist(start,goal)
    if length < 1e-6:
        return None
    forward = tuple((goal[i]-start[i])/length for i in (0,1))
    right = (forward[1],-forward[0])
    course = math.degrees(math.atan2(*forward))%360
    blockers=[]
    for target in targets:
        if (target.position_m is None or not finite(target.speed_mps) or not finite(target.course_deg)
                or target.age_s>config.max_target_age_s or
                abs((target.course_deg-course+180)%360-180)>30.):
            continue
        heading=math.radians(target.course_deg)
        velocity=(target.speed_mps*math.sin(heading),target.speed_mps*math.cos(heading))
        center=tuple(target.position_m[i]+velocity[i]*target.age_s for i in (0,1))
        relative=tuple(own.position_m[i]-center[i] for i in (0,1))
        lead=sum(relative[i]*forward[i] for i in (0,1))
        lateral=sum(relative[i]*right[i] for i in (0,1))
        if abs(lateral)<config.safe_distance_m or abs(lateral)>config.safe_distance_m+150 or lead < -200.:
            continue
        merge_time=max(config.action_horizon_s,abs(lateral)/(max(own.speed_mps,1.)*.5))
        merge_time+=2*config.max_turn_rate_deg_s/config.max_turn_acceleration_deg_s2
        target_forward=sum(velocity[i]*forward[i] for i in (0,1))
        closing=max(0.,target_forward-own.speed_mps*math.cos(math.radians(30.)))
        required=config.safe_distance_m+20.+config.uncertainty_mps*(target.age_s+merge_time)
        required+=2*target_forward+closing*merge_time
        braking_distance=max(0.,own.speed_mps**2-entry_speed_mps**2)/(2*braking_mps2)+2*own.speed_mps
        needs_braking=(own.speed_mps>entry_speed_mps+.05 and
                       math.dist(own.position_m,goal)<geometry.radius+braking_distance)
        if lead < required or (needs_braking and lead<required+400.):
            blockers.append(dict(id=target.target_id,lead_m=lead,required_lead_m=required,
                                 lateral_m=lateral,merge_preview_s=merge_time,needs_lead=lead<required,
                                 needs_braking=needs_braking))
    if not blockers:
        return None
    nearest=min(blockers,key=lambda t:abs(t['lead_m']))
    side=1 if nearest['lateral_m']>0 else -1
    # Widen gently enough to remain outside a nearby waypoint speed circle.
    advance=max(100.,own.speed_mps*config.action_horizon_s)
    # Build the clearance before leveling out. A distant point with only a
    # small lateral offset otherwise flattens the turn before the lane is clear.
    outward=(advance*math.tan(math.radians(15.))
             if abs(nearest['lateral_m'])<config.safe_distance_m+25. else 0.)
    guidance=tuple(own.position_m[i]+advance*forward[i]+outward*side*right[i] for i in (0,1))
    return dict(blocked=True,targets=blockers,course_deg=course,side=side,
                needs_lead=any(t['needs_lead'] for t in blockers),
                guidance_xy_m=guidance,reason='moving_corridor_not_clear_for_rejoin')
