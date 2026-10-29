"""Waypoint lookahead and soft planned-speed references; course choice stays with Kev."""
import math

from tools.waypoint_geometry import KNOT_MPS
from tools.vessel_dynamics import curve_limit, turn_profile
from .turning import ALIGNMENT_ENTER_DEG


def _bearing(vector):
    return math.degrees(math.atan2(vector[0], vector[1])) % 360


def _turn_speed(angle):
    for threshold, speed in ((15., 50.), (45., 40.), (90., 30.), (135., 20.)):
        if angle <= threshold:
            return speed
    return 10.


def plan_speed(observation, geometry, speeds, turn_rate_deg_s, recovering=False, *, apply_heading_cap=True, detouring=False):
    """Plan cruise/approach speeds from the next leg and existing motion assumptions.

    Brake lead includes physical braking distance, two command periods and
    travel during the complete acceleration-limited turn at entry speed.
    Speed-dependent curves remain provisional until calibrated.
    """
    task, boat = observation['task'], observation['boat']
    index = task['next_waypoint_index']
    points = geometry.waypoint_positions
    target = points[index]
    incoming = geometry.target if index == 0 else (
        target[0] - points[index - 1][0], target[1] - points[index - 1][1])
    if math.hypot(*incoming) < 1e-6:
        incoming = geometry.target
    next_index = next((i for i in range(index + 1, len(points))
                       if math.dist(points[i], target) > 1e-6), None)
    next_bearing = None
    turn = 0.
    observed_turn = 0.
    if next_index is not None:
        outgoing = (points[next_index][0] - target[0], points[next_index][1] - target[1])
        next_bearing = _bearing(outgoing)
        turn = abs((next_bearing - _bearing(incoming) + 180) % 360 - 180)
        observed_turn = abs((next_bearing - boat['targetCourse'] + 180) % 360 - 180)

    cruise = max(speeds)
    limit = task['planWaypointList'][index]['routeSpeed'] / KNOT_MPS
    entry_cap = min(limit, _turn_speed(turn)) if next_index is not None else min(limit, 10.)
    entry = max((s for s in speeds if s <= entry_cap), default=min(speeds))
    planning_speed = max(boat['targetSpeed'], cruise) * KNOT_MPS
    entry_mps = entry * KNOT_MPS
    execution = observation['execution']
    braking_limit = min([execution.get('deceleration_mps2', 1.75),execution.get('planning_deceleration_mps2',1.75)] + [p[1] for p in execution.get('deceleration_curve', ())])
    brake = max(0., planning_speed**2-entry_mps**2)/(2*braking_limit)
    reaction = planning_speed * 2 * observation['planning']['action_duration_s']
    turn_cap = curve_limit(execution.get('turn_rate_curve', ()), entry_mps, min(turn_rate_deg_s, 12.))
    _, turn_times = turn_profile(turn, rate_limit=turn_cap, acceleration=execution.get('max_turn_acceleration_deg_s2',4.))
    turn_arc = entry_mps * max(turn_times, default=0.)
    approach_distance = task['arrival_radius_m'] + brake + reaction + turn_arc
    distance = math.hypot(*geometry.target)
    heading_error = abs((_bearing(geometry.target) - boat['targetCourse'] + 180) % 360 - 180)

    cap, phase = cruise, 'cruise'
    if distance <= approach_distance and entry < cap:
        cap, phase = entry, 'approach'
    if distance > task['arrival_radius_m'] and distance <= approach_distance:
        # Do not force a speed match merely because cruise-speed brake lead
        # reached the waypoint. Preserve two control periods and the entry turn.
        available = max(0.,distance-task['arrival_radius_m']-turn_arc-1.)
        reaction_time = 2*observation['planning']['action_duration_s']
        reserve = braking_limit*reaction_time
        safe_speed = (math.sqrt(reserve*reserve+entry_mps*entry_mps+2*braking_limit*available)-reserve)/KNOT_MPS
        cap,phase = min(cruise,max(entry,safe_speed)),('detour_approach' if detouring else 'approach')
    if apply_heading_cap and not detouring and heading_error > ALIGNMENT_ENTER_DEG and observation.get('dynamic_scene') is None:
        cap, phase = min(cap, 10.), 'heading_alignment'
    elif apply_heading_cap and not detouring and heading_error > 15.:
        cap, phase = min(cap, _turn_speed(heading_error)), 'align'
    if recovering:
        cap, phase = min(cap, 10.), 'recovery'
    allowed = [cruise] if phase == 'cruise' else sorted({s for s in speeds if s <= cap} | {cap})
    if not allowed:
        allowed = [min(speeds)]
    result = dict(phase=phase, speeds_knots=allowed, entry_speed_knots=entry,
                speed_cap_knots=max(allowed), next_waypoint_index=next_index,
                next_waypoint_xy_m=list(points[next_index]) if next_index is not None else None,
                next_leg_bearing_deg=next_bearing, next_turn_deg=turn,
                observed_next_turn_deg=observed_turn,
                approach_distance_m=approach_distance,
                below_minimum_speed_limit=limit < min(speeds))
    return result


def detour_braking_safe(observation, geometry, speed_knots, course_deg):
    """Hold for two control cycles, then brake; enforce every waypoint circle.

    Offline diagnostic only: the runtime does not reject commands using the
    planned waypoint speed. Collision checks retain their own braking tails.
    """
    from tools.navigation_motion import predict_ground_motion
    from tools.vessel_dynamics import motion_parameters
    from tools.waypoint_geometry import circle_interval
    boat=observation['boat'];execution=observation['execution']
    parameters=motion_parameters(execution)
    parameters['deceleration_mps2']=min(parameters.get('deceleration_mps2',1.75),
        execution.get('planning_deceleration_mps2',1.75))
    modes={parameters.get('acceleration_response','bounded'),'bounded'}
    for mode in modes:
        parameters['acceleration_response']=mode
        motion=predict_ground_motion(boat['targetSpeed']*KNOT_MPS,boat['targetCourse'],
            speed_knots*KNOT_MPS,course_deg,0.,0.,2*observation['planning']['action_duration_s'],
            dt=.1,**parameters)
        path=list(motion['path']);speeds=list(motion['path_speeds'])
        deceleration=min([parameters['deceleration_mps2']]+[p[1] for p in execution.get('deceleration_curve',())])
        if motion['end_speed']>0:
            tail_parameters=dict(parameters,initial_turn_rate_deg_s=motion['end_turn_rate_deg_s'])
            tail=predict_ground_motion(motion['end_speed'],motion['end_course_deg'],0.,motion['end_course_deg'],
                0.,0.,motion['end_speed']/deceleration,dt=.1,**tail_parameters)
            end=path[-1]
            path.extend((end[0]+p[0],end[1]+p[1]) for p in tail['path'][1:])
            speeds.extend(tail['path_speeds'][1:])
        for center,point in zip(geometry.waypoints,observation['task']['planWaypointList']):
            limit=point['routeSpeed']
            for i,(a,b) in enumerate(zip(path,path[1:])):
                interval=circle_interval(a,b,center,observation['task']['arrival_radius_m'])
                if interval is not None and max(speeds[i]+(speeds[i+1]-speeds[i])*u for u in interval)>limit+1e-7:
                    return False
    return True
