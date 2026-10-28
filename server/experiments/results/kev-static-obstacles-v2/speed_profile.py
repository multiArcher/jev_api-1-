"""Waypoint lookahead and speed stages; course choice remains with Kev."""
import math

from tools.waypoint_geometry import KNOT_MPS


def _bearing(vector):
    return math.degrees(math.atan2(vector[0], vector[1])) % 360


def _turn_speed(angle):
    for threshold, speed in ((15., 50.), (45., 40.), (90., 30.), (135., 20.)):
        if angle <= threshold:
            return speed
    return 10.


def plan_speed(observation, geometry, speeds, turn_rate_deg_s, recovering=False):
    """Plan cruise/approach speeds from the next leg and existing motion assumptions.

    Brake lead includes the configured linear speed transition, two command
    periods and the arc length of the next turn at its planned entry speed.
    These are simulation assumptions, not calibrated vessel performance.
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
    transition = observation['execution']['speed_transition_time_s']
    brake = (planning_speed + entry_mps) * .5 * transition
    reaction = planning_speed * 2 * observation['planning']['action_duration_s']
    turn_arc = entry_mps * math.radians(turn) / math.radians(turn_rate_deg_s)
    approach_distance = task['arrival_radius_m'] + brake + reaction + turn_arc
    distance = math.hypot(*geometry.target)
    heading_error = abs((_bearing(geometry.target) - boat['targetCourse'] + 180) % 360 - 180)

    cap, phase = cruise, 'cruise'
    if distance <= approach_distance and entry < cap:
        cap, phase = entry, 'approach'
    if heading_error > 15.:
        cap, phase = min(cap, _turn_speed(heading_error)), 'align'
    # All visited circles still occupied retain their speed limits.
    for i in range(index):
        if math.hypot(*geometry.waypoints[i]) <= task['arrival_radius_m']:
            previous_limit = task['planWaypointList'][i]['routeSpeed'] / KNOT_MPS
            if previous_limit < cap:
                cap, phase = previous_limit, 'exit_circle'
    if recovering:
        cap, phase = min(cap, 10.), 'recovery'
    allowed = [cruise] if phase == 'cruise' else [s for s in speeds if s <= cap]
    if not allowed:
        allowed = [min(speeds)]
    return dict(phase=phase, speeds_knots=allowed, entry_speed_knots=entry,
                speed_cap_knots=max(allowed), next_waypoint_index=next_index,
                next_waypoint_xy_m=list(points[next_index]) if next_index is not None else None,
                next_leg_bearing_deg=next_bearing, next_turn_deg=turn,
                observed_next_turn_deg=observed_turn,
                approach_distance_m=approach_distance,
                below_minimum_speed_limit=limit < min(speeds))
