"""Shared external telemetry validation and waypoint progress for policy backends."""
import copy
import math
import threading

from tools.waypoint_geometry import KNOT_MPS, WaypointGeometry
from tools.vessel_dynamics import limits_state, positive_limit, MAX_SPEED_KNOTS, validate_curve
from tools.acceleration_response import DEFAULT_ACCELERATION_RESPONSE, validate_response_mode
from .waypoint_radius import waypoint_arrival_radius


class WaypointPolicy:
    def __init__(self, speed_unit="knots", max_speed_knots=50.0, duration=1.0,
                 speed_transition=0.5, course_transition=0.5, motion_curves=None, acceleration_response=DEFAULT_ACCELERATION_RESPONSE):
        if speed_unit not in ("knots", "mps"):
            raise ValueError("speed_unit must be knots or mps")
        if not all(math.isfinite(v) and v > 0 for v in (max_speed_knots, duration)):
            raise ValueError("max speed and duration must be positive and finite")
        if not all(math.isfinite(v) and v >= 0 for v in (speed_transition, course_transition)):
            raise ValueError("transition times must be nonnegative and finite")
        self.speed_unit = speed_unit
        positive_limit('max_speed_knots', max_speed_knots, MAX_SPEED_KNOTS)
        self.max_speed, self.duration = max_speed_knots, duration
        self.execution = dict(speed_transition_time_s=speed_transition,
                              course_transition_time_s=course_transition, **limits_state())
        self.execution['acceleration_response'] = validate_response_mode(acceleration_response)
        curves = motion_curves or {}
        for key, maximum in (('acceleration_curve', 1.46), ('deceleration_curve', 1.75), ('turn_rate_curve', 12.)):
            self.execution[key] = validate_curve(curves.get(key, ()), maximum)
        if set(curves)-{'acceleration_curve', 'deceleration_curve', 'turn_rate_curve'}:
            raise ValueError('unknown motion curve key')
        self.local = threading.local()

    def _observation_course(self, boat):
        """Keep valid reported heading at rest so startup also obeys yaw limits."""
        course = boat.get('targetCourse')
        if course is None and boat['targetSpeed'] < 1e-6:
            return None
        if isinstance(course,bool) or not isinstance(course,(int,float)) or not math.isfinite(course) or not 0 <= course < 360:
            raise ValueError('Missing or invalid situation targetCourse')
        return course

    def observation(self, packet):
        if packet.get("code") != "trackingMode":
            raise ValueError("Only trackingMode waypoint tasks are supported")
        params = packet["params"]
        boat = next((s for s in packet.get("situation", ())
                     if s["platformId"] == params["platCode"]), None)
        if boat is None:
            raise ValueError("No situation for task params.platCode")
        speed = boat["targetSpeed"]
        if speed is None or not math.isfinite(speed) or speed < 0:
            raise ValueError("Missing or invalid actual speed")
        waypoints = copy.deepcopy(params["planWaypointList"])
        if not waypoints or type(params.get("planWaypointNum")) is not int or params["planWaypointNum"] != len(waypoints):
            raise ValueError("Expected at least one ordered waypoint and a matching planWaypointNum")
        for point in waypoints:
            if not all(math.isfinite(point[k]) for k in ("routeLon", "routeLat", "routeSpeed")):
                raise ValueError("Invalid waypoint")
            if not -180 <= point["routeLon"] <= 180 or not -90 <= point["routeLat"] <= 90 or point["routeSpeed"] < 0:
                raise ValueError("Invalid waypoint coordinates or speed")
            if self.speed_unit == "knots":
                point["routeSpeed"] *= KNOT_MPS
        key = (packet.get("simId"), packet.get("id"),
               {k: v for k, v in params.items() if k != "next_waypoint_index"})
        requested_index = params.get("next_waypoint_index", params.get("currentRoute", 0))
        if type(requested_index) is not int or not 0 <= requested_index <= len(waypoints):
            raise ValueError("currentRoute must be a completed waypoint count in [0, planWaypointNum]")
        if getattr(self.local, "task_key", None) != key:
            self.local.task_key = copy.deepcopy(key)
            self.local.next_index = min(requested_index, len(waypoints) - 1)
            self.local.route_complete = requested_index == len(waypoints)
        elif requested_index > self.local.next_index:
            self.local.next_index = min(requested_index, len(waypoints) - 1)
            self.local.route_complete = requested_index == len(waypoints)
        index = self.local.next_index
        if type(index) is not int or not 0 <= index < len(waypoints):
            raise ValueError("Invalid current route / next waypoint index")
        execution = dict(self.execution, initial_turn_rate_deg_s=boat.get('_turn_rate_deg_s', 0.))
        obs = dict(
            boat=dict(longitude=boat["longitude"], latitude=boat["latitude"],
                      targetSpeed=speed if self.speed_unit == "knots" else speed / KNOT_MPS,
                      targetCourse=self._observation_course(boat),
                      _turn_rate_deg_s=boat.get('_turn_rate_deg_s', 0.)),
            planning=dict(action_duration_s=self.duration), execution=execution,
            action_space=dict(speeds_knots=[self.max_speed * f for f in (0, .05, .1, .2, .35, .5, .75, 1)],
                              courses_deg=list(range(0, 360, 45))),
            task=dict(planWaypointList=waypoints, planWaypointNum=len(waypoints),
                      next_waypoint_index=index, routeSpeed_unit="m/s", stop_speed_mps=0.1,
                      arrival_radius_m=waypoint_arrival_radius(waypoints[index])))
        if not all(math.isfinite(obs["boat"][k]) for k in ("longitude", "latitude")) or not -180 <= boat["longitude"] <= 180 or not -90 <= boat["latitude"] <= 90:
            raise ValueError("Invalid boat coordinates")
        course = obs["boat"]["targetCourse"]
        if speed >= 1e-6 and (course is None or not math.isfinite(course) or not 0 <= course < 360):
            raise ValueError("Missing or invalid movement course")
        while not self.local.route_complete:
            geometry = WaypointGeometry(obs)
            if math.hypot(*geometry.target) > obs["task"]["arrival_radius_m"]:
                break
            if self.local.next_index == len(waypoints) - 1:
                self.local.route_complete = True
                break
            self.local.next_index += 1
            obs["task"]["next_waypoint_index"] = self.local.next_index
            obs["task"]["arrival_radius_m"] = waypoint_arrival_radius(waypoints[self.local.next_index])
        self.local.completed_count = len(waypoints) if self.local.route_complete else self.local.next_index
        obs["task"]["route_complete"] = self.local.route_complete
        return obs

    def current_route(self):
        """Return the number of completed mandatory waypoints."""
        return getattr(self.local, "completed_count", 0)
