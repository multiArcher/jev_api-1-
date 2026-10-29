"""Remote Jev backend; shares observations with local backends."""
import os
import math
from choice_control import build_payload, decode_action
from tools.http_transport import JevTransport
from tools.waypoint_geometry import KNOT_MPS
from tools.acceleration_response import DEFAULT_ACCELERATION_RESPONSE
if __package__:
    from .policies.waypoint import WaypointPolicy
else:
    from policies.waypoint import WaypointPolicy


class JevPolicy(WaypointPolicy):
    def __init__(self, model="jev-latest", endpoint="https://api.typesafe.ai/v1/systemone",
                 speed_unit="knots", max_speed_knots=50.0, duration=1.0,
                 speed_transition=0.5, course_transition=0.5, api_timeout=10.0, motion_curves=None, acceleration_response=DEFAULT_ACCELERATION_RESPONSE):
        super().__init__(speed_unit, max_speed_knots, duration, speed_transition, course_transition, motion_curves, acceleration_response)
        if not math.isfinite(api_timeout) or api_timeout <= 0:
            raise ValueError("api_timeout must be positive and finite")
        self.model, self.endpoint, self.api_timeout = model, endpoint, api_timeout

    def __call__(self, packet):
        obs = self.observation(packet)
        if obs['task']['route_complete']:
            return 0., obs['boat']['targetCourse'] or 0.
        if not hasattr(self.local, "transport"):
            self.local.transport = JevTransport(timeout=self.api_timeout)
        # Environment files and copy/paste can accidentally leave a trailing
        # newline.  HTTP libraries reject newlines in Authorization headers.
        key = os.environ.get("JEV_API_KEY", "").strip()
        if not key:
            raise RuntimeError("JEV_API_KEY is not configured")
        payload = build_payload(obs, self.model)
        payload["state"]["task"]["background"] = payload["state"]["task"]["background"].replace(
            "No obstacles, wind or sideways drift are simulated. The world pauses while you decide.",
            "Live external telemetry: the world keeps moving during API calls. Obstacles, wind and drift are not observed or modeled.")
        response = self.local.transport.call(payload, key, self.endpoint)
        action = decode_action(response, obs)
        speed = action["speed"]
        if self.speed_unit == "mps":
            speed *= KNOT_MPS
        return speed, action["course"]



jev_policy = JevPolicy()
