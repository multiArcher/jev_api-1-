"""Local Julia backend: fixed 10/20-knot speeds, eight courses, one 16-option question."""
import threading
import logging
import os
import time
from pathlib import Path

from choice_control import DIRECTIONS, _candidate_facts
from tools.waypoint_geometry import KNOT_MPS, WaypointGeometry
from .waypoint import WaypointPolicy


JULIA_SPEEDS_KNOTS = (10.0, 20.0)
JULIA_ACTION_CHOICES = {
    f"speed_{speed:g}knots__{name}": (i, j)
    for i, speed in enumerate(JULIA_SPEEDS_KNOTS)
    for j, name in enumerate(DIRECTIONS)
}


class JuliaPolicy(WaypointPolicy):
    def __init__(self, model_path, device="cpu", engine=None, **kwargs):
        super().__init__(**kwargs)
        self.model_path = str(Path(model_path).resolve())
        self.device = device
        self._lock = threading.Lock()
        if engine is None:
            if not Path(self.model_path, "model.safetensors").is_file():
                raise RuntimeError("Julia weights missing; run server/local_models/setup_julia.py first")
            # PyTorch 2.14 otherwise JIT-compiles optional native CUDA ops,
            # requiring Python development headers on deployment hosts.
            os.environ.setdefault("TORCH_DISABLE_NATIVE_JIT", "1")
            from julia import load_model
            engine = load_model(self.model_path, device=device, strict_encoding=True,
                                max_length=8192, head_length=2048)
        self.engine = engine

    def observation(self, packet):
        obs = super().observation(packet)
        obs["action_space"]["speeds_knots"] = list(JULIA_SPEEDS_KNOTS)
        return obs

    def _request(self, obs):
        geometry = WaypointGeometry(obs)
        task = obs["task"]
        index = task["next_waypoint_index"]
        import math
        distance = math.hypot(*geometry.target)
        limit = task["planWaypointList"][index]["routeSpeed"]
        previous = index - 1
        if previous < 0 or math.hypot(*geometry.waypoints[previous]) > task["arrival_radius_m"]:
            previous = None
        state = (
            "Civilian boat navigation. All coordinates are fixed east/north, course clockwise from north. "
            "Choose a speed/course command to reach mandatory waypoints in order. "
            "Speed limits are ceilings, not desired speeds. Forecasts are approximate. "
            "Avoid speed violations, make efficient progress, avoid unnecessary stops. "
            "Available speeds are 10 or 20 knots. This experiment tests waypoint arrival without final parking. "
            f"Current speed {obs['boat']['targetSpeed'] * KNOT_MPS:.2f} m/s, "
            f"course {obs['boat']['targetCourse']} degrees. "
            f"Target east {geometry.target[0]:.2f} m, north {geometry.target[1]:.2f} m, "
            f"distance {distance:.2f} m, radius {task['arrival_radius_m']:.2f} m, "
            f"speed ceiling {limit:.2f} m/s. "
            f"Final waypoint: {index == len(geometry.waypoints)-1}. "
            f"Inside circle: {distance <= task['arrival_radius_m']}. "
            f"Command duration {self.duration}s. Physical limits: 50 knots, acceleration 1.46 m/s^2, "
            "braking 1.75 m/s^2, yaw 12 deg/s, yaw acceleration 4 deg/s^2. "
            "Brake early; a heading target is not reached instantly. "
            "Options give command speed/course, predicted distance after one/two steps, "
            "and speed-limit violation over either forecast or the previous active circle. "
            "Prefer no violation; then smaller target distance."
        )
        options = {}
        for name, (i, j) in JULIA_ACTION_CHOICES.items():
            candidate = _candidate_facts(obs, geometry,
                obs["action_space"]["speeds_knots"][i] * KNOT_MPS,
                obs["action_space"]["courses_deg"][j], DIRECTIONS[j], distance, limit, previous)
            command, effect = candidate["command"], candidate["approximate_effect"]
            violation = any((effect['passing_speed_above_ceiling'],
                effect['approach_margin_speed_above_ceiling'],
                effect['same_command_preview']['passing_speed_above_ceiling'],
                effect.get('previous_circle_speed_above_ceiling', False)))
            # Keep each description below Julia's native per-option token budget (48).
            options[name] = (f"Speed {command['speed_mps']:.1f} m/s, course {command['absolute_course_deg']:.0f} deg; "
                f"distance {effect['end_checkpoint_distance_m']:.1f} then "
                f"{effect['same_command_preview']['end_checkpoint_distance_m']:.1f} m; "
                f"speed violation {'yes' if violation else 'no'}.")
        return state, options

    def __call__(self, packet):
        obs = self.observation(packet)
        if obs['task']['route_complete']:
            return 0., obs['boat']['targetCourse'] or 0.
        state, options = self._request(obs)
        started = time.perf_counter()
        instructions = "Which command best reaches the current waypoint and keeps close to the tie line?"
        with self._lock:
            result = self.engine.predict(state=state, questions={"action": dict(
                type="choice", instructions=instructions, criteria=options)})
        answers = result.get("answers") if isinstance(result, dict) else None
        answer = answers.get("action") if isinstance(answers, dict) else None
        if not isinstance(answer, dict) or answer.get("type") != "choice":
            raise ValueError("Expected Julia Choice answer for action")
        choice = answer.get("choice")
        if not isinstance(choice, str) or choice not in JULIA_ACTION_CHOICES:
            raise ValueError("Julia returned a choice outside the 16 candidates")
        i, j = JULIA_ACTION_CHOICES[choice]
        action = dict(speed=obs["action_space"]["speeds_knots"][i],
                      course=obs["action_space"]["courses_deg"][j])
        self.local.last_decision = dict(choice=choice, candidate_count=len(options),
            selection_mode="single_choice", speeds_knots=list(JULIA_SPEEDS_KNOTS),
            inference_s=time.perf_counter() - started, state=state, options=options,
            instructions=instructions, result=result)
        logging.getLogger(__name__).debug("Julia inference_s=%.6f choice=%s",
            self.local.last_decision["inference_s"], choice)
        speed = action['speed'] * (KNOT_MPS if self.speed_unit == 'mps' else 1.0)
        return speed, action['course']
