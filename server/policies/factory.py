"""Construct backends without importing local-model dependencies for Jev users."""
from .turning import DEFAULT_MAX_TURN_RATE_DEG_S


DEFAULT_BACKEND = "kev"


def create_policy(backend=DEFAULT_BACKEND, *, model_path="models/Julia-1", device="cpu", kev_endpoint=None, kev_timeout=120., kev_max_turn_rate_deg_s=DEFAULT_MAX_TURN_RATE_DEG_S, kev_initial_course_deg=None, kev_obstacle_config=None, kev_dynamic_config=None, kev_route_rejoin_config=None, **kwargs):
    if backend == "kev":
        for remote_setting in ("model", "endpoint", "api_timeout"):
            kwargs.pop(remote_setting, None)
        from .kev_policy import KevPolicy, DEFAULT_KEV_ENDPOINT
        return KevPolicy(endpoint=kev_endpoint or DEFAULT_KEV_ENDPOINT, timeout=kev_timeout, max_turn_rate_deg_s=kev_max_turn_rate_deg_s,
                         initial_course_deg=kev_initial_course_deg, obstacle_config=kev_obstacle_config,
                         dynamic_config=kev_dynamic_config, route_rejoin_config=kev_route_rejoin_config, **kwargs)
    if backend == "julia":
        for remote_setting in ("model", "endpoint", "api_timeout"):
            kwargs.pop(remote_setting, None)
        from .julia_policy import JuliaPolicy
        return JuliaPolicy(model_path, device=device, **kwargs)
    if backend == "jev":
        if __package__.startswith('server.'):
            from ..jev_policy import JevPolicy
        else:
            from jev_policy import JevPolicy
        return JevPolicy(**kwargs)
    raise ValueError(f"Unknown policy backend: {backend}")
