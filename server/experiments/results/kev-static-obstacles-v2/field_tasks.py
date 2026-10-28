"""Repeatable closed-loop Kev evaluations for the two supplied field tasks."""
import argparse
import copy
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from server.decision_service import (
    ALGORITHM_TASK_ID, ControlSession, build_json_packet, build_situation_packet,
    decode_situation_packet, decode_speed_course_packet, service,
)
from server.policies.kev_policy import KEV_SPEEDS_KNOTS, KevPolicy
from server.policies.progress import NavigationStalled
from server.policies.turning import DEFAULT_MAX_TURN_RATE_DEG_S
from server.policies.waypoint_radius import DEFAULT_ARRIVAL_RADIUS_M
from server.policies.static_obstacles import StaticObstacleConfig, StaticObstacleBlocked, segment_distance
from tools.navigation_motion import predict_ground_motion
from tools.waypoint_geometry import EARTH_RADIUS, KNOT_MPS, circle_interval, xy

TASK_DIR = Path(__file__).with_name('tasks')


@dataclass(frozen=True)
class TestConfig:
    steps: int = 5000
    dt: float = 1.0
    start_offset_m: float = 50.0
    initial_course_deg: float = 90.0
    initial_position_xy_m: tuple | None = None
    max_turn_rate_deg_s: float = DEFAULT_MAX_TURN_RATE_DEG_S
    kev_endpoint: str = 'http://127.0.0.1:18028/v1/systemone'
    kev_timeout: float = 120.0

    def __post_init__(self):
        if type(self.steps) is not int or self.steps < 1:
            raise ValueError('steps must be a positive integer')
        for name in ('dt', 'start_offset_m', 'max_turn_rate_deg_s', 'kev_timeout'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if not math.isfinite(self.initial_course_deg) or not 0 <= self.initial_course_deg < 360:
            raise ValueError('initial_course_deg must be in [0, 360)')
        if self.initial_position_xy_m is not None and (
                len(self.initial_position_xy_m) != 2 or
                not all(math.isfinite(v) for v in self.initial_position_xy_m)):
            raise ValueError('initial_position_xy_m must contain two finite coordinates')


def load_task(name):
    """Return an independent copy of the original task; all speeds are knots."""
    return json.loads((TASK_DIR / f'{name}.json').read_text(encoding='utf-8'))


def _telemetry(origin, x, y, speed, course, platform_id):
    longitude = origin[0] + math.degrees(x / (EARTH_RADIUS * math.cos(math.radians(origin[1]))))
    latitude = origin[1] + math.degrees(y / EARTH_RADIUS)
    return decode_situation_packet(build_situation_packet(dict(
        platformId=platform_id, longitude=longitude, latitude=latitude,
        targetSpeed=speed / KNOT_MPS, targetCourse=course,
        direction=0, sign=0, height=0, type=0, batch=0)))


def _circle_peak(motion, center):
    peak = None
    for i, (a, b) in enumerate(zip(motion['path'], motion['path'][1:])):
        interval = circle_interval(a, b, center, DEFAULT_ARRIVAL_RADIUS_M)
        if interval is not None:
            va, vb = motion['path_speeds'][i:i + 2]
            value = max(va + (vb - va) * u for u in interval)
            peak = value if peak is None else max(peak, value)
    return peak


def run_task(name, output, config=None, policy=None, *, plots=True, static_obstacles=(), detection_range_m=5000.):
    """Run a task; route completion and constraint compliance are separate results.

    Inference pauses simulated time. This evaluates the policy and idealized
    motion, not live UDP scheduling or identified vessel dynamics.
    """
    config = config or TestConfig()
    task = load_task(name)
    session = ControlSession(task, ('127.0.0.1', 0), speed_unit='knots')
    waypoints = task['params']['planWaypointList']
    first = waypoints[0]
    origin = (first['routeLon'], first['routeLat'])
    points = [xy((p['routeLon'], p['routeLat']), origin) for p in waypoints]
    obstacles = [dict(r) for r in static_obstacles if r['platformId'] != task['params']['platCode']]
    centers = [xy((r['longitude'], r['latitude']), origin) for r in obstacles]
    if policy is None:
        policy = KevPolicy(endpoint=config.kev_endpoint, timeout=config.kev_timeout,
                           duration=config.dt, max_turn_rate_deg_s=config.max_turn_rate_deg_s,
                           speed_unit='knots', initial_course_deg=config.initial_course_deg)
    output = Path(output) / name
    output.mkdir(parents=True, exist_ok=True)
    (output / 'task.json').write_text(json.dumps(task, ensure_ascii=False, indent=2) + '\n')
    (output / 'static-obstacles.json').write_text(json.dumps(obstacles, ensure_ascii=False, indent=2) + '\n')
    collision_radius = (policy.obstacle_config if isinstance(policy, KevPolicy) else StaticObstacleConfig()).collision_radius_m
    minimum_clearance = float('inf')
    x, y = config.initial_position_xy_m or (-config.start_offset_m, 0.)
    speed, course = 0., config.initial_course_deg
    rows, arrivals, violations = [], [], []
    completed = False
    reason = 'step_limit'
    error = None
    failed = False
    max_speed_knots = max_actual_turn_rate = 0.
    started = time.perf_counter()
    with (output / 'trajectory.jsonl').open('w') as log:
        for step in range(config.steps):
            state = _telemetry(origin, x, y, speed, course, task['params']['platCode'])
            previous_route = session.current_route
            session.update_situation(state)
            for index in range(previous_route, session.current_route):
                arrivals.append(dict(index=index, time_s=step * config.dt,
                                     speed_knots=state['targetSpeed']))
            target_index = min(session.current_route, len(points) - 1)
            distance = math.dist((x, y), points[target_index])
            inference_started = time.perf_counter()
            try:
                if session.route_complete or failed:
                    command = dict(speed=0., course=course)
                    decision = None
                else:
                    request = copy.deepcopy(task)
                    request['params']['next_waypoint_index'] = session.current_route
                    detections = tuple(r for r,center in zip(obstacles, centers)
                                       if math.dist((x,y),center) <= detection_range_m)
                    command = decode_speed_course_packet(service(
                        build_json_packet(request, ALGORITHM_TASK_ID), policy, (state,)+detections))
                    decision = getattr(getattr(policy, 'local', None), 'last_decision', None)
                    decision = copy.deepcopy(decision) if isinstance(decision, dict) else None
            except (ValueError, RuntimeError, OSError, KeyError, TypeError) as exc:
                reason = ('navigation_no_progress' if isinstance(exc, NavigationStalled) else
                          'static_obstacle_blocked' if isinstance(exc, StaticObstacleBlocked) else 'model_error')
                error, failed = str(exc), True
                command, decision = dict(speed=0., course=course), None
            latency = time.perf_counter() - inference_started
            turn = abs((command['course'] - course + 180) % 360 - 180)
            invalid = (not 0 <= command['speed'] <= 50. or not 0 <= command['course'] < 360
                       or (command['speed'] > 0 and turn > min(config.max_turn_rate_deg_s * config.dt, 179.99) + .011))
            if invalid:
                reason = 'invalid_command'
                violations.append(dict(kind='command', step=step, command=command))
            row = dict(step=step, time_s=step * config.dt, x=x, y=y,
                       actual_speed_knots=speed / KNOT_MPS, actual_course_deg=course,
                       completed_waypoints=session.current_route, target_index=target_index,
                       distance_m=distance, command=command, inference_s=latency, decision=decision,
                       error=error)
            rows.append(row)
            log.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
            log.flush()
            if invalid:
                break
            if (session.route_complete or failed) and state['targetSpeed'] <= .1:
                if not failed:
                    completed, reason = True, 'completed'
                break
            motion = predict_ground_motion(speed, course, command['speed'] * KNOT_MPS,
                                           command['course'], .5, .5, config.dt,
                                           max_turn_rate_deg_s=config.max_turn_rate_deg_s)
            for obstacle,center in zip(obstacles,centers):
                relative=(center[0]-x,center[1]-y)
                clearance=min(segment_distance(relative,a,b) for a,b in zip(motion['path'],motion['path'][1:]))-collision_radius
                minimum_clearance=min(minimum_clearance,clearance)
                if clearance <= 0:
                    violations.append(dict(kind='static_collision',step=step,obstacle_id=obstacle['platformId'],clearance_m=clearance))
            # Check the active circle and all already visited circles still occupied.
            for index in range(target_index + 1):
                center = (points[index][0] - x, points[index][1] - y)
                if index < target_index and math.hypot(*center) > DEFAULT_ARRIVAL_RADIUS_M:
                    continue
                peak = _circle_peak(motion, center)
                if peak is not None and peak / KNOT_MPS > waypoints[index]['routeSpeed'] + .01:
                    violations.append(dict(kind='waypoint_speed', step=step, index=index,
                                           peak_speed_knots=peak / KNOT_MPS,
                                           limit_knots=waypoints[index]['routeSpeed']))
            max_speed_knots = max(max_speed_knots, max(motion['path_speeds']) / KNOT_MPS)
            next_course = motion['end_course_deg']
            max_actual_turn_rate = max(max_actual_turn_rate,
                                      abs((next_course - course + 180) % 360 - 180) / config.dt)
            x += motion['endpoint'][0]
            y += motion['endpoint'][1]
            speed, course = motion['end_speed'], next_course
            if (step + 1) % 100 == 0:
                print(f'{name}: {step + 1} steps, {session.current_route}/{len(points)} waypoints, distance={distance:.1f}m', flush=True)
    summary = dict(task=name, task_id=task['id'], passed=completed and not violations,
                   completed=completed, reason=reason, error=error, steps=len(rows),
                   reached_waypoints=session.current_route, total_waypoints=len(points),
                   simulated_seconds=rows[-1]['time_s'] if rows else 0.,
                   final_speed_knots=rows[-1]['actual_speed_knots'] if rows else 0.,
                   max_speed_knots=max_speed_knots, max_actual_turn_rate_deg_s=max_actual_turn_rate,
                   available_speeds_knots=list(KEV_SPEEDS_KNOTS),
                   recovery_decisions=sum(bool(r['decision'] and r['decision']['state']['progress']['mode'] == 'recovery') for r in rows),
                   stop_confirmed=bool(rows and rows[-1]['command']['speed'] == 0. and rows[-1]['actual_speed_knots'] <= .1),
                   arrival_radius_m=DEFAULT_ARRIVAL_RADIUS_M, speed_unit='knots',
                   waypoint_speed_limits_knots=[p['routeSpeed'] for p in waypoints],
                   speed_violation_count=sum(v['kind'] == 'waypoint_speed' for v in violations),
                   static_obstacle_count=len(obstacles), static_collision_radius_m=collision_radius,
                   static_collision_count=sum(v['kind'] == 'static_collision' for v in violations),
                   minimum_static_clearance_m=minimum_clearance if math.isfinite(minimum_clearance) else None,
                   violations=violations, arrivals=arrivals, config=asdict(config),
                   elapsed_s=time.perf_counter() - started,
                   limitations='Idealized motion; simulation pauses during inference. No live UDP timing or measured hull dynamics.')
    (output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    if plots and rows:
        _plot(rows, points, waypoints, output, centers, collision_radius)
    return summary


def test_task_one(output, config=None, policy=None, *, plots=True, static_obstacles=()):
    """Basic 12-waypoint field route."""
    return run_task('task_one', output, config, policy, plots=plots, static_obstacles=static_obstacles)


def test_task_two(output, config=None, policy=None, *, plots=True, static_obstacles=()):
    """45-waypoint field route with more demanding direction changes."""
    return run_task('task_two', output, config, policy, plots=plots, static_obstacles=static_obstacles)


def _plot(rows, points, waypoints, output, obstacles=(), collision_radius=0.):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    axes[0].plot(*zip(*points), '--', color='gray', label='Waypoint order')
    axes[0].plot([r['x'] for r in rows], [r['y'] for r in rows], label='Trajectory')
    axes[0].scatter(rows[0]['x'], rows[0]['y'], marker='s', label='Start')
    for center in obstacles:
        axes[0].add_patch(Circle(center, collision_radius, color='red', alpha=.2))
    for index, point in enumerate(points):
        axes[0].add_patch(Circle(point, DEFAULT_ARRIVAL_RADIUS_M, fill=False, color='gray', alpha=.5))
        axes[0].annotate(str(index + 1), point, fontsize=7)
    axes[0].set(xlabel='East (m)', ylabel='North (m)', aspect='equal')
    times = [r['time_s'] for r in rows]
    axes[1].plot(times, [r['actual_speed_knots'] for r in rows], label='Actual')
    axes[1].step(times, [r['command']['speed'] for r in rows], label='Command', alpha=.6)
    axes[1].step(times, [waypoints[r['target_index']]['routeSpeed'] for r in rows], label='Waypoint limit', linestyle='--')
    axes[1].set(xlabel='Time (s)', ylabel='Speed (knots)')
    axes[2].plot(times, [r['actual_course_deg'] for r in rows], label='Actual')
    axes[2].step(times, [r['command']['course'] for r in rows], label='Command', alpha=.6)
    axes[2].set(xlabel='Time (s)', ylabel='Course (deg)')
    for ax in axes:
        ax.legend()
        ax.grid(alpha=.2)
    fig.tight_layout()
    fig.savefig(output / 'trajectory.png', dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', choices=('one', 'two', 'both'), default='both')
    parser.add_argument('--output', type=Path, default=ROOT / 'server/experiments/results/field-tasks')
    parser.add_argument('--steps', type=int, default=5000)
    parser.add_argument('--dt', type=float, default=1.)
    parser.add_argument('--initial-course-deg', type=float, default=90.)
    parser.add_argument('--start-offset-m', type=float, default=50.)
    parser.add_argument('--max-turn-rate-deg-s', type=float, default=DEFAULT_MAX_TURN_RATE_DEG_S)
    parser.add_argument('--kev-endpoint', default='http://127.0.0.1:18028/v1/systemone')
    parser.add_argument('--kev-timeout', type=float, default=120.)
    parser.add_argument('--no-plots', action='store_true')
    parser.add_argument('--static-situation', type=Path, help='JSON array of recorded stationary target records')
    args = parser.parse_args()
    try:
        config = TestConfig(**{name: getattr(args, name) for name in TestConfig.__dataclass_fields__ if hasattr(args, name)})
    except ValueError as exc:
        parser.error(str(exc))
    functions = {'one': test_task_one, 'two': test_task_two}
    selected = functions.values() if args.task == 'both' else [functions[args.task]]
    obstacles = json.loads(args.static_situation.read_text()) if args.static_situation else ()
    results = [function(args.output, config, plots=not args.no_plots, static_obstacles=obstacles) for function in selected]
    for result in results:
        print(json.dumps({key: result[key] for key in ('task', 'passed', 'completed', 'reason', 'reached_waypoints', 'total_waypoints', 'speed_violation_count')}, ensure_ascii=False))
    return 0 if all(result['passed'] for result in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
