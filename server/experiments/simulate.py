"""Civilian waypoint closed loop: real local model, binary packets, optional localhost UDP."""
import argparse
import copy
import json
import math
import os
import re
from pathlib import Path
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from server.decision_service import (
    ALGORITHM_TASK_ID, ALGORITHM_STOP_ID,
    build_json_packet, build_situation_packet, decode_situation_packet,
    decode_speed_course_packet, analyze_task, service,
)
from server.experiments.scenarios import SCENARIOS, ROUTES
from server.policies.factory import create_policy
from server.policies.turning import DEFAULT_MAX_TURN_RATE_DEG_S
from server.policies.waypoint_radius import DEFAULT_ARRIVAL_RADIUS_M
from tools.navigation_motion import predict_ground_motion
from tools.vessel_dynamics import motion_parameters
from tools.waypoint_geometry import KNOT_MPS

EARTH = 6378137.0
ORIGIN = (120.5, 30.2)



def lonlat(x, y):
    return (ORIGIN[0] + math.degrees(x / (EARTH * math.cos(math.radians(ORIGIN[1])))),
            ORIGIN[1] + math.degrees(y / EARTH))


def scenario(name):
    waypoints = []
    # The origin is supplied by telemetry, never included as an already-visited waypoint.
    for x, y in ROUTES[name][1:]:
        lon, lat = lonlat(x, y)
        waypoints.append(dict(routeLon=lon, routeLat=lat, routeSpeed=SCENARIOS[name]['speed_knots'], routeRadius=SCENARIOS[name]['radius_m']))
    return dict(simId='local-civilian', id=name, code='trackingMode', params=dict(
        platCode=1010, planRouteNum=1, currentRoute=0,
        planWaypointNum=len(waypoints), planWaypointList=waypoints))


def telemetry(x, y, speed, course):
    lon, lat = lonlat(x, y)
    # Unused legacy fields carry unknown/invalid values; never model inputs.
    return dict(platformId=1010, longitude=lon, latitude=lat,
        targetSpeed=speed / KNOT_MPS, targetCourse=course,
        direction=0, sign=0, height=0, type=0,
        targetTypeConfid=None, threatLevel=None, batch=0)


def advance(x, y, speed, course, command, dt, max_turn_rate_deg_s=None, *,
            initial_turn_rate_deg_s=0., return_turn_rate=False, motion_settings=None):
    motion = predict_ground_motion(speed, course,
        command['speed'] * KNOT_MPS, command['course'], 0., 0., dt,
        **motion_parameters(dict(motion_settings if isinstance(motion_settings,dict) else {}, initial_turn_rate_deg_s=initial_turn_rate_deg_s,
                                 **({'max_turn_rate_deg_s':max_turn_rate_deg_s} if max_turn_rate_deg_s is not None else {}))))
    dx, dy = motion['endpoint']
    result = (x+dx, y+dy, motion['end_speed'], motion['end_course_deg'])
    return (*result, motion['end_turn_rate_deg_s']) if return_turn_rate else result


def draw_map(ax, rows, name):
    from matplotlib.patches import Circle
    points = ROUTES[name]
    ax.plot(*zip(*points), '--', color='black', linewidth=1.4, label='Reference order', zorder=1)
    ax.plot([r['x'] for r in rows], [r['y'] for r in rows], color='#0078b8', linewidth=1.7, label='Actual model trajectory', zorder=3)
    ax.scatter(*points[0], color='#13965e', s=35, marker='s', label='Start', zorder=4)
    ax.scatter(rows[-1]['x'], rows[-1]['y'], color='#cf483d', marker='x', s=40, label='Last position', zorder=5)
    for index, (x, y) in enumerate(points):
        label = 'S' if index == 0 else ('F' if index == len(points)-1 else f'P{index}')
        ax.annotate(label, (x, y), xytext=(5, 9), textcoords='offset points', fontsize=9)
        if index:
            ax.add_patch(Circle((x, y), DEFAULT_ARRIVAL_RADIUS_M, fill=False, color='gray', alpha=.65))
    ax.grid(alpha=.2)


def plot_map(rows, name, output, completed=None):
    """Large reference/actual trajectory view, matching the supplied figure's style."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(11, 7))
    draw_map(ax, rows, name)
    points = ROUTES[name] + [(r['x'], r['y']) for r in rows]
    radius = DEFAULT_ARRIVAL_RADIUS_M
    xs, ys = list(zip(*points))
    margin = max(radius*2, (max(xs)-min(xs))*.12, (max(ys)-min(ys))*.12)
    ax.set(xlim=(min(xs)-margin, max(xs)+margin), ylim=(min(ys)-margin, max(ys)+margin),
           xlabel='East (m)', ylabel='North (m)', aspect='equal',
           title=f'{name}: reference and actual trajectory')
    ax.legend(loc='best', fontsize=9)
    reached = rows[-1].get('target_index', 1)-1
    if rows[-1].get('distance_m', float('inf')) <= radius:
        reached += 1
    status = ('completed' if completed else 'step limit reached') if completed is not None else 'UDP control test'
    interval = (rows[1]['t'] - rows[0]['t']) if len(rows) > 1 else None
    timing = f'{len(rows)} samples' if interval is None else f'{len(rows)} samples | interval {interval:g} s'
    fig.text(.5, .02, f"{status} | reached {reached}/{len(ROUTES[name])-1} waypoints | {timing} | simulation {rows[-1]['t']:.1f} s",
             ha='center', fontsize=10)
    fig.tight_layout(rect=(0,.04,1,1)); fig.savefig(output / f'{name}-map.png', dpi=180); plt.close(fig)


def plot(rows, name, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    draw_map(axes[0], rows, name)
    axes[0].set(xlabel='East (m)', ylabel='North (m)', title=name, aspect='equal')
    points = ROUTES[name] + [(r['x'], r['y']) for r in rows]
    xmin, xmax = min(p[0] for p in points), max(p[0] for p in points)
    ymin, ymax = min(p[1] for p in points), max(p[1] for p in points)
    half = max(xmax-xmin, ymax-ymin, 20.) * .65
    axes[0].set_xlim((xmin+xmax)/2-half, (xmin+xmax)/2+half)
    axes[0].set_ylim((ymin+ymax)/2-half, (ymin+ymax)/2+half)
    axes[0].legend(fontsize=8)
    axes[1].plot([r['t'] for r in rows], [r['speed_mps'] / KNOT_MPS for r in rows], label='Actual')
    axes[1].plot([r['t'] for r in rows], [r['command']['speed'] for r in rows], label='Command', alpha=.6)
    axes[1].axhline(SCENARIOS[name]['speed_knots'], linestyle='--', color='red', label='Waypoint ceiling')
    axes[1].set(xlabel='Simulation time (s)', ylabel='Speed (knots)'); axes[1].legend()
    axes[2].plot([r['t'] for r in rows], [r['latency_s'] for r in rows])
    axes[2].set(xlabel='Simulation time (s)', ylabel='Time since last control (s)' if rows[0].get('latency_kind') == 'age' else 'Model response time (s; not simulation interval)')
    fig.tight_layout(); fig.savefig(output / f'{name}.png', dpi=160); plt.close(fig)


def direct(policy, name, args, out):
    task = scenario(name)
    x = y = speed = yaw_rate = 0.; course = getattr(args, 'initial_course_deg', 90.)
    rows = []; completed = False; violation_steps = 0; final_arrived = False
    for step in range(args.steps):
        state = decode_situation_packet(build_situation_packet(telemetry(x, y, speed, course)))
        state['_turn_rate_deg_s'] = yaw_rate
        started = time.perf_counter()
        control = service(build_json_packet(task, ALGORITHM_TASK_ID), policy, (state,))
        elapsed = time.perf_counter() - started
        command = decode_speed_course_packet(control)
        decision = copy.deepcopy(getattr(policy.local, 'last_decision', {}))
        current = min(policy.current_route(), len(ROUTES[name]) - 2) + 1
        target = ROUTES[name][current]
        distance = math.hypot(x-target[0], y-target[1])
        if distance <= DEFAULT_ARRIVAL_RADIUS_M and speed > SCENARIOS[name]['speed_knots'] * KNOT_MPS + .01:
            violation_steps += 1
        if current == len(ROUTES[name])-1 and distance <= DEFAULT_ARRIVAL_RADIUS_M:
            final_arrived = True
        if final_arrived:
            command = dict(speed=0., course=course if course is not None else command['course'])
        rows.append(dict(t=step * args.dt, x=x, y=y, speed_mps=speed,
            actual_course_deg=course, command=command, latency_s=elapsed, target_index=current, distance_m=distance,
            decision=decision))
        if (step + 1) % 100 == 0:
            print(f'step {step+1}/{args.steps}: target={current}, distance={distance:.1f}m, course={command["course"]:.1f}, decision={elapsed:.3f}s', flush=True)
        if final_arrived and speed <= .1 * KNOT_MPS and command['speed'] == 0:
            completed = True; break
        x, y, speed, course, yaw_rate = advance(x, y, speed, course, command, args.dt,
            max_turn_rate_deg_s=args.max_turn_rate_deg_s if args.backend == 'kev' else None,
            initial_turn_rate_deg_s=yaw_rate, return_turn_rate=True, motion_settings=getattr(policy,'execution',{}))
    save(rows, name, out)
    plot_map(rows, name, out, completed)
    latencies = sorted(r['latency_s'] for r in rows)
    return dict(scenario=name, completed=completed, steps=len(rows),
        reached_waypoints=len(ROUTES[name])-1 if final_arrived else current - 1,
        total_waypoints=len(ROUTES[name])-1, target_waypoint_index=current,
        simulated_seconds=rows[-1]['t'], final_distance_m=rows[-1]['distance_m'],
        final_speed_knots=rows[-1]['speed_mps'] / KNOT_MPS, speed_violation_sample_count=violation_steps,
        mean_latency_s=sum(latencies)/len(latencies), p95_latency_s=latencies[int(.95*(len(latencies)-1))],
        max_latency_s=max(latencies))


def save(rows, name, out):
    with (out / f'{name}.jsonl').open('w') as f:
        for row in rows:
            f.write(json.dumps(row, ensure_ascii=False) + '\n')
    plot(rows, name, out)


def udp(args, out):
    """Exercise the actual service subprocess, status channel and verified stop."""
    env = dict(os.environ, HF_HUB_OFFLINE='1', TRANSFORMERS_OFFLINE='1')
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as control, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as peer:
        control.bind(('127.0.0.1', 0)); peer.bind(('127.0.0.1', 0))
        # Reserve a random local task port, then hand it to the subprocess.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
            reserve.bind(('127.0.0.1', 0)); port = reserve.getsockname()[1]
        target = ('127.0.0.1', port)
        log = (out / 'udp-service.log').open('w')
        process = subprocess.Popen([sys.executable, '-m', 'server.decision_service',
            '--kev-endpoint', args.kev_endpoint, '--kev-timeout', str(args.kev_timeout),
            '--max-turn-rate-deg-s', str(args.max_turn_rate_deg_s), '--initial-course-deg', str(args.initial_course_deg),
            '--acceleration-response',args.acceleration_response,
            '--backend', args.backend, '--device', args.device, '--local-model-path', str(args.model_path),
            '--host', '127.0.0.1', '--port', str(port), '--action-host', '127.0.0.1',
            '--action-port', str(control.getsockname()[1]), '--situation-port', '0',
            '--platform-id', '1010', '--max-speed-knots', str(args.max_speed_knots), '--control-interval', str(args.dt)],
            cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
        rows = []; statuses = []; control_events = []
        name = 'straight' if args.scenario == 'all' else args.scenario
        task = scenario(name); stop_sent = False; current_target = 1
        try:
            deadline = time.monotonic() + 120
            while 'listening on UDP' not in (out / 'udp-service.log').read_text():
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('Service startup failed; see udp-service.log')
                time.sleep(.1)
            control.setblocking(False); peer.setblocking(False)
            x = y = speed = yaw_rate = 0.; course = args.initial_course_deg
            command = dict(speed=0., course=0.)
            peer.sendto(build_situation_packet(telemetry(x, y, speed, course)), target)
            peer.sendto(build_json_packet(task, ALGORITHM_TASK_ID), target)
            start = previous = time.monotonic(); next_tick = start; last_action = start
            final = None
            while time.monotonic() - start < args.udp_seconds + 12:
                now = time.monotonic(); dt = now - previous; previous = now
                x, y, speed, course, yaw_rate = advance(x, y, speed, course, command, dt,
                    max_turn_rate_deg_s=args.max_turn_rate_deg_s if args.backend == 'kev' else None,
                    initial_turn_rate_deg_s=yaw_rate, return_turn_rate=True,
                    motion_settings=dict(acceleration_response=args.acceleration_response))
                if now >= next_tick:
                    peer.sendto(build_situation_packet(telemetry(x, y, speed, course)), target)
                    tx, ty = ROUTES[name][current_target]
                    rows.append(dict(t=now-start, x=x, y=y, speed_mps=speed,
                        target_index=current_target, distance_m=math.hypot(x-tx, y-ty),
                        command=dict(command), latency_s=now-last_action, latency_kind='age'))
                    next_tick = now + .1
                try:
                    while True:
                        data, _ = control.recvfrom(65507)
                        command = decode_speed_course_packet(data); last_action = now
                        control_events.append(dict(t=now-start, command=dict(command)))
                except BlockingIOError:
                    pass
                try:
                    while True:
                        data, _ = peer.recvfrom(65507)
                        status = analyze_task(data); statuses.append(status)
                        next_index = status.get('params', {}).get('nextWaypointIndex')
                        if type(next_index) is int and 0 <= next_index < len(ROUTES[name])-1:
                            current_target = next_index + 1
                        if status.get('status') in (2, 3):
                            final = status
                except BlockingIOError:
                    pass
                if now-start >= args.udp_seconds and not stop_sent:
                    peer.sendto(build_json_packet({k:task[k] for k in ('simId','id','code')}, ALGORITHM_STOP_ID), target)
                    stop_sent = True
                if final:
                    break
                time.sleep(.01)
            save(rows, name, out)
            plot_map(rows, name, out)
            (out / 'udp-statuses.json').write_text(json.dumps(statuses, indent=2))
            (out / 'udp-controls.json').write_text(json.dumps(control_events, indent=2))
            inference = [float(v) for v in re.findall(r'(?:Julia|Kev) inference_s=([0-9.]+)', (out / 'udp-service.log').read_text())]
            return dict(inference_seconds=inference, received_controls=len(control_events), transport='localhost UDP', task_address=target,
                action_address=control.getsockname(), status_address=peer.getsockname(),
                final_status=final, received_statuses=len(statuses),
                received_motion=any(r['command']['speed'] > 0 for r in rows),
                stopped=bool(final and final.get('status')==2 and final.get('params', {}).get('stopConfirmed')))
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
            log.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend', choices=['julia', 'jev', 'kev'], default='julia')
    p.add_argument('--kev-endpoint', default='http://127.0.0.1:18028/v1/systemone')
    p.add_argument('--max-turn-rate-deg-s', type=float, default=DEFAULT_MAX_TURN_RATE_DEG_S, help='Kev yaw-rate simulation assumption, degrees/second')
    p.add_argument('--acceleration-response', choices=('measured_0_50kn','bounded'), default='measured_0_50kn')
    p.add_argument('--initial-course-deg', type=float, default=90., help='Configured simulated initial direction; default east')
    p.add_argument('--kev-timeout', type=float, default=120.)
    p.add_argument('--device', choices=['cpu','cuda'], default='cpu')
    p.add_argument('--model-path', type=Path, default=ROOT / 'models/Julia-1')
    p.add_argument('--transport', choices=['direct','udp'], default='direct')
    p.add_argument('--scenario', choices=[*ROUTES,'all'], default='all')
    p.add_argument('--max-speed-knots', type=float, default=50.)
    p.add_argument('--steps', type=int, default=100)
    p.add_argument('--dt', type=float, default=1.)
    p.add_argument('--udp-seconds', type=float, default=25.)
    p.add_argument('--output', type=Path, default=ROOT / 'server/experiments/results/julia-cpu')
    args = p.parse_args()
    if not math.isfinite(args.max_speed_knots) or args.max_speed_knots <= 0 or args.steps < 1 or not math.isfinite(args.dt) or args.dt <= 0 or args.udp_seconds <= 0:
        p.error('steps, dt, and udp-seconds must be positive')
    effective_speeds = [10., 20.] if args.backend == 'julia' else [args.max_speed_knots * f for f in (0, .05, .1, .2, .35, .5, .75, 1)]
    if args.backend == 'kev':
        from server.policies.kev_policy import KEV_SPEEDS_KNOTS
        effective_speeds = [speed for speed in KEV_SPEEDS_KNOTS if speed <= args.max_speed_knots]
        print(f'Kev: {effective_speeds} knots x 16 local courses, turn rate <= {args.max_turn_rate_deg_s:g} deg/s, interval {args.dt:g}s, initial direction {args.initial_course_deg:g} deg (simulation assumption).', flush=True)
    if args.backend == 'julia':
        print('Julia uses fixed speeds [10, 20] knots and 16 options; --max-speed-knots does not change Julia speeds.', flush=True)
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=True)
    os.environ['HF_HUB_OFFLINE'] = '1'; os.environ['TRANSFORMERS_OFFLINE'] = '1'
    started = time.perf_counter()
    if args.transport == 'udp':
        results = [udp(args, out)]
    else:
        policy = create_policy(args.backend, model_path=args.model_path, device=args.device,
            duration=args.dt, max_speed_knots=args.max_speed_knots,acceleration_response=args.acceleration_response,
            kev_endpoint=args.kev_endpoint, kev_timeout=args.kev_timeout,
            kev_max_turn_rate_deg_s=args.max_turn_rate_deg_s, kev_initial_course_deg=args.initial_course_deg)
        print(('Kev HTTP adapter ready; model is resident in the separate Kev service.' if args.backend == 'kev' else f'Model loaded in {time.perf_counter()-started:.2f}s'), flush=True)
        results = []
        for name in (ROUTES if args.scenario == 'all' else [args.scenario]):
            result = direct(policy, name, args, out); results.append(result)
            print(json.dumps(result), flush=True)
    selected = list(ROUTES) if args.scenario == 'all' and args.transport == 'direct' else ['straight' if args.scenario == 'all' else args.scenario]
    (out / 'scenarios.json').write_text(json.dumps({n: SCENARIOS[n] for n in selected}, indent=2) + '\n')
    summary = dict(acceleration_response=args.acceleration_response,requested_steps=args.steps, candidate_turn_limit_deg=min(args.max_turn_rate_deg_s * args.dt, 179.99) if args.backend == 'kev' else None, max_turn_rate_deg_s=args.max_turn_rate_deg_s if args.backend == 'kev' else None, initial_course_deg=args.initial_course_deg, completion_mode='final_parking', kev_endpoint=args.kev_endpoint if args.backend == 'kev' else None, dt=args.dt, effective_speeds_knots=effective_speeds, max_speed_knots=args.max_speed_knots, backend=args.backend, device='kev_service' if args.backend == 'kev' else args.device, transport=args.transport,
        model_path=None if args.backend == 'kev' else str(args.model_path), elapsed_s=time.perf_counter()-started, results=results,
        limitations='Idealized motion; direct mode pauses world during inference; sampled speed checks are not continuous verification. UDP mode runs in real time. Cycle-limit status is not physical route completion.')
    (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
