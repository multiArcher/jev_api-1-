"""Per-task model worker and safe error metadata; no socket ownership here."""

from concurrent.futures import Future
import math
import os
import queue
import re
import threading
import socket
import time
from urllib.error import HTTPError, URLError


class PolicyWorker:
    """Keep policy thread-local state on one daemon thread while control keeps ticking."""

    def __init__(self, policy):
        self.policy = policy
        self.requests = queue.Queue(maxsize=1)
        self.closed = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True, name="policy-worker")
        self.thread.start()

    def submit(self, packet):
        future = Future()
        self.requests.put_nowait((future, packet))
        return future

    def close(self):
        self.closed.set()
        try:
            self.requests.put_nowait(None)
        except queue.Full:
            pass

    def _run(self):
        while not self.closed.is_set():
            request = self.requests.get()
            if request is None:
                return
            future, packet = request
            if not future.set_running_or_notify_cancel():
                continue
            try:
                future.set_result(self.policy(packet))
            except Exception as exc:
                future.set_exception(exc)


class TaskSuperseded(Exception):
    """Internal cancellation: revoking control must not send emergency stop."""


def error_details(stage, exc):
    text = " ".join(str(exc).split())
    key = os.environ.get("JEV_API_KEY", "").strip()
    if key:
        text = text.replace(key, "[redacted]")
    text = re.sub(r"Bearer\s+\S+|apikey_[A-Za-z0-9_]+", "[redacted]", text, flags=re.I)
    timeout = isinstance(exc, TimeoutError) or "timed out" in text.lower() or "timeout" in text.lower()
    if stage == "situation":
        code = "SITUATION_TIMEOUT" if timeout else "INVALID_SITUATION"
    elif stage == "inference":
        code = "MODEL_TIMEOUT" if timeout else "MODEL_REQUEST_FAILED"
    elif stage == "control":
        code = "CONTROL_SEND_FAILED"
    else:
        code = "INVALID_TASK"
    return dict(errorCode=getattr(exc, 'error_code', code), errorStage=stage, errorType=type(exc).__name__, lastError=text[:600])


def retryable_error(stage, exc):
    if isinstance(exc, HTTPError):
        return exc.code in (408, 429) or exc.code >= 500
    if isinstance(exc, (OSError, URLError)):
        return True
    if stage == "inference" and isinstance(exc, RuntimeError):
        message = str(exc).lower()
        status = re.search(r"http\s+(\d{3})", message)
        if status:
            code = int(status.group(1))
            return code in (408, 429) or code >= 500
        return "not configured" not in message and "truncated" not in message
    return False


def run_task_control(session, status_socket, situations, action_target, policy,
                     control_interval, max_cycles, situation_max_age, stop_speed_threshold,
                     registry, max_consecutive_errors, command_hold_timeout,
                     build_control, build_status, logger):
    """Own task lifecycle independently of synchronous model request latency."""
    task = session.task
    plat_code = task['params'].get('platCode')
    worker = None
    pending = None
    pending_route = None
    pending_started = None
    last_action = None
    last_action_at = None
    cycles = 0
    deadline = time.monotonic()
    failed_this_tick = False
    distance_monitor = (policy.create_distance_monitor(logger, task, situation_max_age)
                        if callable(getattr(type(policy), 'create_distance_monitor', None)) else None)

    def situation_snapshot(own):
        # Real stores expose all targets; old injected test stores may expose
        # only get(). Own-ship freshness remains enforced separately above.
        query = getattr(type(situations), 'query_state', None)
        records = situations.query_state(include_age=True) if callable(query) else ()
        return (dict(own),) + tuple(dict(r) for r in records if r.get('platformId') != plat_code)

    def stop_course():
        state = session.latest_state or {}
        course = state.get('targetCourse')
        if isinstance(course, (int, float)) and math.isfinite(course) and 0 <= course < 360:
            return float(course)
        return last_action[1] if last_action is not None else 0.

    def send_terminal(status, description):
        # Serialize final and periodic reports so status=1 cannot follow a final
        # result and overwrite it in the behavior tree's simId/id cache.
        with session.output_lock, session.status_lock:
            if session.superseded.is_set():
                return
            send_terminal_locked(status, description)

    def send_terminal_locked(status, description):
        session.finished.set()
        _, fields = session.status_fields()
        packet = build_status(task, status, description, fields)
        logger.info('task %s terminal status=%d: %s', task.get('id'), status, description)
        for attempt in range(max_consecutive_errors):
            try:
                status_socket.sendto(packet, session.sender)
                return
            except OSError as exc:
                logger.warning('task %s terminal status send %d/%d failed: %s',
                               task.get('id'), attempt + 1, max_consecutive_errors, exc)
                if attempt + 1 < max_consecutive_errors:
                    time.sleep(min(control_interval, .1))

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as control_socket:
        def send_control(speed, course, mode):
            if speed > 0 and callable(getattr(type(policy), 'validate_control', None)):
                fresh = situations.get(plat_code, situation_max_age)
                check = (policy.validate_control(task, fresh, situation_snapshot(fresh), speed, course)
                         if fresh is not None else dict(safe=False, obstacle_id=None, min_clearance_m=None))
                if not check['safe']:
                    logger.warning('task %s static obstacle control veto target=%s clearance=%s mode=%s; sending zero speed',
                                   task.get('id'), check['obstacle_id'], check['min_clearance_m'], mode)
                    speed, course, mode = 0., stop_course(), 'obstacle_stop'
            if callable(getattr(type(policy), 'validate_dynamic_control', None)):
                fresh=situations.get(plat_code,situation_max_age)
                records=situation_snapshot(fresh) if fresh is not None else ()
                check=(policy.validate_dynamic_control(task,fresh,records,speed,course)
                       if fresh is not None else dict(safe=False,obstacle_id=None,min_clearance_m=None))
                if not check['safe']:
                    logger.warning('task %s dynamic obstacle control veto target=%s clearance=%s mode=%s; sending zero speed',
                        task.get('id'),check['obstacle_id'],check['min_clearance_m'],mode)
                    moving=speed>0
                    speed,course,mode=0.,stop_course(),'dynamic_obstacle_stop'
                    if moving and fresh is not None:
                        stopped=policy.validate_dynamic_control(task,fresh,records,speed,course)
                        if not stopped['safe']:
                            logger.warning('task %s dynamic obstacle zero-speed risk target=%s clearance=%s; awaiting new decision',
                                task.get('id'),stopped['obstacle_id'],stopped['min_clearance_m'])
            packet = build_control(plat_code, speed, course)
            with session.output_lock:
                if session.superseded.is_set():
                    raise TaskSuperseded()
                control_socket.sendto(packet, action_target)
                logger.info(
                    'sent control 0x0662 to %s:%d task=%s platCode=%d '
                    'speed=%.2f %s course=%.2f deg mode=%s packet=%s',
                    *action_target, task.get('id'), plat_code,
                    round(speed * 100) / 100, session.speed_unit,
                    round(course * 100) / 100, mode, packet.hex(' '),
                )

        def send_stop():
            session.stop_command_sent = False
            send_control(0., stop_course(), 'stop')
            session.stop_command_sent = True

        def issue(stage, exc, force_terminal=False):
            nonlocal failed_this_tick
            if session.superseded.is_set():
                raise TaskSuperseded()
            if not failed_this_tick:
                session.record_error(stage, exc, max_consecutive_errors)
                failed_this_tick = True
            description, _ = session.status_fields()
            logger.warning('task %s: %s', task.get('id'), description)
            if not force_terminal and retryable_error(stage, exc) and session.consecutive_errors < max_consecutive_errors:
                return False
            try:
                send_stop()
                stop_text = '已下发零速停车指令，尚未确认实际停车'
            except (OSError, ValueError, TypeError) as stop_error:
                stop_text = '停车指令发送失败：{}'.format(error_details('control', stop_error)['lastError'])
            reason = ('连续错误达到{}次'.format(max_consecutive_errors)
                      if session.consecutive_errors >= max_consecutive_errors else '不可恢复错误')
            description = '{}；{}，任务终止；{}'.format(description, reason, stop_text)
            session.set_phase('failed', description)
            logger.error('task %s failed: %s', task.get('id'), description)
            send_terminal(3, description)
            return True

        try:
            if type(plat_code) is not int or not 0 <= plat_code <= 0xFFFFFFFF:
                issue('task', ValueError('params.platCode must be an integer in [0, 4294967295]'))
                return
            worker = PolicyWorker(policy)
            while not session.finished.is_set():
                failed_this_tick = False
                new_action = False
                now = time.monotonic()
                own_state = situations.get(plat_code, situation_max_age)
                if own_state is not None:
                    try:
                        if distance_monitor is not None:
                            policy.record_actual_distance(distance_monitor, task, own_state,
                                                          situation_snapshot(own_state))
                        if callable(getattr(type(policy), 'validate_position', None)):
                            policy.validate_position(task, own_state, situation_snapshot(own_state))
                        session.update_situation(own_state)
                        if session.errors.get('errorStage') == 'situation':
                            session.clear_errors()
                    except (ValueError, TypeError, KeyError) as exc:
                        issue('situation', exc)
                        return

                stopping = session.stop_requested.is_set()
                completed = session.route_complete
                cycle_limit = max_cycles > 0 and cycles >= max_cycles
                if stopping or completed or cycle_limit:
                    phase = 'stopping' if stopping else 'finishing'
                    reason = ('收到停止请求' if stopping else
                              ('全部航路点已到达' if completed else '已达到控制周期上限'))
                    session.set_phase(phase, '{}，正在下发零速并等待态势确认停车'.format(reason))
                    try:
                        send_stop()
                    except OSError as exc:
                        if issue('control', exc):
                            return
                    else:
                        if own_state is not None and own_state['targetSpeed'] <= stop_speed_threshold:
                            session.stop_confirmed = True
                            description = 'stopped' if stopping else '{}，实际速度已降至停车阈值，任务完成'.format(reason)
                            session.set_phase('stopped' if stopping else 'completed', description)
                            send_terminal(2, description)
                            return
                    if own_state is None and issue('situation', TimeoutError(
                            '停车确认缺少新鲜态势，最大允许态势年龄为{}秒'.format(situation_max_age))):
                        return
                else:
                    if own_state is None:
                        if issue('situation', TimeoutError(
                                '未收到本艇新鲜态势，最大允许态势年龄为{}秒'.format(situation_max_age))):
                            return
                    else:
                        if pending is not None and pending.done():
                            try:
                                action = pending.result()
                                if not isinstance(action, (tuple, list)) or len(action) != 2:
                                    raise ValueError('model must return (speed, course)')
                                build_control(plat_code, action[0], action[1])
                                # Progress/stop decisions from fresh telemetry always win over an old request.
                                if pending_route == session.current_route:
                                    if now - pending_started > command_hold_timeout:
                                        raise TimeoutError('模型结果已过期：请求耗时超过{}秒控制有效期'.format(command_hold_timeout))
                                    last_action = tuple(action)
                                    last_action_at = now
                                    new_action = True
                            except Exception as exc:
                                if issue('inference', exc):
                                    return
                            finally:
                                pending = None
                        if pending is None and not (max_cycles > 0 and cycles + int(new_action) >= max_cycles):
                            packet = dict(task)
                            packet['params'] = dict(task['params'], next_waypoint_index=session.current_route)
                            packet['situation'] = situation_snapshot(own_state)
                            pending_route = session.current_route
                            pending_started = now
                            pending = worker.submit(packet)

                    # A model request never blocks the periodic control sender.
                    if session.stop_requested.is_set():
                        try:
                            send_stop()
                        except OSError as exc:
                            if issue('control', exc):
                                return
                    elif last_action is not None:
                        try:
                            if now - last_action_at <= command_hold_timeout:
                                send_control(*last_action, 'new' if new_action else 'hold')
                                if last_action[0] > 0:
                                    session.stop_command_sent = False
                                if new_action:
                                    cycles += 1
                                    session.clear_errors()
                                    logger.debug('task %s cycle=%d target=%d/%d speed=%.2f course=%.2f',
                                                task.get('id'), cycles, session.current_route + 1,
                                                len(session.waypoints), *last_action)
                                elif session.errors.get('errorStage') == 'control':
                                    session.clear_errors()
                            else:
                                send_stop()
                                description, _ = session.status_fields()
                                session.set_phase('waiting',
                                    '{}；有效指令超过{}秒续发时限，暂发零速，等待新决策'.format(
                                        description.split('；有效指令超过')[0], command_hold_timeout))
                        except OSError as exc:
                            if issue('control', exc):
                                return

                deadline += control_interval
                now = time.monotonic()
                if deadline <= now:
                    deadline = now + control_interval
                delay = deadline - now
                if session.stop_requested.is_set():
                    time.sleep(delay)
                else:
                    session.stop_requested.wait(delay)
        except TaskSuperseded:
            pass
        except Exception as exc:
            try:
                issue('task', exc, force_terminal=True)
            except TaskSuperseded:
                pass
        finally:
            if distance_monitor is not None:
                distance_monitor.close()
            if pending is not None:
                pending.cancel()
            if worker is not None:
                worker.close()
            session.finished.set()
            registry.remove(session)
