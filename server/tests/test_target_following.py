"""Following protocol, joint forecasts, guards, loss recovery and stop lifecycle."""
import copy
import math
import threading
import select
import socket
import subprocess
import sys
import time
from pathlib import Path
import unittest
from unittest.mock import MagicMock, Mock, patch

from server import decision_service as ds
from server.navigation.target_following import (TargetUnavailable, target_record,
    range_facts, relative_state)
from server.policies.kev_policy import KevPolicy


def task():
    return dict(simId='offline', id='following-test', code='targetFollowing', params=dict(
        platCode=1010, targetPlatformId=3001, minFollowingDistanceM=150., maxFollowingDistanceM=250.))


def own():
    return dict(platformId=1010, longitude=120., latitude=30., targetSpeed=10., targetCourse=90.)


def target(distance=200.):
    return dict(own(), platformId=3001, longitude=120.+distance/(6378137*math.pi/180*math.cos(math.radians(30))))


def choose(payload):
    criteria = payload['questions']['action']['criteria']
    key = min(criteria, key=lambda k: (criteria[k]['reference_error_m'], criteria[k]['target_heading_error_deg']))
    return dict(answers=dict(action=dict(type='choice', choice=key)))


class FollowingTests(unittest.TestCase):
    def test_invalid_tasks_rejected(self):
        for field, value in [('targetPlatformId', 1010), ('targetPlatformId', True),
                             ('minFollowingDistanceM', -1.), ('maxFollowingDistanceM', 150.),
                             ('targetMaxAgeS', float('nan')), ('currentRoute', 0)]:
            packet = task()
            packet['params'][field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                ds.ControlSession(packet, ('127.0.0.1', 1))

    def test_band_entry_does_not_complete_or_report_waypoint_progress(self):
        session = ds.ControlSession(task(), ('127.0.0.1', 1))
        session.update_situation(own())
        for distance, stage in ((400., 'too_far'), (200., 'in_band'), (110., 'too_close')):
            session.update_target((target(distance),))
            fields = session.status_fields()[1]
            self.assertEqual(fields['followingStage'], stage)
            self.assertAlmostEqual(fields['targetDistanceM'], distance, places=2)
            self.assertFalse(session.route_complete)
            self.assertNotIn('currentRoute', fields)
            self.assertNotIn('completionRate', fields)
            self.assertNotIn('nextWaypoint', fields)

    def test_missing_stale_unknown_target_clears_availability(self):
        for records in ((), (dict(target(), _received_age_s=3.),),
                        (dict(target(), targetCourse=None),), (dict(target(), targetSpeed=-1),)):
            session = ds.ControlSession(task(), ('127.0.0.1', 1))
            session.update_situation(own())
            session.update_target((target(),))
            with self.assertRaises(TargetUnavailable):
                session.update_target(records)
            self.assertFalse(session.status_fields()[1]['targetAvailable'])

    def test_knots_and_mps_relative_forecast_and_age_agree(self):
        boat, tracked = own(), dict(target(), _received_age_s=1.)
        knots = relative_state(boat, tracked, 'knots')
        mps = relative_state(dict(boat, targetSpeed=boat['targetSpeed']*ds.KNOT_TO_MPS),
                             dict(tracked, targetSpeed=tracked['targetSpeed']*ds.KNOT_TO_MPS), 'mps')
        self.assertEqual(knots, mps)
        self.assertAlmostEqual(knots[2], 200.+10*ds.KNOT_TO_MPS)
        self.assertAlmostEqual(knots[3], 0.)

    def test_swept_range_detects_crossing_between_samples(self):
        motion = dict(path=((0., 0.), (0., 0.)), _times=(0., 1.), end_course_deg=0., end_speed=0.)
        facts = range_facts(motion, (-10., 0.), (20., 0.))
        self.assertEqual(facts[0], 0.)
        self.assertEqual(facts[2], 10.)

    def test_follows_moving_target_using_model_and_preserves_input(self):
        packet = dict(task(), situation=(own(), target(400.)))
        before = copy.deepcopy(packet)
        policy = KevPolicy(client=choose)
        speed, course = policy(packet)
        self.assertGreater(speed, 10.)
        self.assertLessEqual(abs(course-90), policy.max_turn_rate_deg_s)
        self.assertEqual(packet, before)
        self.assertEqual(policy.local.last_decision['state']['task'], 'line_of_sight_distance_following')
        self.assertIn('radial_braking_margin_m', next(iter(policy.local.last_decision['options'].values())))

    def test_matching_low_speed_and_zero_are_available_in_band(self):
        for unit in ('knots', 'mps'):
            factor = ds.KNOT_TO_MPS if unit == 'mps' else 1.
            boat = dict(own(), targetSpeed=3.*factor)
            tracked = dict(target(), targetSpeed=3.*factor)
            client = Mock(side_effect=choose)
            policy = KevPolicy(client=client, speed_unit=unit)
            action = policy(dict(task(), situation=(boat, tracked)))
            self.assertAlmostEqual(action[0], round(3.*factor, 2), delta=.01)
            speeds = {c['speed_knots'] for c in policy.local.last_decision['options'].values()}
            self.assertIn(0., speeds)
            self.assertTrue(any(abs(s-3.) < .01 for s in speeds))

    def test_safe_catchup_candidates_cannot_freeze_an_excessive_range(self):
        policy = KevPolicy(client=choose)
        policy(dict(task(), situation=(own(), target(400.))))
        self.assertEqual(policy.local.last_decision['state']['recovery_phase'], 'close_range')
        self.assertTrue(all(c['range_reduction_m'] > 0 for c in policy.local.last_decision['options'].values()))

    def test_stopped_target_in_band_settles_to_zero(self):
        policy = KevPolicy(client=choose)
        action = policy(dict(task(), situation=(own(), dict(target(),targetSpeed=0.))))
        self.assertEqual(action[0], 0.)
        self.assertEqual(policy.local.last_decision['state']['recovery_phase'], 'stationary_hold')

    def test_initial_course_fallback_handles_stationary_own_without_course(self):
        boat = own()
        boat.pop('targetCourse')
        boat['targetSpeed'] = 0.
        session = ds.ControlSession(task(), ('127.0.0.1',1))
        session.update_situation(boat)
        session.update_target((target(),))
        self.assertIsNotNone(session.status_fields()[1]['rangeRateMps'])
        policy = KevPolicy(client=choose,initial_course_deg=90.)
        self.assertGreaterEqual(policy(dict(task(),situation=(boat,target())))[0],0.)

    def test_invalid_model_answer_and_truncation_rejected(self):
        for result, error in ((dict(answers=dict(action=dict(type='choice', choice='bad'))), ValueError),
                              (dict(truncated=True), RuntimeError)):
            with self.assertRaises(error):
                KevPolicy(client=lambda _: result)(dict(task(), situation=(own(), target())))

    def test_unsafe_minimum_is_rejected_before_inference(self):
        packet = task()
        packet['params']['minFollowingDistanceM'] = 50.
        client = Mock()
        with self.assertRaises(ValueError):
            KevPolicy(client=client)(dict(packet, situation=(own(), target())))
        client.assert_not_called()

    def test_static_obstacle_remains_in_candidate_guards(self):
        obstacle = dict(target(120.), platformId=4001, targetSpeed=0.)
        policy = KevPolicy(client=choose)
        packet = dict(task(), situation=(own(), target(400.), obstacle))
        policy(packet)
        self.assertTrue(policy.local.last_decision['options'])
        for candidate in policy.local.last_decision['options'].values():
            self.assertTrue(policy.validate_control(packet, own(), packet['situation'],
                candidate['speed_knots'], candidate['absolute_course_deg'])['safe'])


class FollowingLifecycleTests(unittest.TestCase):
    def run_loop(self, *, lose_target=False, transient=False):
        session = ds.ControlSession(task(), ('127.0.0.1', 1))
        registry = ds.TaskRegistry()
        registry.add(session)
        boat = own()
        commands = []
        lost = False
        missing_reads = 0
        class Store:
            def get(self, *_):
                return dict(boat)
            def query_state(self, **_):
                nonlocal missing_reads
                if lost and (not transient or missing_reads < 1):
                    missing_reads += 1
                    return ()
                return (target(),)
        policy = Mock(return_value=(10., 90.))
        def sent(payload, _):
            nonlocal lost
            command = ds.decode_speed_course_packet(payload)
            commands.append(command)
            if command['speed'] > 0:
                if lose_target:
                    lost = True
                    if transient and missing_reads:
                        session.request_stop()
                else:
                    session.request_stop()
            else:
                boat['targetSpeed'] = 0.
        context = MagicMock()
        context.__enter__.return_value = Mock(sendto=Mock(side_effect=sent))
        status = Mock()
        watchdog = threading.Timer(3., session.request_stop)
        watchdog.start()
        try:
            with patch.object(ds.socket, 'socket', return_value=context), patch.object(ds, 'LOGGER'):
                ds.run_control_loop(session, status, Store(), ('127.0.0.1', 1), policy,
                    .005, 0, 2.5, .1, registry, 3, .2)
        finally:
            watchdog.cancel()
        return ds.analyze_task(status.sendto.call_args.args[0]), commands, policy

    def test_explicit_stop_confirms_actual_zero_without_target_requirement(self):
        status, commands, _ = self.run_loop()
        self.assertEqual(status['status'], 2)
        self.assertTrue(status['params']['stopConfirmed'])
        self.assertEqual(commands[-1]['speed'], 0.)
        self.assertNotIn('currentRoute', status['params'])

    def test_target_loss_stops_immediately_then_fails_after_three_retries(self):
        status, commands, _ = self.run_loop(lose_target=True)
        self.assertEqual(status['status'], 3)
        self.assertEqual(status['params']['errorCode'], 'FOLLOWING_TARGET_UNAVAILABLE')
        self.assertEqual(status['params']['consecutiveErrors'], 3)
        self.assertTrue(all(c['speed'] == 0. for c in commands[1:]))
        self.assertIn('追踪目标3001', status['description'])

    def test_transient_target_loss_recovers_before_retry_limit(self):
        status, commands, _ = self.run_loop(lose_target=True, transient=True)
        self.assertEqual(status['status'], 2)
        first_stop = next(i for i, c in enumerate(commands) if c['speed'] == 0.)
        self.assertTrue(any(c['speed'] > 0 for c in commands[first_stop+1:]))


class FollowingUdpTests(unittest.TestCase):
    def test_real_udp_accepts_following_continues_in_band_and_stops(self):
        script = """
import sys
from server.decision_service import start_server
from server.policies.kev_policy import KevPolicy
from server.experiments.target_following import mock_choice
start_server(host='127.0.0.1', port=int(sys.argv[1]),
    policy=KevPolicy(client=mock_choice, duration=.02),
    situation_port=0, multicast_interface='127.0.0.1',
    action_host='127.0.0.1', action_port=int(sys.argv[2]),
    control_interval=.02, status_interval=.02, platform_id=1010)
"""
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as peer, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as control:
            peer.bind(('127.0.0.1',0))
            control.bind(('127.0.0.1',0))
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as reserve:
                reserve.bind(('127.0.0.1',0))
                port = reserve.getsockname()[1]
            address = ('127.0.0.1',port)
            process = subprocess.Popen([sys.executable,'-c',script,str(port),str(control.getsockname()[1])],
                cwd=Path(__file__).resolve().parents[2],stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
            boat = dict(own(), direction=0,sign=0,height=0,type=0,batch=0)
            tracked = dict(target(),direction=0,sign=0,height=0,type=0,batch=0)
            accepted = stopping = False
            motions = 0
            terminal = None
            deadline = time.monotonic()+5.
            try:
                while terminal is None and time.monotonic() < deadline:
                    peer.sendto(ds.build_situation_packet(boat),address)
                    if not stopping:
                        peer.sendto(ds.build_situation_packet(tracked),address)
                    if not accepted:
                        peer.sendto(ds.build_json_packet(task(),ds.ALGORITHM_TASK_ID),address)
                    if stopping:
                        peer.sendto(ds.build_json_packet(task(),ds.ALGORITHM_STOP_ID),address)
                    ready,_,_ = select.select([peer,control],[],[],.02)
                    for endpoint in ready:
                        payload,_ = endpoint.recvfrom(65507)
                        if endpoint is control:
                            command = ds.decode_speed_course_packet(payload)
                            if command['speed'] > 0:
                                motions += 1
                                if motions >= 3:
                                    stopping = True
                            else:
                                boat['targetSpeed'] = 0.
                        else:
                            status = ds.analyze_task(payload)
                            accepted = True
                            self.assertNotIn('currentRoute',status['params'])
                            self.assertEqual(status['params']['targetPlatformId'],3001)
                            if status['status'] in (2,3):
                                terminal = status
                self.assertGreaterEqual(motions,3,'following ended on band entry')
                self.assertIsNotNone(terminal)
                self.assertEqual(terminal['status'],2,terminal)
                self.assertTrue(terminal['params']['stopConfirmed'])
            finally:
                process.terminate()
                process.communicate(timeout=3.)
