"""Progress checks use observed distance and heading, without a live model."""
import copy
import math
import unittest
from unittest.mock import Mock

from server.experiments.field_tasks import load_task
from server.policies.kev_policy import KevPolicy
from server.policies.progress import NavigationStalled, ProgressConfig, ProgressSupervisor
from server.tests.test_kev_policy import answer, fastest_straight
from tools.waypoint_geometry import EARTH_RADIUS, LocalFrame, WaypointGeometry


class ProgressTests(unittest.TestCase):
    def test_approach_and_turning_away_with_improving_heading_do_not_trigger_recovery(self):
        for approach in (True, False):
            supervisor = ProgressSupervisor()
            for i in range(12):
                distance = 1000 - i * 10 if approach else 1000 + i * 10
                error = 170 if approach else 170 - i * 10
                self.assertEqual(supervisor.update(0, distance, error)['mode'], 'normal')

    def test_straight_away_enters_recovery_and_persistent_failure_is_terminal(self):
        supervisor = ProgressSupervisor(ProgressConfig(recovery_limit=3))
        for i in range(6):
            state = supervisor.update(0, 1000 + i * 10, 170)
        self.assertEqual(state['mode'], 'recovery')
        supervisor.update(0, 1060, 170)
        supervisor.update(0, 1070, 170)
        with self.assertRaises(NavigationStalled):
            supervisor.update(0, 1080, 170)

    def test_recovery_exits_after_alignment_and_approach(self):
        supervisor = ProgressSupervisor()
        for i in range(6):
            supervisor.update(0, 1000 + i * 10, 170)
        for i in range(5):
            state = supervisor.update(0, 1040 - i * 10, 140 - i * 30)
        self.assertEqual(state['mode'], 'normal')

    def test_waypoint_switch_clears_recovery_and_its_window(self):
        supervisor = ProgressSupervisor()
        for i in range(6):
            supervisor.update(0, 1000 + i * 10, 170)
        self.assertEqual(supervisor.update(1, 4000, 179), dict(mode='normal', recovery_cycles=0))
        self.assertEqual(len(supervisor.samples), 1)

    def test_slow_but_improving_recovery_does_not_exhaust_failure_budget(self):
        supervisor = ProgressSupervisor(ProgressConfig(recovery_limit=3))
        for i in range(6):
            supervisor.update(0, 1000 + i * 10, 170)
        for i in range(1, 15):
            state = supervisor.update(0, 1050 + i * 5, 170 - i * 3)
        self.assertEqual(state['mode'], 'recovery')
        self.assertEqual(state['recovery_cycles'], 0)

    def test_policy_recovery_restricts_speed_and_new_task_resets_it(self):
        task = load_task('task_one')
        first = task['params']['planWaypointList'][0]
        client = Mock(side_effect=fastest_straight)
        policy = KevPolicy(client=client)
        original = copy.deepcopy(task)
        # Move west away from the first target, continuing to face west.
        for i in range(6):
            lon = first['routeLon'] - math.degrees((200 + i * 10) / (EARTH_RADIUS * math.cos(math.radians(first['routeLat']))))
            packet = dict(task, situation=[dict(platformId=1010, longitude=lon,
                          latitude=first['routeLat'], targetSpeed=5., targetCourse=270.)])
            policy(packet)
        payload = client.call_args.args[0]
        self.assertEqual(payload['state']['progress']['mode'], 'recovery')
        self.assertEqual(payload['state']['speeds_knots'], [10.])
        self.assertTrue(1 <= len(payload['questions']['action']['criteria']) <= 16)
        self.assertEqual(task, original)
        packet['id'] = 'new-task'
        policy(packet)
        self.assertEqual(client.call_args.args[0]['state']['progress']['mode'], 'normal')
        self.assertEqual(client.call_args.args[0]['state']['recent_decisions'], [])
        packet['params'] = dict(packet['params'], next_waypoint_index=1)
        policy(packet)
        self.assertEqual(client.call_args.args[0]['state']['recent_decisions'], [])

    def test_recovery_rejects_a_high_speed_model_choice(self):
        task = load_task('task_one')
        first = task['params']['planWaypointList'][0]
        client = Mock(side_effect=fastest_straight)
        policy = KevPolicy(client=client)
        for i in range(5):
            packet = dict(task, situation=[dict(platformId=1010,
                          longitude=first['routeLon'] - .002 - i * .0001,
                          latitude=first['routeLat'], targetSpeed=5., targetCourse=270.)])
            policy(packet)
        packet['situation'][0]['longitude'] -= .0001
        client.side_effect = None
        client.return_value = answer('speed_50__heading_07')
        with self.assertRaisesRegex(ValueError, 'available speed/course'):
            policy(packet)


class FixedFrameTests(unittest.TestCase):
    def test_previous_circle_limits_speed_options(self):
        task = load_task('task_one')
        first = task['params']['planWaypointList'][0]
        client = Mock(side_effect=fastest_straight)
        policy = KevPolicy(client=client)
        packet = dict(task, situation=[dict(platformId=1010,
                      longitude=first['routeLon'] - .0004, latitude=first['routeLat'],
                      targetSpeed=5., targetCourse=90.)])
        policy(packet)
        options = client.call_args.args[0]['questions']['action']['criteria']
        self.assertTrue(all(not v['speed_limit_violation'] for v in options.values()))
        self.assertNotIn('speed_50__heading_07', options)
        self.assertEqual(client.call_args.args[0]['state']['speed_profile']['phase'], 'heading_alignment')

    def test_origin_is_first_waypoint_and_stays_fixed_when_boat_moves(self):
        task = load_task('task_one')
        first = task['params']['planWaypointList'][0]
        frame = LocalFrame(task['params']['planWaypointList'])
        self.assertEqual(frame.project(first['routeLon'], first['routeLat']), (0., 0.))
        original = copy.deepcopy(task)
        policy = KevPolicy(client=Mock(side_effect=fastest_straight))
        positions = []
        for lon_offset in (-.002, -.003):
            packet = dict(task, situation=[dict(platformId=1010,
                          longitude=first['routeLon'] + lon_offset, latitude=first['routeLat'],
                          targetSpeed=10., targetCourse=90.)])
            obs = policy.observation(packet)
            geometry = WaypointGeometry(obs)
            positions.append(geometry.boat_position)
            self.assertEqual(geometry.frame.origin, (first['routeLon'], first['routeLat']))
            self.assertEqual(geometry.waypoint_positions[0], (0., 0.))
            self.assertEqual(geometry.target, (-geometry.boat_position[0], -geometry.boat_position[1]))
            policy(packet)
            state = policy.client.call_args.args[0]['state']
            self.assertEqual(state['target_xy_m'], [0., 0.])
            self.assertEqual(state['boat_xy_m'][0], round(geometry.boat_position[0], 2))
            self.assertNotIn('longitude', state)
        self.assertNotEqual(positions[0], positions[1])
        self.assertEqual(task, original)

    def test_session_and_policy_use_the_same_fixed_frame(self):
        from server.decision_service import ControlSession
        task = load_task('task_two')
        session = ControlSession(task, ('127.0.0.1', 0))
        first = task['params']['planWaypointList'][0]
        state = dict(platformId=1010, longitude=first['routeLon'] - .001,
                     latitude=first['routeLat'], targetSpeed=0., targetCourse=90.)
        session.update_situation(state)
        policy = KevPolicy(client=Mock())
        geometry = WaypointGeometry(policy.observation(dict(task, situation=[state])))
        self.assertEqual(session.frame.origin, geometry.frame.origin)
        self.assertEqual(session.waypoint_positions, geometry.waypoint_positions)
        self.assertEqual(session.current_route, policy.current_route())


if __name__ == '__main__':
    unittest.main()
