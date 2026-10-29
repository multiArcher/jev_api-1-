"""Check the two field-task runners without calling a model or field endpoints."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from server.experiments import field_tasks as ft


class FieldTaskTests(unittest.TestCase):
    def test_original_tasks_and_speed_units_are_preserved(self):
        for name, count, radius, task_id in (
            ('task_one', 12, 100., '84b6706c-b5d0-4352-9b4f-dd27be357299'),
            ('task_two', 45, 0., 'eaa367de-d805-46a1-a19f-c6dce33113a9'),
        ):
            with self.subTest(name=name):
                task = ft.load_task(name)
                self.assertEqual(task['id'], task_id)
                self.assertEqual(task['params']['currentRoute'], 0)
                self.assertEqual(task['params']['planWaypointNum'], count)
                points = task['params']['planWaypointList']
                self.assertEqual(len(points), count)
                self.assertTrue(all(p['routeRadius'] == radius for p in points))
                self.assertTrue(all(p['routeSpeed'] == 10.28888888888889 for p in points))
                points.clear()
                self.assertEqual(len(ft.load_task(name)['params']['planWaypointList']), count)

    def test_both_functions_start_near_first_point_with_bounded_acceleration(self):
        for runner, name, count in ((ft.test_task_one, 'task_one', 12),
                                    (ft.test_task_two, 'task_two', 45)):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as folder:
                original = ft.load_task(name)
                policy = Mock(return_value=(50., 90.))
                result = runner(folder, ft.TestConfig(steps=3), policy, plots=False)
                self.assertEqual(result['speed_unit'], 'knots')
                self.assertEqual(result['total_waypoints'], count)
                self.assertEqual(result['arrival_radius_m'], 100.)
                self.assertEqual(result['speed_violation_count'], 0)
                self.assertFalse(result['passed'])
                self.assertFalse(result['completed'])
                self.assertAlmostEqual(result['max_speed_knots'], 4.939384939360771)
                self.assertLess(result['max_speed_knots'],3*1.46/(1852/3600))
                output = Path(folder) / name
                self.assertEqual(json.loads((output / 'task.json').read_text()), original)
                rows = [json.loads(line) for line in (output / 'trajectory.jsonl').read_text().splitlines()]
                self.assertEqual(rows[0]['completed_waypoints'], 1)
                self.assertEqual((rows[0]['x'], rows[0]['y']), (-50., 0.))
                self.assertEqual(rows[0]['actual_speed_knots'], 0.)
                self.assertEqual(rows[0]['actual_course_deg'], 90.)
                packet = policy.call_args_list[0].args[0]
                self.assertEqual(packet['params']['planWaypointList'], original['params']['planWaypointList'])
                self.assertEqual(packet['params']['next_waypoint_index'], 1)
                self.assertEqual(json.loads((output / 'summary.json').read_text())['passed'], False)

    def test_arrival_requires_final_stop_and_does_not_call_model_after_arrival(self):
        task = ft.load_task('task_one')
        task['params']['planWaypointList'] = task['params']['planWaypointList'][:1]
        task['params']['planWaypointNum'] = 1
        policy = Mock(return_value=(10., 90.))
        with tempfile.TemporaryDirectory() as folder, patch.object(ft, 'load_task', return_value=task):
            result = ft.test_task_one(folder, ft.TestConfig(steps=10, start_offset_m=105.), policy, plots=False)
            rows = [json.loads(line) for line in (Path(folder) / 'task_one/trajectory.jsonl').read_text().splitlines()]
        self.assertTrue(result['completed'])
        self.assertTrue(result['passed'])
        self.assertEqual(result['reached_waypoints'], 1)
        self.assertLessEqual(result['final_speed_knots'], .1)
        self.assertEqual([r['command']['speed'] for r in rows[-2:]], [0., 0.])
        self.assertEqual(policy.call_count, sum(r['command']['speed'] > 0 for r in rows))

    def test_real_kev_adapter_uses_speed_stages_for_each_task(self):
        for runner in (ft.test_task_one, ft.test_task_two):
            with self.subTest(runner=runner.__name__), tempfile.TemporaryDirectory() as folder:
                client = Mock(return_value={'answers': {'action': {'type': 'choice', 'choice': 'speed_10__heading_07'}}})
                policy = ft.KevPolicy(client=client)
                runner(folder, ft.TestConfig(steps=3), policy, plots=False)
                payload = client.call_args.args[0]
                self.assertEqual(payload['state']['speeds_knots'], [10.])
                self.assertEqual(policy.speeds, [10., 20., 30., 40., 50.])
                self.assertEqual(payload['state']['arrival_radius_m'], 100.)
                self.assertTrue(1 <= len(payload['questions']['action']['criteria']) <= 16)

    def test_invalid_speed_or_turn_is_reported_without_execution(self):
        for action in ((55., 90.), (50., 180.)):
            with self.subTest(action=action), tempfile.TemporaryDirectory() as folder:
                result = ft.test_task_two(folder, ft.TestConfig(steps=2),
                                          Mock(return_value=action), plots=False)
                self.assertEqual(result['reason'], 'invalid_command')
                self.assertFalse(result['passed'])
                self.assertEqual(result['max_speed_knots'], 0.)
                self.assertEqual(result['violations'][0]['kind'], 'command')

    def test_yaw_envelope_uses_packet_course_but_physics_keeps_unrounded_course(self):
        with tempfile.TemporaryDirectory() as folder:
            result=ft.test_task_two(folder, ft.TestConfig(steps=1, initial_course_deg=175.91),
                                    Mock(return_value=(10.,163.)), plots=False)
            row=json.loads((Path(folder)/'task_two/trajectory.jsonl').read_text().splitlines()[0])
        self.assertEqual(row['observed_course_deg'],175.)
        self.assertEqual(row['actual_course_deg'],175.91)
        self.assertNotEqual(result['reason'],'invalid_command')
        self.assertAlmostEqual(result['max_actual_turn_rate_deg_s'],2.)

    def test_model_failure_saves_a_failed_result(self):
        with tempfile.TemporaryDirectory() as folder:
            result = ft.test_task_one(folder, ft.TestConfig(steps=2),
                                      Mock(side_effect=RuntimeError('unavailable')), plots=False)
            self.assertEqual(result['reason'], 'model_error')
            self.assertEqual(result['error'], 'unavailable')
            self.assertFalse(result['passed'])
            self.assertTrue((Path(folder) / 'task_one/summary.json').is_file())

    def test_explicit_internal_start_position_is_preserved_in_trajectory(self):
        with tempfile.TemporaryDirectory() as folder:
            ft.test_task_one(folder, ft.TestConfig(steps=1, initial_position_xy_m=(200.,300.)),
                             Mock(return_value=(10.,90.)), plots=False)
            rows = [json.loads(line) for line in (Path(folder) / 'task_one/trajectory.jsonl').read_text().splitlines()]
        self.assertEqual((rows[0]['x'], rows[0]['y']), (200.,300.))

    def test_cli_runs_both_tasks_by_default_and_propagates_failure(self):
        result = dict(task='mock', passed=False, completed=False, reason='step_limit',
                      reached_waypoints=0, total_waypoints=12, speed_violation_count=0)
        with patch('sys.argv', ['field_tasks.py', '--no-plots']), \
                patch.object(ft, 'test_task_one', return_value=result) as one, \
                patch.object(ft, 'test_task_two', return_value=result) as two:
            self.assertEqual(ft.main(), 1)
            one.assert_called_once()
            two.assert_called_once()

    def test_invalid_simulation_settings_are_rejected(self):
        for settings in ({'steps': 0}, {'dt': 0.}, {'start_offset_m': float('nan')},
                         {'initial_course_deg': 360.}, {'max_turn_rate_deg_s': -1.},
                         {'initial_position_xy_m': (0., float('nan'))}):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                ft.TestConfig(**settings)

    def test_checkpoint_images_are_saved_at_each_crossed_ten_point_milestone(self):
        task = ft.load_task('task_two')
        task['params']['planWaypointList'] = [dict(task['params']['planWaypointList'][0]) for _ in range(21)]
        task['params']['planWaypointNum'] = 21
        with tempfile.TemporaryDirectory() as folder, patch.object(ft, 'load_task', return_value=task), \
                patch.object(ft, '_plot'), patch.object(ft, '_plot_checkpoint', return_value=Path(folder)/'image.png') as render:
            result = ft.test_task_two(folder, ft.TestConfig(steps=1), Mock(), checkpoint_every=10)
            saved = json.loads((Path(folder)/'task_two/checkpoints.json').read_text())
            self.assertTrue(result['passed'])
            self.assertEqual([c['milestone'] for c in saved], [10, 20])
            self.assertEqual([call.args[-1]['milestone'] for call in render.call_args_list], [10, 20, 21])
            self.assertTrue(render.call_args.args[-1]['final'])

    def test_invalid_checkpoint_interval_and_no_plots(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(ValueError):
                ft.test_task_two(folder, ft.TestConfig(steps=1), Mock(), checkpoint_every=-1)
            with patch.object(ft, '_plot_checkpoint') as render:
                result=ft.test_task_two(folder, ft.TestConfig(steps=1), Mock(return_value=(10.,90.)),
                                        plots=False, checkpoint_every=10)
            render.assert_not_called()
            self.assertEqual(result['checkpoints'], [])


if __name__ == '__main__':
    unittest.main()
