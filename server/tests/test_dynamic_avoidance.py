"""Independent dynamic planning regressions; no model or transport dependencies."""
from dataclasses import replace
import math
import unittest

from server.navigation.dynamic_avoidance import (
    Action, AvoidanceConfig, DynamicAvoidancePlanner, Target, Vessel,
    assess_action, forecast, scene_from_situation)
from server.navigation.dynamic_avoidance.prediction import Trajectory, assess_trajectory
from server.experiments.dynamic_avoidance import verify, advance, shifted


class DynamicAvoidanceTests(unittest.TestCase):
    def setUp(self):
        self.config = AvoidanceConfig()
        self.own = Vessel((0., 0.), 10., 0.)
        self.desired = Action(10., 0.)

    def plan(self, targets, own=None, desired=None):
        return DynamicAvoidancePlanner(self.config).plan(own or self.own, targets, desired or self.desired)

    def test_default_radius_and_validation(self):
        self.assertAlmostEqual(self.config.safe_distance_m, 86.04400529476)
        for kwargs in ({'dt_s': 0}, {'acceleration_mps2': -1}, {'horizon_s': math.nan},
                       {'unknown_target_speed_mps': False}, {'heading_step_deg': 181}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                AvoidanceConfig(**kwargs)
        with self.assertRaises(ValueError):
            Vessel((math.nan, 0), 1, 0)
        with self.assertRaises(ValueError):
            Action(-1, 0)

    def test_no_targets_preserves_desired(self):
        result = self.plan([])
        self.assertEqual((result.status, result.action), ('clear', self.desired))
        self.assertIsNone(result.assessment.min_clearance_m)

    def test_head_on_crossings_overtaking_and_static(self):
        cases = [Target(1, (0, 600), 10, 180), Target(1, (-300, 300), 10, 90),
                 Target(1, (300, 300), 10, 270), Target(1, (0, 200), 3, 0),
                 Target(1, (0, 300), 0, None)]
        for target in cases:
            with self.subTest(target=target):
                result = self.plan([target])
                self.assertEqual(result.status, 'avoiding')
                self.assertTrue(result.assessment.safe)
                self.assertGreater(result.action.course_deg, 0)
                self.assertLess(result.action.course_deg, 180)
                self.assertGreater(verify(self.own, [target], result.action, self.config), 0)

    def test_parallel_and_receding_preserve_action(self):
        for target in (Target(1, (200, 0), 10, 0), Target(1, (0, 300), 15, 0)):
            self.assertEqual(self.plan([target]).status, 'clear')

    def test_right_blocked_uses_safe_port_turn(self):
        targets = [Target(1, (0, 600), 10, 180), Target(2, (90, 100), 0, 0)]
        result = self.plan(targets)
        self.assertEqual(result.status, 'avoiding')
        self.assertGreater(result.action.course_deg, 180)
        self.assertGreater(verify(self.own, targets, result.action, self.config), 0)

    def test_initial_overlap_no_safe_solution(self):
        result = self.plan([Target(1, (0, 30), 0, 0), Target(2, (0, -30), 10, 0)])
        self.assertEqual(result.status, 'blocked')
        self.assertIsNone(result.action)
        self.assertIsNotNone(result.emergency_action)
        self.assertFalse(result.emergency_assessment.safe)
        self.assertLess(result.emergency_assessment.min_clearance_m, 0)

    def test_zero_speed_can_be_hit_by_incoming_target(self):
        own = Vessel((0, 0), 0, 0)
        target = Target(1, (0, 300), 10, 180)
        check = assess_action(own, [target], Action(0, 0), self.config)
        self.assertFalse(check.safe)
        self.assertAlmostEqual(check.risks[0].closest_time_s, 30.)
        self.assertLess(check.risks[0].closest_distance_m, 1e-8)
        result = self.plan([target], own, Action(0, 0))
        self.assertEqual(result.status, 'blocked')
        self.assertIsNone(result.action)
        self.assertFalse(result.emergency_assessment.safe)

    def test_age_compensation(self):
        target = Target(1, (0, 320), 10, 180, 2.)
        check = assess_action(Vessel((0, 0), 0, 0), [target], Action(0, 0), self.config)
        self.assertAlmostEqual(check.risks[0].closest_time_s, 30.)

    def test_uncertainty_increases_with_age(self):
        own = Vessel((0, 0), 0, 0)
        check = lambda age: assess_action(own, [Target(1, (200, 0), 0, None, age)], Action(0, 0), self.config)
        self.assertLess(check(3).min_clearance_m, check(0).min_clearance_m)

    def test_unknown_and_stale_targets_are_not_silently_ignored(self):
        for target in (Target(1, (0, 500), 10, 180, 6), Target(1, (0, 500)),
                       Target(1, None), Target(1, (0, 500), 10, None)):
            with self.subTest(target=target):
                result = self.plan([target])
                self.assertEqual(result.status, 'insufficient_data')
                self.assertIsNone(result.action)
                self.assertIn(1, result.assessment.uncertain_target_ids)
                self.assertTrue(result.assessment.diagnostics)
        distant = self.plan([Target(1, (100000, 0), None, None, 6)])
        self.assertEqual(distant.status, 'clear')
        self.assertTrue(distant.assessment.diagnostics)

    def test_unknown_fast_target_envelope_uses_reported_speed_bound(self):
        target = Target(1, (5000, 0), 200, None)
        result = self.plan([target])
        self.assertEqual(result.status, 'insufficient_data')

    def test_between_sample_collision(self):
        config = replace(self.config, uncertainty_mps=0.)
        path = Trajectory((0., 1.), ((-200., 0.), (200., 0.)), 1.)
        check = assess_trajectory(path, [Target(1, (0., 0.), 0., 0.)], config)
        self.assertFalse(check.safe)
        self.assertAlmostEqual(check.risks[0].closest_time_s, .5)
        self.assertEqual(check.risks[0].closest_distance_m, 0.)

    def test_braking_tail_remains_time_aligned_and_extends_horizon(self):
        config = replace(self.config, horizon_s=1., obstacle_radius_m=1.,
                         vessel_length_m=.1, vessel_width_m=.1, uncertainty_mps=0.)
        own = Vessel((0, 0), 10, 0)
        path = forecast(own, Action(10, 0), config)
        self.assertGreater(path.times_s[-1], 1+10/config.deceleration_mps2-.001)
        # Target crosses the braking path only after the one-second hold.
        targets = [Target(1, (0, 30), 0, 0)]
        hold_end = path.times_s.index(path.braking_start_s)+1
        hold = Trajectory(path.times_s[:hold_end], path.positions_m[:hold_end], 1.)
        self.assertTrue(assess_trajectory(hold, targets, config).safe)
        check = assess_action(own, targets, Action(10, 0), config)
        self.assertFalse(check.safe)
        self.assertEqual(len(path.times_s), len(path.positions_m))
        self.assertTrue(all(b > a for a, b in zip(path.times_s, path.times_s[1:])))

    def test_spatial_crossing_at_different_times_is_safe(self):
        config = replace(self.config, uncertainty_mps=0.)
        # Both cross (0, 300), but own ship arrives at t=30 and target t=5.
        target = Target(1, (-50, 300), 10, 90)
        self.assertTrue(assess_action(self.own, [target], self.desired, config).safe)

    def test_moving_targets_collide_between_samples(self):
        config = replace(self.config, uncertainty_mps=0.)
        path = Trajectory((0., 1.), ((0., 0.), (0., 0.)), 1.)
        check = assess_trajectory(path, [Target(1, (-200., 0.), 400., 90.)], config)
        self.assertFalse(check.safe)
        self.assertAlmostEqual(check.risks[0].closest_time_s, .5)

    def test_slow_candidates_and_zero_are_available(self):
        candidates = DynamicAvoidancePlanner(self.config)._candidates(self.own, self.desired)
        self.assertIn(Action(0., 0.), candidates)
        self.assertTrue(any(0 < a.speed_mps < 10*1852/3600 for a in candidates))

    def test_many_targets_selected_action_is_safe_against_all(self):
        targets = [Target(1, (0, 600), 10, 180), Target(2, (90, 100), 0, 0),
                   Target(3, (-600, 300), 3, 90)]
        result = self.plan(targets)
        self.assertEqual(result.status, 'avoiding')
        self.assertEqual(len(result.assessment.risks), 3)
        self.assertGreater(verify(self.own, targets, result.action, self.config), 0)

    def test_turn_and_speed_transition_boundaries_match_independent_simulation(self):
        own = Vessel((4, 5), 7, 359)
        action = Action(11, 14)
        path = forecast(own, action, self.config)
        independent, _ = advance(own, action, path.braking_start_s, self.config)
        index = path.times_s.index(path.braking_start_s)
        self.assertLess(math.dist(independent.position_m, path.positions_m[index]), .01)
        self.assertGreater(path.positions_m[index][0], own.position_m[0])
        self.assertEqual(Action(2, 361).course_deg, 1)

    def test_adapter_units_origin_and_own_age(self):
        records = [dict(platformId=1, longitude=120., latitude=30., targetSpeed=10., targetCourse=90., _received_age_s=2.),
                   dict(platformId=2, longitude=120.01, latitude=30., targetSpeed=5., targetCourse=270., _received_age_s=1.)]
        own, targets = scene_from_situation(records, origin_lon_lat=(120, 30), speed_unit='knots', own_id=1)
        self.assertAlmostEqual(own.speed_mps, 10*1852/3600)
        self.assertAlmostEqual(own.position_m[0], own.speed_mps*2)
        self.assertAlmostEqual(targets[0].speed_mps, 5*1852/3600)
        self.assertEqual(targets[0].age_s, 1)
        _, metric = scene_from_situation(records, origin_lon_lat=(120, 30), speed_unit='mps', own_id=1)
        self.assertEqual(metric[0].speed_mps, 5)
        records[0]['_received_age_s'] = 6
        with self.assertRaises(ValueError):
            scene_from_situation(records, origin_lon_lat=(120, 30), speed_unit='mps', own_id=1)

    def test_adapter_retains_invalid_target_and_rejects_invalid_ids(self):
        own = dict(platformId=1, longitude=0, latitude=0, targetSpeed=0, targetCourse=0)
        bad = dict(platformId=2, longitude=math.nan, latitude=0, targetSpeed=-1)
        boat, targets = scene_from_situation([own, bad], origin_lon_lat=(0, 0), speed_unit='mps', own_id=1)
        self.assertEqual(self.plan(targets, boat).status, 'insufficient_data')
        with self.assertRaises(ValueError):
            scene_from_situation([own, {}], origin_lon_lat=(0, 0), speed_unit='mps', own_id=1)

    def test_closed_loop_and_reset(self):
        planner = DynamicAvoidancePlanner(self.config)
        own, targets = self.own, [Target(1, (0, 600), 10, 180)]
        directions = []
        for _ in range(40):
            result = planner.plan(own, targets, self.desired)
            self.assertIsNotNone(result.action)
            if result.status == 'avoiding':
                turn = (result.action.course_deg-own.course_deg+180) % 360-180
                if abs(turn) > .01:
                    directions.append(math.copysign(1, turn))
            own, trace = advance(own, result.action, 1., self.config)
            self.assertGreater(min(math.dist((x, y), shifted(targets[0], t).position_m)
                                   for t, x, y in trace), self.config.safe_distance_m)
            targets = [shifted(targets[0], 1.)]
        self.assertTrue(directions)
        self.assertEqual(set(directions), {1.})
        planner.reset()
        fresh = planner.plan(self.own, [Target(1, (0, 600), 10, 180)], self.desired)
        self.assertEqual(fresh.action, self.plan([Target(1, (0, 600), 10, 180)]).action)

    def test_duplicate_target_ids_rejected(self):
        target = Target(1, (0, 600), 10, 180)
        with self.assertRaises(ValueError):
            self.plan([target, target])


if __name__ == '__main__':
    unittest.main()
