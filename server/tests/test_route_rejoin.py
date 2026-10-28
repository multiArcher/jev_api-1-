"""Route recovery geometry and Kev integration; no model/network dependency."""
import copy
import math
from types import SimpleNamespace
import unittest

from server.navigation.route_rejoin import RouteRejoin, RouteRejoinConfig, refine_courses
from server.policies.factory import create_policy
from server.policies.kev_policy import KevPolicy
from server.policies.static_obstacles import clear_segment
from server.policies.turning import reachable_courses
from server.tests.test_static_obstacles import choose_heading, packet, record
from server.tests.test_kev_dynamic_avoidance import moving_packet
from tools.waypoint_geometry import EARTH_RADIUS, KNOT_MPS


def geometry(position, points=((2000., 0.),), index=0):
    return SimpleNamespace(boat_position=position, waypoint_positions=points, next_index=index)


class RouteGeometryTests(unittest.TestCase):
    def guide(self, tracker, position, *, points=((2000., 0.),), index=0, speed=10.,
              course=90., obstacles=(), avoiding=False):
        return tracker.guide(geometry(position, points, index), speed, course,
            lambda a, b: clear_segment(a, b, obstacles, 10.), avoiding=avoiding)

    def test_forward_merge_on_both_sides_uses_fixed_first_leg(self):
        for side in (-1, 1):
            route = RouteRejoin((0., 0.));route.activate()
            guide = self.guide(route, (500., side * 100.))
            x, y = guide['guidance_xy_m']
            self.assertAlmostEqual(x, 500. + 100./math.tan(math.radians(30.)))
            self.assertEqual(y, 0.)
            self.assertEqual(guide['segment_start_xy_m'], (0., 0.))
            self.assertAlmostEqual(guide['cross_track_m'], side * 100.)
            next_guide = self.guide(route, (600., side * 50.))
            self.assertEqual(next_guide['segment_start_xy_m'], (0., 0.))

    def test_lookahead_speed_and_minimum(self):
        route = RouteRejoin((0., 0.));route.activate()
        self.assertEqual(self.guide(route, (500., 0.), speed=0.)['guidance_xy_m'], (530., 0.))
        self.assertEqual(self.guide(route, (500., 0.), speed=25.)['guidance_xy_m'], (700., 0.))

    def test_following_requires_three_aligned_cycles_and_remains_enabled(self):
        route = RouteRejoin((0., 0.));route.activate()
        for expected in ('rejoining', 'rejoining', 'following', 'following'):
            self.assertEqual(self.guide(route, (500., 5.))['mode'], expected)
        self.assertEqual(self.guide(route, (600., 15.))['mode'], 'rejoining')
        route.activate()
        self.assertEqual(self.guide(route, (700., 0.), avoiding=True)['mode'], 'avoiding')
        self.assertEqual(self.guide(route, (700., 0.))['mode'], 'rejoining')

    def test_heading_wraparound(self):
        route = RouteRejoin((0., 0.));route.activate()
        for _ in range(3):
            result = self.guide(route, (1., 100.), points=((0., 2000.),), course=359.)
        self.assertEqual(result['mode'], 'following')
        self.assertEqual(result['heading_error_deg'], 1.)

    def test_next_leg_uses_previous_mandatory_waypoint_and_resets_alignment(self):
        route = RouteRejoin((0., 0.));route.activate()
        for _ in range(3):self.guide(route, (500., 0.))
        result = self.guide(route, (2005., 100.), points=((2000., 0.), (2000., 2000.)),
                            index=1, course=0.)
        self.assertEqual(result['segment_start_xy_m'], (2000., 0.))
        self.assertEqual(result['segment_end_xy_m'], (2000., 2000.))
        self.assertEqual(result['mode'], 'rejoining')

    def test_obstructed_merge_scans_forward_then_waits_without_forcing_path(self):
        route = RouteRejoin((0., 0.));route.activate()
        obstacle = dict(xy=(750., 0.), radius_m=86.)
        result = self.guide(route, (500., 150.), obstacles=[obstacle])
        self.assertGreater(result['guidance_xy_m'][0], 1000.)
        self.assertTrue(clear_segment((500., 150.), result['guidance_xy_m'], [obstacle], 10.))
        result = self.guide(route, (500., 100.), obstacles=[dict(xy=(500., 100.), radius_m=86.)])
        self.assertEqual(result['mode'], 'waiting')
        self.assertIsNone(result['guidance_xy_m'])
        self.assertTrue(route.active)
        self.assertIsNotNone(self.guide(route, (500., 100.))['guidance_xy_m'])

    def test_short_leg_endpoint_and_duplicate_waypoints(self):
        route = RouteRejoin((0., 0.));route.activate()
        self.assertEqual(self.guide(route, (95., 20.), points=((100., 0.),))['guidance_xy_m'], (100., 0.))
        result = self.guide(route, (95., 20.), points=((100., 0.), (100., 0.)), index=1)
        self.assertEqual(result['mode'], 'degenerate')
        self.assertIsNone(result['guidance_xy_m'])

    def test_disabled_and_invalid_configuration(self):
        route = RouteRejoin((0., 0.), RouteRejoinConfig(enabled=False));route.activate()
        self.assertIsNone(self.guide(route, (500., 100.)))
        for kwargs in ({'min_lookahead_m':0}, {'lookahead_s':float('nan')},
                       {'merge_angle_deg':90}, {'stable_cycles':0}, {'stable_cycles':1.5}, {'enabled':1}):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):RouteRejoinConfig(**kwargs)

    def test_course_refinement_preserves_count_straight_and_yaw_envelope(self):
        for current, desired in ((93., 90.), (3., 0.), (357., 0.)):
            original = reachable_courses(current, 1., 45.)
            if current != 357.:
                self.assertNotIn(desired, original)
            courses = refine_courses(original, current, 45., (desired, 270.))
            self.assertEqual(len(courses), 16)
            self.assertEqual(len(set(courses)), 16)
            self.assertIn(current, courses)
            self.assertIn(desired, courses)
            self.assertTrue(all(abs((c-current+180)%360-180) <= 45. for c in courses))
            self.assertEqual(original, reachable_courses(current, 1., 45.))


class KevRouteRejoinTests(unittest.TestCase):
    def assert_safe(self, policy, p, command):
        args = (p, p['situation'][0], p['situation'], *command)
        self.assertTrue(policy.validate_control(*args)['safe'])
        self.assertTrue(policy.validate_dynamic_control(*args)['safe'])

    def test_static_exit_rejoins_line_at_normal_speed_and_persists_without_nearby_obstacles(self):
        p = packet(speed=50., obstacle_x=350., target=3500.)
        policy = KevPolicy(client=choose_heading)
        self.assertEqual(policy(p)[0], 25.)
        initial = policy.local.route_rejoin.start_xy_m
        p['situation'][0] = record(1010, 600., 130., speed=25., course=90.)
        command = policy(p)
        state = policy.local.last_decision['state']
        self.assertIn('route_rejoin', state)
        self.assertEqual(state['route_rejoin']['mode'], 'rejoining')
        self.assertEqual(state['target_xy_m'][1], 0.)
        self.assertLess(state['target_xy_m'][0], -2500.)
        self.assertEqual(command[0], 50.)
        self.assert_safe(policy, p, command)
        p['situation'][0] = record(1010, 1300., 30., speed=50., course=90.)
        p['situation'] = p['situation'][:1]
        command = policy(p)
        self.assertEqual(policy.local.route_rejoin.start_xy_m, initial)
        self.assertNotIn('static_avoidance', policy.local.last_decision['state'])
        self.assertEqual(policy.local.last_decision['state']['route_rejoin']['mode'], 'rejoining')
        self.assert_safe(policy, p, command)

    def test_dynamic_exit_rejoins_and_wait_also_latches_recovery(self):
        for x in (600., 10.):
            p = moving_packet(x=x)
            policy = KevPolicy(client=choose_heading);policy(p)
            self.assertTrue(policy.local.route_rejoin.active)
            self.assertIn(policy.local.dynamic_status, ('avoiding', 'blocked'))
            p['situation'] = [record(1010, 300., -100., speed=20., course=90.)]
            command = policy(p)
            state = policy.local.last_decision['state']
            self.assertEqual(state['route_rejoin']['mode'], 'rejoining')
            self.assertEqual(state['target_xy_m'][1], 0.)
            self.assertEqual(policy.local.dynamic_status, 'clear')
            self.assert_safe(policy, p, command)

    def test_approach_and_warning_only_do_not_activate(self):
        cases = ((packet(speed=50., obstacle_x=1200., target=3500.), 50.),
                 (moving_packet(x=1000.), 20.))
        for p, max_speed in cases:
            policy = KevPolicy(client=choose_heading, max_speed_knots=max_speed)
            policy(p)
            self.assertFalse(policy.local.route_rejoin.active)
            self.assertNotIn('route_rejoin', policy.local.last_decision['state'])

    def test_static_detour_overrides_active_rejoin_and_retains_half_cap(self):
        p = packet(speed=50., obstacle_x=350., target=3500.)
        policy = KevPolicy(client=choose_heading);policy(p)
        p['situation'][0] = record(1010, 500., 0., speed=50., course=90.)
        p['situation'][1] = record(4001, 800., 0.)
        command = policy(p)
        decision = policy.local.last_decision
        self.assertEqual(decision['state']['route_rejoin']['mode'], 'avoiding')
        self.assertTrue(decision['state']['static_avoidance']['detour'])
        self.assertEqual(command[0], 25.)
        self.assertEqual(decision['state']['target_xy_m'], [round(v, 2) for v in decision['static_path_xy_m'][0]])
        self.assert_safe(policy, p, command)

    def test_moving_merge_target_does_not_reset_progress_and_detects_stall(self):
        p = packet(speed=20., obstacle_x=350., target=3500.)
        policy = KevPolicy(client=choose_heading);policy(p)
        p['situation'] = [record(1010, 700., 100., speed=20., course=90.)]
        goals = []
        for x in (700., 710., 720., 730.):
            p['situation'][0] = record(1010, x, 100., speed=20., course=90.)
            policy(p)
            goals.append(policy.local.last_decision['state']['target_xy_m'][0])
        self.assertEqual(len(set(goals)), 4)
        self.assertEqual(len(policy.local.supervisor.samples), 4)
        self.assertEqual(policy.local.supervisor.index, (0, 'route'))
        # The route preference must not conceal a truly stalled boat.
        for _ in range(7):policy(p)
        self.assertEqual(policy.local.supervisor.mode, 'recovery')

    def test_task_reset_factory_disable_and_metric_units(self):
        config = RouteRejoinConfig(enabled=False)
        disabled = create_policy(client=choose_heading, kev_route_rejoin_config=config)
        p = packet(speed=50., obstacle_x=350., target=3500.)
        disabled(p)
        self.assertIs(disabled.route_rejoin_config, config)
        self.assertFalse(disabled.local.route_rejoin.active)
        policy = KevPolicy(client=choose_heading);policy(p)
        previous = policy.local.route_rejoin
        p['id'] = 'replacement';p['situation'] = [record(1010, 600., 100., speed=20.)]
        policy(p)
        self.assertIsNot(policy.local.route_rejoin, previous)
        self.assertFalse(policy.local.route_rejoin.active)
        self.assertAlmostEqual(policy.local.route_rejoin.start_xy_m[1], 100.)
        metric_p = packet(speed=50., obstacle_x=350., target=3500.)
        for r in metric_p['situation']:r['targetSpeed'] *= KNOT_MPS
        for w in metric_p['params']['planWaypointList']:w['routeSpeed'] *= KNOT_MPS
        metric = KevPolicy(client=choose_heading, speed_unit='mps');metric(metric_p)
        metric_p['situation'] = [record(1010, 700., 100., speed=20.*KNOT_MPS, course=90.)]
        command = metric(metric_p)
        self.assertEqual(metric.local.last_decision['state']['route_rejoin']['mode'], 'rejoining')
        self.assert_safe(metric, metric_p, command)

    def test_following_does_not_oversteer_for_small_error_when_slowing(self):
        p = packet(speed=50., obstacle_x=350., target=3500.)
        policy = KevPolicy(client=choose_heading);policy(p)
        p['situation'] = [record(1010, 3360., 4., speed=10., course=93.)]
        for _ in range(3):command = policy(p)
        state = policy.local.last_decision['state']
        self.assertEqual(state['route_rejoin']['mode'], 'following')
        self.assertEqual(command[1], 90.)
        for item in policy.local.last_decision['options'].values():
            self.assertLessEqual(item['route_heading_error_deg'], 5.)
            self.assertLessEqual(item['route_cross_track_after_m'], 10.)
            self.assertEqual(list(item)[-1], 'turn_deg')

    def test_new_dynamic_risk_can_override_rejoin_and_all_options_pass_both_guards(self):
        p = packet(speed=50., obstacle_x=350., target=3500.)
        policy = KevPolicy(client=choose_heading);policy(p)
        p['situation'][0] = record(1010, 600., 130., speed=25., course=90.)
        p['situation'].append(record(2001, 1000., 0., speed=20., course=270.))
        command = policy(p)
        decision = policy.local.last_decision
        self.assertEqual(policy.local.dynamic_status, 'avoiding')
        self.assertEqual(decision['state']['route_rejoin']['mode'], 'avoiding')
        for item in decision['options'].values():
            self.assert_safe(policy, p, (item['speed_knots'], item['absolute_course_deg']))
        self.assert_safe(policy, p, command)

    def test_next_waypoint_references_previous_center_and_finishing_stays_stopped(self):
        p = packet(speed=20., obstacle_x=350., target=1000.)
        p['params']['planWaypointList'].append(dict(routeLon=math.degrees(3000./EARTH_RADIUS),
            routeLat=0., routeSpeed=50., routeRadius=0.))
        p['params']['planWaypointNum'] = 2
        original = copy.deepcopy(p)
        policy = KevPolicy(client=choose_heading);policy(p)
        self.assertEqual(p, original)
        p['situation'] = [record(1010, 1000., 50., speed=10., course=90.)]
        policy(p)
        state = policy.local.last_decision['state']
        self.assertEqual(state['current_waypoint_index'], 1)
        self.assertEqual(state['route_rejoin']['segment_start_xy_m'], [0., 0.])
        self.assertEqual(state['route_rejoin']['segment_end_xy_m'], [2000., 0.])
        p['situation'] = [record(1010, 2950., 0., speed=10., course=90.)]
        self.assertEqual(policy(p)[0], 0.)
        p['situation'] = [record(1010, 3100., 100., speed=10., course=90.)]
        self.assertEqual(policy(p)[0], 0.)


if __name__ == '__main__':
    unittest.main()
