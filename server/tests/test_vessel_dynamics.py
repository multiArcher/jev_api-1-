"""Physical constraints and cross-feature regressions without external assets."""
from dataclasses import replace
import math
import random
import unittest
from unittest.mock import patch

from tools.navigation_motion import predict_ground_motion
from tools.waypoint_geometry import KNOT_MPS
from tools.vessel_dynamics import MAX_SPEED_MPS, turn_profile, curve_limit
from server.navigation.dynamic_avoidance import Vessel, Action, AvoidanceConfig, forecast
from server.navigation.target_following import forecast as following_forecast
from server.policies.kev_policy import KevPolicy
from server.jev_policy import JevPolicy
from server.decision_service import SituationStore
from server.experiments.simulate import advance


class VesselDynamicsTests(unittest.TestCase):
    def motion(self, speed=0., target=MAX_SPEED_MPS, heading=0., duration=1., **kwargs):
        return predict_ground_motion(speed, 0., target, heading, .001, .001, duration, **kwargs)

    def assert_bounds(self, motion):
        self.assertEqual(len(motion['path']),len(motion['times_s']))
        self.assertTrue(all(abs(w) <= 12.+1e-9 for w in motion['path_turn_rates_deg_s']))
        for i,(begin,end) in enumerate(zip(motion['times_s'],motion['times_s'][1:])):
            dv=(motion['path_speeds'][i+1]-motion['path_speeds'][i])/(end-begin)
            dw=(motion['path_turn_rates_deg_s'][i+1]-motion['path_turn_rates_deg_s'][i])/(end-begin)
            self.assertLessEqual(dv,1.46+1e-8)
            self.assertGreaterEqual(dv,-1.75-1e-8)
            self.assertLessEqual(abs(dw),4.+1e-8)

    def test_startup_and_speed_cap_cannot_be_bypassed_by_transition_time(self):
        for duration in (.2,1.,25.):
            motion=self.motion(target=100.,duration=duration)
            self.assertAlmostEqual(motion['end_speed'],min(1.46*duration,MAX_SPEED_MPS))
            self.assert_bounds(motion)

    def test_fifty_knot_stop_time_and_distance(self):
        motion=self.motion(speed=MAX_SPEED_MPS,target=0.,duration=MAX_SPEED_MPS/1.75)
        self.assertAlmostEqual(motion['end_speed'],0.)
        self.assertAlmostEqual(motion['endpoint'][1],MAX_SPEED_MPS**2/(2*1.75),places=6)
        self.assert_bounds(motion)

    def test_yaw_accelerates_caps_and_settles_without_snapping(self):
        motion=self.motion(speed=5.,target=5.,heading=90.,duration=12.)
        self.assertEqual(motion['end_course_deg'],90.)
        self.assertEqual(motion['end_turn_rate_deg_s'],0.)
        self.assertAlmostEqual(max(motion['path_turn_rates_deg_s']),12.)
        self.assertAlmostEqual(self.motion(speed=5.,target=5.,heading=90.)['end_course_deg'],2.)
        self.assert_bounds(motion)

    def test_command_reversal_and_emergency_stop_keep_angular_momentum(self):
        for heading,target in ((270.,5.),(0.,0.)):
            motion=self.motion(speed=5.,target=target,heading=heading,duration=10.,initial_turn_rate_deg_s=12.)
            self.assertEqual(motion['path_turn_rates_deg_s'][0],12.)
            self.assertGreater(motion['path_courses_deg'][1],0.)
            self.assert_bounds(motion)
        stopped=self.motion(speed=5.,target=0.,duration=5.,initial_turn_rate_deg_s=12.)
        self.assertAlmostEqual(stopped['end_course_deg'],18.)

    def test_random_turn_profiles_are_continuous_and_bounded(self):
        rng=random.Random(28)
        for _ in range(80):
            delta,rate=rng.uniform(-180,180),rng.uniform(-12,12)
            sample,boundaries=turn_profile(delta,rate)
            for t in boundaries:
                before,after=sample(max(0.,t-1e-6)),sample(t+1e-6)
                self.assertLess(abs(after[0]-before[0]),.00003)
                self.assertLess(abs(after[1]-before[1]),.00001)
            self.assertAlmostEqual(sample(200)[0],delta)
            self.assertEqual(sample(200)[1],0.)

    def test_speed_curves_slow_response_and_extend_braking_tail(self):
        curve=((0.,1.46),(MAX_SPEED_MPS,.5))
        fixed=self.motion(duration=10.)
        curved=self.motion(duration=10.,acceleration_curve=curve)
        self.assertLess(curved['end_speed'],fixed['end_speed'])
        self.assert_bounds(curved)
        for i,(begin,end) in enumerate(zip(curved['times_s'],curved['times_s'][1:])):
            acceleration=(curved['path_speeds'][i+1]-curved['path_speeds'][i])/(end-begin)
            self.assertLessEqual(acceleration,curve_limit(curve,curved['path_speeds'][i+1],1.46)+1e-8)
        config=replace(AvoidanceConfig(),deceleration_curve=((0.,.5),(MAX_SPEED_MPS,.5)))
        path=forecast(Vessel((0.,0.),10.,0.),Action(10.,0.),config,horizon_s=1.)
        self.assertAlmostEqual(path.times_s[-1],21.)
        self.assertAlmostEqual(path.positions_m[-1][1],110.,places=5)

    def test_turn_speed_curve_changes_angular_response(self):
        motion=self.motion(speed=10.,target=10.,heading=90.,duration=20.,turn_rate_curve=((0.,12.),(10.,6.)))
        self.assertLessEqual(max(motion['path_turn_rates_deg_s']),6.+1e-9)
        self.assert_bounds(motion)

    def test_invalid_limits_and_curves_are_rejected(self):
        for kwargs in ({'acceleration_mps2':2.}, {'deceleration_mps2':2.},
                       {'max_turn_rate_deg_s':45.}, {'max_turn_acceleration_deg_s2':5.},
                       {'acceleration_curve':((10.,1.),(0.,1.))},
                       {'turn_rate_curve':((0.,20.),)}):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):
                self.motion(**kwargs)
        with self.assertRaises(ValueError):
            JevPolicy(max_speed_knots=51.)

    def test_following_simulation_and_dynamic_forecast_agree(self):
        policy=KevPolicy()
        own=dict(targetSpeed=10.,targetCourse=359.,_turn_rate_deg_s=3.)
        expected=predict_ground_motion(10.*KNOT_MPS,359.,12.*KNOT_MPS,20.,0.,0.,1.,dt=.1,initial_turn_rate_deg_s=3.)
        following=following_forecast(policy,own,12.,20.,1.)
        self.assertEqual(following['times_s'],expected['times_s'])
        self.assertEqual(following['endpoint'],expected['endpoint'])
        x,y,v,c,w=advance(0.,0.,10.*KNOT_MPS,359.,dict(speed=12.,course=20.),1.,initial_turn_rate_deg_s=3.,return_turn_rate=True)
        self.assertLess(math.dist((x,y),expected['endpoint']),.0001)
        self.assertEqual(w,expected['end_turn_rate_deg_s'])

    def test_telemetry_rotation_estimate_wraps_north(self):
        store=SituationStore()
        with patch('server.decision_service.time.monotonic',side_effect=[10.,11.,11.,11.]):
            store.update(dict(platformId=1,targetCourse=359.))
            store.update(dict(platformId=1,targetCourse=1.))
            self.assertEqual(store.get(1,2.)['_turn_rate_deg_s'],2.)
            self.assertEqual(store.query_state(include_age=True)[0]['_turn_rate_deg_s'],2.)

    def test_sustained_dynamic_turn_avoids_head_on_under_new_limits(self):
        from server.experiments.kev_dynamic_avoidance import run_scene
        from server.navigation.dynamic_avoidance import Target
        result=run_scene('bounded_head_on',(Target(2001,(600.,0.),20*KNOT_MPS,270.),),
                         steps=50,mock=True,response='physical')
        self.assertGreater(result['min_clearance_m']['dynamic'],0.)
        samples=result['samples']
        # A target beyond one cycle is allowed; actual rotation stays bounded.
        self.assertTrue(any(abs((row['command']['course_deg']-row['own']['course_deg']+180)%360-180)>12.01 for row in samples))
        for before,after in zip(samples,samples[1:]):
            self.assertLessEqual(abs(after['own']['turn_rate_deg_s']),12.+1e-9)
            self.assertLessEqual(abs(after['own']['turn_rate_deg_s']-before['own']['turn_rate_deg_s']),4.+1e-8)
            acceleration=after['own']['speed_mps']-before['own']['speed_mps']
            self.assertLessEqual(acceleration,1.46+1e-8)
            self.assertGreaterEqual(acceleration,-1.75-1e-8)

    def test_curves_propagate_to_all_forecasts(self):
        curves=dict(acceleration_curve=((0.,1.),(25.,.5)),deceleration_curve=((0.,.8),),turn_rate_curve=((0.,8.),))
        policy=KevPolicy(motion_curves=curves)
        remote=JevPolicy(motion_curves=curves)
        for key,value in curves.items():
            self.assertEqual(policy.execution[key],value)
            self.assertEqual(remote.execution[key],value)
            self.assertEqual(getattr(policy.dynamic_config,key),value)
        config=replace(AvoidanceConfig(),acceleration_curve=curves['acceleration_curve'])
        self.assertEqual(KevPolicy(dynamic_config=config).execution['acceleration_curve'],curves['acceleration_curve'])
