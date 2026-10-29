"""Measured controller-response replay and physical-envelope safety regressions."""
import csv
from dataclasses import replace
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace
from server.policies.static_obstacles import motion_clearance, StaticObstacleConfig

from tools.acceleration_response import measured_speed_profile, response_points
from tools.navigation_motion import predict_ground_motion
from tools.vessel_dynamics import MAX_SPEED_MPS, motion_parameters
from tools.waypoint_geometry import KNOT_MPS
from server.experiments.calibrate_acceleration import calibrate
from server.experiments.dynamic_avoidance import advance as dense_advance
from server.experiments.simulate import advance
from server.navigation.dynamic_avoidance import Action, AvoidanceConfig, Target, Vessel, assess_action, forecast
from server.navigation.dynamic_avoidance.prediction import assess_trajectory
from server.navigation.dynamic_avoidance.kev import check_models
from server.policies.kev_policy import KevPolicy
from server.jev_policy import JevPolicy
from server import decision_service


class AccelerationResponseTests(unittest.TestCase):
    def motion(self, speed=0., target=MAX_SPEED_MPS, duration=1., course=90., initial_course=90., rate=0., **kwargs):
        return predict_ground_motion(speed,initial_course,target,course,0.,0.,duration,
            initial_turn_rate_deg_s=rate,acceleration_response='measured_0_50kn',**kwargs)

    def test_replay_observed_milestones_and_distance(self):
        initial=.227796014*KNOT_MPS
        response,boundaries=measured_speed_profile(initial,MAX_SPEED_MPS,1.46)
        for knots,seconds in ((10.,6.0270854287),(20.,10.8477633857),(30.,14.7207935476),(40.,19.4944644218),(50.,35.5215053165)):
            with self.subTest(knots=knots):
                self.assertAlmostEqual(response(seconds)/KNOT_MPS,knots,places=6)
        self.assertAlmostEqual(boundaries[-1],35.5215053165,places=7)
        motion=self.motion(speed=initial,duration=35.6)
        self.assertEqual(motion['end_speed'],MAX_SPEED_MPS)
        # Raw position path: 584.16m; raw speed integral: 582.62m.
        self.assertLess(abs(motion['endpoint'][0]-582.6158809123),2.)

    def test_repeated_commands_preserve_acceleration_progress(self):
        initial=.227796014*KNOT_MPS
        continuous=self.motion(speed=initial,duration=35.6)
        own=Vessel((0.,0.),initial,90.)
        config=AvoidanceConfig()
        for _ in range(356):
            own,_=dense_advance(own,Action(MAX_SPEED_MPS,90.),.1,config)
        self.assertAlmostEqual(own.speed_mps,MAX_SPEED_MPS,places=7)
        self.assertLess(math.dist(own.position_m,continuous['endpoint']),.01)

    def test_hard_cap_and_capability_curve_still_apply(self):
        motion=self.motion(duration=50.,acceleration_mps2=.8,acceleration_curve=((0.,.8),(MAX_SPEED_MPS,.6)))
        for i,(begin,end) in enumerate(zip(motion['times_s'],motion['times_s'][1:])):
            acceleration=(motion['path_speeds'][i+1]-motion['path_speeds'][i])/(end-begin)
            self.assertLessEqual(acceleration,.8+1e-8)
        self.assertLessEqual(motion['end_speed'],MAX_SPEED_MPS)
        self.assertGreater(motion['speed_transition_time_s'],35.5)

    def test_other_setpoints_turns_braking_and_unknown_heading_keep_bounds(self):
        for kwargs in (dict(target=40*KNOT_MPS),dict(course=100.),dict(rate=1.),
                       dict(initial_course=None),dict(speed=10.,target=0.)):
            with self.subTest(kwargs=kwargs):
                result=self.motion(**kwargs)
                self.assertEqual(result['acceleration_response_used'],'bounded')
        self.assertAlmostEqual(self.motion(target=40*KNOT_MPS)['end_speed'],1.46)
        self.assertAlmostEqual(self.motion(speed=10.,target=0.)['end_speed'],8.25)

    def test_two_decimal_si_target_selects_measured_response(self):
        motion=self.motion(target=round(MAX_SPEED_MPS,2))
        self.assertEqual(motion['acceleration_response_used'],'measured_0_50kn')
        self.assertLess(motion['end_speed'],1.46)

    def test_calibration_is_a_response_and_contains_no_location(self):
        data=json.loads(Path(__file__).resolve().parents[2].joinpath('jev_api/tools/data/acceleration_0_50kn.json').read_text())
        self.assertEqual(data['kind'],'observed_controller_response')
        self.assertEqual(data['source_samples'],357)
        self.assertEqual(len(data['source_sha256']),64)
        self.assertLessEqual(data['maximum_segment_acceleration_mps2'],1.46)
        self.assertEqual(response_points()[0][1],0.)
        self.assertNotIn('latitude',json.dumps(data))
        self.assertNotIn('longitude',json.dumps(data))

    def safety_fixture(self):
        config=replace(AvoidanceConfig(),horizon_s=4.,obstacle_radius_m=.1,vessel_length_m=.1,
                       vessel_width_m=.1,uncertainty_mps=0.,clearance_epsilon_m=.01)
        own=Vessel((0.,0.),0.,90.)
        action=Action(MAX_SPEED_MPS,90.)
        nominal_x=self.motion(duration=4.)['endpoint'][0]
        physical_x=.5*1.46*4**2
        targets=(Target(1,(physical_x,-4.),1.,0.),Target(2,(nominal_x,-4.),1.,0.))
        return own,action,config,targets

    def test_dynamic_guard_rejects_risk_visible_only_under_faster_response(self):
        own,action,config,targets=self.safety_fixture()
        target=(targets[0],)
        self.assertTrue(assess_trajectory(forecast(own,action,config),target,config).safe)
        self.assertFalse(assess_action(own,target,action,config).safe)
        self.assertFalse(check_models((own,target),action,config,.5).safe)

    def test_dynamic_guard_rejects_risk_visible_only_under_measured_response(self):
        own,action,config,targets=self.safety_fixture()
        bounded=replace(config,acceleration_response='bounded')
        self.assertTrue(assess_action(own,(targets[1],),action,bounded).safe)
        self.assertFalse(assess_action(own,(targets[1],),action,config).safe)

    def test_risk_ids_are_merged_across_response_models(self):
        own,action,config,targets=self.safety_fixture()
        result=assess_action(own,targets,action,config)
        self.assertEqual([risk.target_id for risk in result.risks if not risk.safe],[1,2])

    def test_following_waypoint_and_simulator_share_selector(self):
        for policy in (KevPolicy(),JevPolicy()):
            self.assertEqual(motion_parameters(policy.execution)['acceleration_response'],'measured_0_50kn')
        self.assertEqual(KevPolicy().dynamic_config.acceleration_response,'measured_0_50kn')
        x,y,speed,_=advance(0.,0.,0.,90.,dict(speed=50.,course=90.),1.)
        expected=self.motion()
        self.assertAlmostEqual(speed,expected['end_speed'])
        self.assertAlmostEqual(x,expected['endpoint'][0])
        self.assertEqual(y,expected['endpoint'][1])

    def test_service_can_select_bounded_response(self):
        with (tempfile.TemporaryDirectory() as folder,
              patch.dict('os.environ',{},clear=True),
              patch.object(decision_service,'LOCAL_CONFIG_PATH',Path(folder)/'none.json'),
              patch('sys.argv',['decision_service.py','--acceleration-response','bounded']),
              patch.object(decision_service,'start_server') as start):
            decision_service.main()
            policy=start.call_args.args[2]
            self.assertEqual(policy.execution['acceleration_response'],'bounded')
            self.assertEqual(policy.dynamic_config.acceleration_response,'bounded')

    def test_calibration_generator_reads_fractional_time_and_validates_input(self):
        with tempfile.TemporaryDirectory() as folder:
            source=Path(folder)/'trace.csv'
            with source.open('w',encoding='utf-8-sig',newline='') as stream:
                writer=csv.writer(stream)
                writer.writerow(('时间','目标绝对速度','时间'))
                writer.writerows((('01:00:00',.2,'00:00.0'),('01:00:00',25.,'00:20.0'),('01:00:00',50.,'00:40.0')))
            result=calibrate(source)
            self.assertEqual(result['observed_duration_s'],40.)
            self.assertAlmostEqual(result['first_50_knots_time_s'],40.)
            source.write_text('时间,目标绝对速度,时间\n00:00:00,.2,00:00.0\n00:00:01,50,00:00.0\n00:00:02,50,00:01.0\n')
            with self.assertRaises(ValueError):
                calibrate(source)

    def test_static_guard_includes_measured_response_and_full_braking_tail(self):
        geometry=SimpleNamespace(boat_position=(0.,0.))
        boat=dict(targetSpeed=0.,targetCourse=90.)
        obstacles=[dict(id=1,xy=(600.,0.),radius_m=5.)]
        bounded=motion_clearance(geometry,boat,50.,90.,obstacles,dict(acceleration_response='bounded'),1.,StaticObstacleConfig())
        calibrated=motion_clearance(geometry,boat,50.,90.,obstacles,dict(acceleration_response='measured_0_50kn'),1.,StaticObstacleConfig())
        self.assertTrue(bounded['safe'])
        self.assertFalse(calibrated['safe'])
        self.assertEqual(calibrated['obstacle_id'],1)

    def test_prompt_response_selector_matches_bounded_configuration(self):
        from choice_control import build_payload
        from server.tests.test_kev_policy import KevTests
        policy=JevPolicy(acceleration_response='bounded')
        payload=build_payload(policy.observation(KevTests().packet()))
        self.assertEqual(payload['state']['execution']['acceleration_response'],'bounded')
        self.assertIn('execution.acceleration_response',payload['state']['execution']['forecast'])
