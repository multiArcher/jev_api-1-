"""Kev request/action contract, no model downloads or external network calls."""
import sys
import math
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from server.decision_service import build_json_packet, build_situation_packet, decode_situation_packet, service, decode_speed_course_packet, ALGORITHM_TASK_ID
from server.experiments.simulate import scenario, telemetry
from server.policies.factory import create_policy
from server.policies.kev_policy import KEV_SPEEDS_KNOTS, KevPolicy
from tools.waypoint_geometry import KNOT_MPS
from server.policies.turning import reachable_courses
from tools.navigation_motion import predict_ground_motion


def answer(choice):
    return {'answers':{'action':dict(type='choice',choice=choice)}}


def fastest_straight(payload):
    speed = max(payload['state']['speeds_knots'])
    options = payload['questions']['action']['criteria']
    return answer(min((k for k,v in options.items() if v['speed_knots']==speed),
                      key=lambda k: options[k]['turn_deg']))


class KevTests(unittest.TestCase):
    def packet(self):
        packet=scenario('crossing_km')
        packet['situation']=[telemetry(0,0,0,None)]
        return packet

    def test_all_five_speeds_and_sixteen_courses_decode(self):
        packet=self.packet();client=Mock()
        policy=KevPolicy(client=client,initial_course_deg=90.)
        for speed in KEV_SPEEDS_KNOTS:
            heading = 90. if speed == 50. else 70.
            packet['situation'][0]['targetCourse'] = heading
            for i,course in enumerate(reachable_courses(heading,1.,12.)):
                client.return_value=answer(f'speed_{speed:g}__heading_{i:02d}')
                self.assertEqual(policy(packet),(speed,course))
        request=client.call_args.args[0]
        criteria=request['questions']['action']['criteria']
        self.assertEqual(len(criteria),16)
        self.assertTrue(all(v['turn_deg'] == abs(v['signed_turn_deg']) for v in criteria.values()))
        self.assertTrue(all(list(v)[-1] == 'turn_deg' for v in criteria.values()))
        self.assertEqual({v['absolute_course_deg'] for v in criteria.values()}, set(reachable_courses(90.,1.,12.)))
        self.assertEqual(request['state']['speeds_knots'],[50.])
        self.assertEqual(request['state']['max_turn_deg'],12.)
        self.assertEqual(KEV_SPEEDS_KNOTS, (10., 20., 30., 40., 50.))
        self.assertNotIn('forecast_note', request['state'])
        self.assertAlmostEqual(request['state']['target_bearing_deg'],90.)
        self.assertNotIn('sign',request['state'])
        self.assertNotIn('threatLevel',request['state'])

    def test_binary_control_preserves_half_degree_course_and_units(self):
        packet=self.packet();state=packet.pop('situation')
        policy=KevPolicy(client=Mock(return_value=answer('speed_50__heading_01')),initial_course_deg=90.,max_turn_rate_deg_s=5.)
        decoded=decode_speed_course_packet(service(build_json_packet(packet,ALGORITHM_TASK_ID),policy,tuple(state)))
        self.assertEqual(decoded['speed'],50.)
        self.assertEqual(decoded['course'],85.67)
        packet['situation']=state
        policy=KevPolicy(client=Mock(return_value=answer('speed_50__heading_15')),speed_unit='mps',initial_course_deg=90.,max_turn_rate_deg_s=5.)
        speed,course=policy(packet)
        self.assertAlmostEqual(speed,50.*KNOT_MPS)
        self.assertEqual(course,95.)

    def test_recent_history_is_bounded_and_reset_for_new_task(self):
        client=Mock(return_value=answer('speed_50__heading_07'))
        policy=KevPolicy(client=client,initial_course_deg=90.);packet=self.packet()
        for _ in range(7):policy(packet)
        self.assertEqual(len(client.call_args.args[0]['state']['recent_decisions']),4)
        packet['id']='another-task';policy(packet)
        self.assertEqual(client.call_args.args[0]['state']['recent_decisions'],[])

    def test_invalid_or_unavailable_model_does_not_substitute_course(self):
        for response in ({'answers':{}},answer('speed_50__heading_99')):
            with self.assertRaises(ValueError):KevPolicy(client=Mock(return_value=response),initial_course_deg=90.)(self.packet())
        with self.assertRaisesRegex(RuntimeError,'unavailable'):
            KevPolicy(client=Mock(side_effect=RuntimeError('unavailable')),initial_course_deg=90.)(self.packet())

    def test_factory_keeps_kev_http_settings_separate_from_jev(self):
        p=create_policy('kev',kev_endpoint='http://127.0.0.1:19028/v1/systemone',kev_timeout=7,
            endpoint='https://old-jev-endpoint.invalid',model='jev-latest',api_timeout=2)
        self.assertEqual(p.endpoint,'http://127.0.0.1:19028/v1/systemone')
        self.assertEqual(p.timeout,7)

    def test_default_service_uses_kev_and_produces_binary_action(self):
        packet = self.packet()
        state = packet.pop('situation')
        state[0]['targetSpeed'] = 15.
        state[0]['targetCourse'] = 90.
        with patch.object(KevPolicy, '_call', return_value=answer('speed_50__heading_07')) as request:
            result = service(build_json_packet(packet, ALGORITHM_TASK_ID), situation=tuple(state))
        action = decode_speed_course_packet(result)
        self.assertEqual(action['speed'], 50.)
        self.assertEqual(action['course'], 90.)
        self.assertEqual(request.call_args.args[0]['model'], 'kev-latest')

    def test_kev_simulator_requires_zero_speed_command_at_final_arrival(self):
        from server.experiments.simulate import direct, ROUTES, SCENARIOS
        policy=Mock(return_value=(15.,90.));policy.current_route.return_value=0
        # A synthetic final circle already containing the boat isolates arrival semantics.
        with patch.dict(ROUTES, {'arrival_test':[(0,0),(0,0)]}), patch.dict(SCENARIOS, {'arrival_test':dict(radius_m=8.,speed_knots=20.)}), patch('server.experiments.simulate.save') as save, patch('server.experiments.simulate.plot_map'):
            result=direct(policy,'arrival_test',SimpleNamespace(steps=1,dt=1.,backend='kev'),Path('/tmp'))
        self.assertTrue(result['completed'])
        self.assertEqual(result['reached_waypoints'],1)
        self.assertEqual(save.call_args.args[0][-1]['command']['speed'], 0.)

    def test_simulator_arrival_uses_one_hundred_metres_instead_of_turning_radius(self):
        from server.experiments.simulate import direct, ROUTES, SCENARIOS
        for distance, turning_radius, completed in ((99., 1., True), (101., 200., False)):
            with self.subTest(distance=distance, turning_radius=turning_radius):
                policy=Mock(return_value=(50.,90.));policy.current_route.return_value=0
                with patch.dict(ROUTES, {'arrival_test':[(0,0),(distance,0)]}), \
                        patch.dict(SCENARIOS, {'arrival_test':dict(radius_m=turning_radius,speed_knots=50.)}), \
                        patch('server.experiments.simulate.save'), patch('server.experiments.simulate.plot_map'):
                    result=direct(policy,'arrival_test',SimpleNamespace(steps=1,dt=1.,backend='kev',max_turn_rate_deg_s=12.),Path('/tmp'))
                self.assertEqual(result['completed'],completed)

    def test_model_payload_rounds_every_float_including_candidates_and_history(self):
        client=Mock(return_value=answer('speed_50__heading_01'))
        policy=KevPolicy(client=client,duration=.123456,speed_transition=.456789)
        packet=self.packet()
        packet['situation'][0].update(targetSpeed=12.345678,targetCourse=91.234567)
        policy(packet);policy(packet)
        payload=client.call_args.args[0]
        def check(value):
            if isinstance(value,float):
                self.assertEqual(value,round(value,2))
            elif isinstance(value,dict):
                for item in value.values():check(item)
            elif isinstance(value,list):
                for item in value:check(item)
        check(payload)
        self.assertEqual(payload['state']['actual_speed_knots'],12.35)
        self.assertEqual(payload['state']['actual_course_deg'],91.23)
        self.assertEqual(payload['state']['command_duration_s'],.12)
        self.assertTrue(payload['state']['recent_decisions'])
        # Quantization is at the model boundary; the predictor retains configured precision.
        self.assertEqual(policy.duration,.123456)
        self.assertEqual(packet['situation'][0]['targetSpeed'],12.345678)

    def test_reachable_window_wraps_at_north_includes_straight_and_quantizes_inward(self):
        for center,dt in ((359.,1.),(1.,.2),(91.234567,.123456)):
            courses=reachable_courses(center,dt,5.)
            self.assertEqual(len(set(courses)),16)
            self.assertTrue(all(abs((c-center+180)%360-180)<=5*dt+1e-8 for c in courses))
            self.assertIn(round(center,2)%360,courses)
            self.assertTrue(all(c==round(c,2) for c in courses))
        with self.assertRaisesRegex(ValueError,'too narrow'):
            reachable_courses(90.,.001,5.)

    def test_stationary_start_requires_an_explicit_orientation(self):
        with self.assertRaisesRegex(ValueError,'initial-course-deg'):
            KevPolicy(client=Mock())(self.packet())

    def test_stationary_wire_situation_uses_reported_course_including_zero(self):
        for course in (0., 123., 359.):
            with self.subTest(course=course):
                packet = self.packet()
                state = packet.pop('situation')[0]
                state.update(targetSpeed=0., targetCourse=course)
                state = decode_situation_packet(build_situation_packet(state))
                client = Mock(side_effect=fastest_straight)
                policy = KevPolicy(client=client)
                result = service(build_json_packet(packet, ALGORITHM_TASK_ID), policy, (state,))
                action = decode_speed_course_packet(result)
                self.assertEqual(action['speed'], max(client.call_args.args[0]['state']['speeds_knots']))
                self.assertIn(action['course'], [v['absolute_course_deg'] for v in
                              client.call_args.args[0]['questions']['action']['criteria'].values()])
                payload = client.call_args.args[0]
                self.assertEqual(payload['state']['actual_speed_knots'], 0.)
                self.assertEqual(payload['state']['actual_course_deg'], course)

    def test_stopped_boat_uses_latest_situation_instead_of_configured_course(self):
        client = Mock(side_effect=fastest_straight)
        policy = KevPolicy(client=client, initial_course_deg=180.)
        packet = self.packet()
        state = packet['situation'][0]
        state.update(targetSpeed=10., targetCourse=90.)
        policy(packet)
        state.update(targetSpeed=0., targetCourse=230., longitude=120.51, latitude=30.21)
        speed, course = policy(packet)
        payload = client.call_args.args[0]
        self.assertEqual(speed, max(payload['state']['speeds_knots']))
        self.assertIn(course, [v['absolute_course_deg'] for v in payload['questions']['action']['criteria'].values()])
        self.assertEqual(payload['state']['actual_course_deg'], 230.)
        self.assertEqual(policy.observation(packet)['boat']['longitude'], 120.51)
        self.assertEqual(policy.observation(packet)['boat']['latitude'], 30.21)

    def test_invalid_reported_course_is_rejected_even_when_stationary(self):
        for course in (-1, 360, 65535, float('nan'), float('inf'), '90', True):
            with self.subTest(course=course):
                packet = self.packet()
                packet['situation'][0]['targetCourse'] = course
                with self.assertRaisesRegex(ValueError, 'targetCourse'):
                    KevPolicy(client=Mock(), initial_course_deg=90.).observation(packet)

    def test_control_interval_scales_default_turn_range(self):
        client = Mock(return_value=answer('speed_50__heading_00'))
        policy = KevPolicy(client=client, initial_course_deg=90., duration=.2)
        _, course = policy(self.packet())
        self.assertEqual(course, 87.6)
        payload = client.call_args.args[0]
        self.assertEqual(payload['state']['max_turn_deg'], 2.4)
        self.assertEqual(payload['questions']['action']['criteria']['speed_50__heading_15']['absolute_course_deg'], 92.4)

    def test_motion_and_simulation_obey_the_same_rate_limit(self):
        speed=15*KNOT_MPS
        motion=predict_ground_motion(speed,0.,speed,90.,.5,.5,1.,max_turn_rate_deg_s=5.)
        self.assertEqual(motion['end_course_deg'],2.)
        self.assertEqual(motion['end_turn_rate_deg_s'],4.)
        self.assertGreater(motion['endpoint'][0],0.)
        from server.experiments.simulate import advance
        x,y,v,course=advance(0,0,speed,0.,dict(speed=15.,course=90.),1.,max_turn_rate_deg_s=5.)
        self.assertEqual(course,2.)
        self.assertEqual(v,speed)
        self.assertAlmostEqual(x,motion['endpoint'][0])
        self.assertAlmostEqual(y,motion['endpoint'][1])

    def test_transition_fields_removed_and_optional_prompt_fields_do_not_drive_history(self):
        client=Mock(return_value=answer('speed_50__heading_07'))
        policy=KevPolicy(client=client,initial_course_deg=90.)
        original=policy._request
        def stripped(obs):
            payload=original(obs)
            for key in ('distance_to_target_m','current_waypoint_index','arrival_radius_m'):
                payload['state'].pop(key,None)
            return payload
        policy._request=stripped
        policy(self.packet())
        state=client.call_args.args[0]['state']
        self.assertNotIn('speed_transition_s',state)
        self.assertNotIn('course_transition_s',state)
        self.assertEqual(len(policy.local.history),1)

    def test_relative_bearing_and_signed_turn_resolve_side_rear_target(self):
        client=Mock(return_value=answer('speed_10__heading_00'))
        policy=KevPolicy(client=client,initial_course_deg=90.)
        packet=self.packet();packet['params']['currentRoute']=1
        packet['situation']=[telemetry(640,0,15*KNOT_MPS,90.)]
        policy(packet)
        payload=client.call_args.args[0]
        self.assertLess(payload['state']['target_relative_bearing_deg'],-90.)
        self.assertEqual(payload['state']['target_turn_side'],'left')
        self.assertEqual(payload['questions']['action']['criteria']['speed_10__heading_00']['maneuver'],'left')
        self.assertTrue(payload['state']['heading_alignment']['active'])
        options=payload['questions']['action']['criteria']
        self.assertTrue(all(v['heading_gain_deg']>=2. for v in options.values()))
        self.assertTrue(any(v['signed_turn_deg'] < -12. for v in options.values()))
