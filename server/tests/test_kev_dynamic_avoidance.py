"""Kev applies independent static/dynamic constraints, preserving static routing."""
import copy
from dataclasses import replace
import unittest
from unittest.mock import MagicMock, Mock, patch
from server import decision_service as ds
from server.navigation.dynamic_avoidance import Action, AvoidanceConfig, Vessel, forecast
from server.navigation.dynamic_avoidance.kev import wire_action, check_models
from server.policies.factory import create_policy
from server.policies.kev_policy import KevPolicy
from server.tests.test_static_obstacles import choose_heading, packet, record
from tools.waypoint_geometry import KNOT_MPS


def moving_packet(x=600.,y=0.,speed=20.,course=270.,age=0.):
    p=packet(speed=20.,target=2500.)
    p['situation']=[p['situation'][0],record(2001,x,y,speed=speed,course=course,_received_age_s=age)]
    return p


class KevDynamicAvoidanceTests(unittest.TestCase):
    def assert_safe_options(self,policy,p):
        options=policy.local.last_decision['options']
        self.assertTrue(options)
        self.assertLessEqual(len(options),16)
        for item in options.values():
            speed=item['speed_knots']*(KNOT_MPS if policy.speed_unit=='mps' else 1.)
            args=(p,p['situation'][0],p['situation'],speed,item['absolute_course_deg'])
            self.assertTrue(policy.validate_control(*args)['safe'])
            self.assertTrue(policy.validate_dynamic_control(*args)['safe'])
            self.assertEqual(list(item)[-1],'turn_deg')

    def test_harmless_dynamic_keeps_exact_static_requests(self):
        for distance in (350.,1200.):
            base=packet(speed=50.,obstacle_x=distance,target=2500.)
            first=Mock(side_effect=choose_heading);baseline=KevPolicy(client=first)
            expected=baseline(base)
            for age in (0.,30.):
                p=copy.deepcopy(base)
                p['situation'].append(record(2001,100000,0,speed=20,course=90,_received_age_s=age))
                client=Mock(side_effect=choose_heading);policy=KevPolicy(client=client)
                self.assertEqual(policy(p),expected)
                self.assertEqual(client.call_args.args[0],first.call_args.args[0])
                self.assertEqual(policy.local.last_decision['static_path_xy_m'],baseline.local.last_decision['static_path_xy_m'])

    def test_classification_and_state_transitions(self):
        p=moving_packet();p['situation'].append(record(4001,1000,0,speed=0,_received_age_s=300))
        policy=KevPolicy();obs=policy.observation(p)
        self.assertEqual([t.target_id for t in obs['dynamic_scene'][1]],[2001])
        self.assertEqual([o['id'] for o in obs['static_obstacles']],[4001])
        p['situation'][1]['targetSpeed']=0
        obs=policy.observation(p)
        self.assertIsNone(obs['dynamic_scene'])
        self.assertEqual(len(obs['static_obstacles']),2)
        p['situation'][1]['targetSpeed']=20
        obs=policy.observation(p)
        self.assertEqual([o['id'] for o in obs['static_obstacles']],[4001])
        self.assertEqual([t.target_id for t in obs['dynamic_scene'][1]],[2001])

    def test_head_on_crossings_overtaking(self):
        for x,y,speed,course in ((600,0,20,270),(300,300,20,180),(300,-300,20,0),(200,0,5,90)):
            with self.subTest(x=x,y=y):
                p=moving_packet(x,y,speed,course);original=copy.deepcopy(p)
                policy=KevPolicy(client=choose_heading)
                self.assertGreater(policy(p)[0],0)
                self.assert_safe_options(policy,p)
                self.assertEqual(p,original)

    def test_dynamic_only_does_not_force_half_speed(self):
        p=moving_packet();policy=KevPolicy(client=choose_heading)
        speed,course=policy(p)
        self.assertGreater(speed,0.)
        self.assertEqual(max(c['speed_knots'] for c in policy.local.last_decision['options'].values()),50.)
        self.assertGreater(course,90.)
        decision=policy.local.last_decision
        self.assertAlmostEqual(decision['dynamic_avoidance']['assessment']['min_clearance_m'],
            decision['options'][decision['choice']]['dynamic_clearance_m'],delta=.011)

    def test_far_static_straight_preference_can_yield_without_replanning(self):
        base=packet(speed=50.,obstacle_x=1200.,target=2500.)
        reference=KevPolicy(client=choose_heading);reference(base)
        self.assertEqual(len(reference.local.last_decision['options']),1)
        p=copy.deepcopy(base);p['situation'].append(record(2001,600,0,speed=20,course=270))
        policy=KevPolicy(client=choose_heading)
        self.assertNotEqual(policy(p)[1],90.)
        self.assertEqual(policy.local.last_decision['static_path_xy_m'],reference.local.last_decision['static_path_xy_m'])
        self.assertEqual(policy.local.last_decision['state']['static_avoidance'],reference.local.last_decision['state']['static_avoidance'])
        self.assert_safe_options(policy,p)

    def test_mixed_half_cap_and_blocked_right_side(self):
        for position in ((100,-90),(60,-85)):
            p=moving_packet();p['situation'].append(record(4001,*position,speed=0))
            policy=KevPolicy(client=choose_heading);speed,course=policy(p)
            self.assertGreater(speed,0)
            self.assertLessEqual(speed,25.)
            self.assertTrue(all(c['speed_knots']<=25 for c in policy.local.last_decision['options'].values()))
            if position==(60,-85):self.assertLess(course,90.)
            self.assert_safe_options(policy,p)

    def test_uncertain_wait_recovers_without_model_failure(self):
        for missing in (False,True):
            p=moving_packet(x=250.,age=0. if missing else 6.)
            if missing:p['situation'][1]['targetCourse']=None
            client=Mock(side_effect=choose_heading);policy=KevPolicy(client=client)
            with self.assertLogs('server.policies.kev_policy',level='WARNING'):
                for _ in range(26):self.assertEqual(policy(p)[0],0.)
            client.assert_not_called()
            self.assertEqual(policy.local.dynamic_status,'insufficient_data')
            p['situation'][1]=record(2001,100000,0,speed=20,course=90)
            self.assertGreater(policy(p)[0],0.)
            client.assert_called_once()

    def test_no_common_action_waits(self):
        p=moving_packet(x=10.);client=Mock();policy=KevPolicy(client=client)
        with self.assertLogs('server.policies.kev_policy',level='WARNING'):
            self.assertEqual(policy(p)[0],0.)
        client.assert_not_called()
        self.assertEqual(policy.local.dynamic_status,'blocked')

    def test_separate_guards_and_zero_speed_risk(self):
        p=moving_packet(x=150.);policy=KevPolicy();own=p['situation'][0]
        self.assertTrue(policy.validate_control(p,own,p['situation'],20,90)['safe'])
        self.assertFalse(policy.validate_dynamic_control(p,own,p['situation'],20,90)['safe'])
        own['targetSpeed']=0.
        self.assertFalse(policy.validate_dynamic_control(p,own,p['situation'],0,90)['safe'])

    def test_no_moving_targets_do_not_add_validation_to_static_stop(self):
        p=packet();own=dict(p['situation'][0],targetSpeed=0.,targetCourse=None)
        self.assertTrue(KevPolicy().validate_dynamic_control(p,own,[own,p['situation'][1]],0,90)['safe'])

    def test_invalid_own_motion_is_a_dynamic_veto_not_a_control_exception(self):
        p=moving_packet();own=dict(p['situation'][0],targetSpeed=0.,targetCourse=None)
        check=KevPolicy().validate_dynamic_control(p,own,p['situation'],0,90)
        self.assertFalse(check['safe'])
        self.assertIn('reason',check)

    def test_factory_metric_units_and_task_reset(self):
        policy=create_policy(kev_dynamic_config=replace(AvoidanceConfig(),horizon_s=30.),kev_max_turn_rate_deg_s=5.,duration=.2)
        self.assertEqual(policy.dynamic_config.horizon_s,30.)
        self.assertEqual(policy.dynamic_config.max_turn_rate_deg_s,5.)
        self.assertIsNot(policy.dynamic_config,policy.obstacle_config)
        self.assertEqual(wire_action(25,90,'mps'),Action(round(25*KNOT_MPS,2),90))
        p=moving_packet()
        for r in p['situation']:r['targetSpeed']*=KNOT_MPS
        for w in p['params']['planWaypointList']:w['routeSpeed']*=KNOT_MPS
        metric=KevPolicy(client=choose_heading,speed_unit='mps');metric(p)
        self.assert_safe_options(metric,p)
        old=metric.local.dynamic_guard;p['id']='new';p['situation']=p['situation'][:1]
        metric(p)
        self.assertIsNot(old,metric.local.dynamic_guard)
        self.assertEqual(metric.local.dynamic_status,'clear')

    def test_periodic_sender_rechecks_new_detection_and_zero(self):
        p=moving_packet(x=100000,course=90.)
        session=ds.ControlSession(p,('127.0.0.1',7612));registry=ds.TaskRegistry();registry.add(session)
        store=ds.SituationStore()
        for r in p['situation']:store.update(r)
        guard=KevPolicy();commands=[];checked=[]
        class Policy:
            def __call__(self,request):
                store.update(record(2001,150,0,speed=20,course=270));return 20.,90.
            def validate_control(self,*args):return guard.validate_control(*args)
            def validate_dynamic_control(self,*args):
                checked.append(args[-2]);return guard.validate_dynamic_control(*args)
        def send(payload,remote):
            command=ds.decode_speed_course_packet(payload);commands.append(command)
            if command['speed']==0:
                store.update(record(1010,0,0,speed=0,course=90));session.request_stop()
        context=MagicMock();context.__enter__.return_value=Mock(sendto=send)
        with patch('server.control_runtime.socket.socket',return_value=context):
            ds.run_control_loop(session,Mock(),store,('127.0.0.1',18123),Policy(),.01,3,2.5,.1,registry,3,.5)
        self.assertTrue(commands)
        self.assertTrue(all(c['speed']==0 for c in commands))
        self.assertIn(0.,checked)
        self.assertIn(20.,checked)


    def test_held_command_is_rechecked_while_next_inference_is_pending(self):
        import threading
        p=moving_packet(x=100000,course=90.)
        session=ds.ControlSession(p,('127.0.0.1',7612));registry=ds.TaskRegistry();registry.add(session)
        store=ds.SituationStore()
        for r in p['situation']:store.update(r)
        guard=KevPolicy();commands=[];release=threading.Event();logger=Mock()
        class Policy:
            calls=0
            def __call__(self,request):
                self.calls+=1
                if self.calls>1:release.wait(1.)
                return 20.,90.
            def validate_control(self,*args):return guard.validate_control(*args)
            def validate_dynamic_control(self,*args):return guard.validate_dynamic_control(*args)
        def send(payload,remote):
            command=ds.decode_speed_course_packet(payload);commands.append(command)
            if command['speed']>0:
                store.update(record(2001,150,0,speed=20,course=270))
            else:
                store.update(record(1010,0,0,speed=0,course=90));session.request_stop()
        context=MagicMock();context.__enter__.return_value=Mock(sendto=send)
        try:
            with patch('server.control_runtime.socket.socket',return_value=context),patch.object(ds,'LOGGER',logger):
                ds.run_control_loop(session,Mock(),store,('127.0.0.1',18123),Policy(),.01,0,2.5,.1,registry,3,.5)
        finally:release.set()
        self.assertEqual(commands[0]['speed'],20.)
        self.assertTrue(all(c['speed']==0 for c in commands[1:]))
        self.assertTrue(any('dynamic obstacle control veto' in call.args[0] and call.args[-1]=='hold'
                            for call in logger.warning.call_args_list))


    def test_warning_only_preserves_model_payload_action_and_sender_permission(self):
        p=moving_packet(x=1000.)
        base=copy.deepcopy(p);base['situation']=base['situation'][:1]
        baseline_client=Mock(side_effect=choose_heading)
        baseline=KevPolicy(client=baseline_client,max_speed_knots=20.)
        expected=baseline(base)
        client=Mock(side_effect=choose_heading)
        policy=KevPolicy(client=client,max_speed_knots=20.)
        with self.assertLogs('server.policies.kev_policy',level='WARNING'):
            self.assertEqual(policy(p),expected)
        self.assertEqual(client.call_args.args[0],baseline_client.call_args.args[0])
        self.assertEqual(policy.local.dynamic_status,'clear')
        warning=policy.local.last_decision['dynamic_warning']
        self.assertEqual(warning['warning_horizon_s'],60.)
        self.assertEqual(warning['action_horizon_s'],10.)
        self.assertFalse(warning['assessment']['safe'])
        self.assertGreater(warning['assessment']['risks'][0]['closest_time_s'],10.)
        scene=policy.observation(p)['dynamic_scene']
        self.assertFalse(check_models(scene,wire_action(*expected),policy.dynamic_config,.5,horizon_s=60.).safe)
        check=policy.validate_dynamic_control(p,p['situation'][0],p['situation'],*expected)
        self.assertTrue(check['safe'])
        self.assertEqual(check['action_horizon_s'],10.)

    def test_warning_becomes_avoidance_when_target_enters_action_window(self):
        p=moving_packet(x=1000.);policy=KevPolicy(client=choose_heading,max_speed_knots=20.)
        policy(p)
        self.assertEqual(policy.local.dynamic_status,'clear')
        p['situation'][1]=record(2001,250,0,speed=20,course=270)
        action=policy(p)
        self.assertNotEqual(action,(20.,90.))
        self.assertEqual(policy.local.dynamic_status,'avoiding')
        self.assertFalse(policy.validate_dynamic_control(p,p['situation'][0],p['situation'],20.,90.)['safe'])
        self.assertTrue(policy.validate_dynamic_control(p,p['situation'][0],p['situation'],*action)['safe'])

    def test_window_configuration_and_override_keep_maneuver_and_braking_tail(self):
        for kwargs in ({'action_horizon_s':0.},{'warning_horizon_s':0.},
                       {'action_horizon_s':61.},{'action_horizon_s':float('nan')}):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):AvoidanceConfig(**kwargs)
        config=AvoidanceConfig()
        own=Vessel((0.,0.),50*KNOT_MPS,90.)
        action=Action(own.speed_mps,90.)
        path=forecast(own,action,config,horizon_s=config.action_horizon_s)
        self.assertEqual(path.braking_start_s,10.)
        self.assertAlmostEqual(path.times_s[-1],10.+own.speed_mps/config.deceleration_mps2)
        self.assertEqual(forecast(own,action,config).braking_start_s,60.)
        slow_turn=replace(config,max_turn_rate_deg_s=5.)
        self.assertGreater(forecast(own,Action(own.speed_mps,270.),slow_turn,horizon_s=10.).braking_start_s,10.)
        stopped=Vessel((0.,0.),0.,90.)
        self.assertGreater(forecast(stopped,action,config,horizon_s=10.).braking_start_s,10.)


if __name__=='__main__':unittest.main()
