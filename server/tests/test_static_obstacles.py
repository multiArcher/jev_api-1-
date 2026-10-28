"""Physical clearance, incrementally received targets and waypoint-region regressions."""
import copy
import math
import logging
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, Mock, patch

from server import decision_service as ds
from server.policies.kev_policy import KevPolicy
from server.policies.static_obstacles import (StaticObstacleConfig, StaticNavigator,
    StaticObstacleBlocked, static_targets, clear_segment, motion_clearance, ActualDistanceMonitor,
    configure_actual_distance_log, ACTUAL_DISTANCE_LOGGER)
from server.tests.test_kev_policy import answer
from tools.waypoint_geometry import EARTH_RADIUS, KNOT_MPS, LocalFrame, WaypointGeometry


def record(pid,x,y,speed=0.,course=90.,**extra):
    return dict(platformId=pid,longitude=math.degrees(x/EARTH_RADIUS),
        latitude=math.degrees(y/EARTH_RADIUS),targetSpeed=speed,targetCourse=course,
        sign=1,type=10,height=0,batch=pid,direction=0,**extra)


def packet(x=0.,y=0.,speed=10.,obstacle_x=500.,target=1000.):
    return dict(simId='static-test',id='one',code='trackingMode',params=dict(platCode=1010,
        currentRoute=0,planWaypointNum=1,planWaypointList=[dict(routeLon=math.degrees(target/EARTH_RADIUS),
        routeLat=0.,routeSpeed=50.,routeRadius=0.)]),
        situation=[record(1010,x,y,speed),record(4001,obstacle_x,0.)])


def choose_heading(payload):
    items=payload['questions']['action']['criteria']
    return answer(min(items,key=lambda k:(items[k]['target_heading_error_deg'],items[k]['distance_after_2_steps_m'])))


class StaticObstacleTests(unittest.TestCase):
    def test_actual_distance_only_logs_inside_125_meters(self):
        logger = Mock()
        monitor = ActualDistanceMonitor(logger, packet())
        obstacles = [dict(id=4001, xy=(0., 0.), radius_m=86.04)]
        monitor.update((126., 0.), obstacles, now=0.)
        logger.info.assert_not_called()
        self.assertFalse(monitor.active)
        monitor.update((125., 0.), obstacles, now=1.)
        self.assertIn(4001, monitor.active)
        self.assertIn('actual distance', logger.info.call_args.args[0])
        logger.reset_mock()
        monitor.update((126., 0.), obstacles, now=2.)
        self.assertFalse(monitor.active)
        self.assertEqual(logger.info.call_count, 1)
        self.assertIn('actual summary', logger.info.call_args.args[0])
        logger.reset_mock()
        monitor.update((200., 0.), obstacles, now=3.)
        logger.info.assert_not_called()

    def test_actual_distance_file_is_overwritten_and_does_not_reach_console(self):
        root_handler = Mock(spec=logging.Handler)
        root_handler.level = logging.NOTSET
        root = logging.getLogger()
        root.addHandler(root_handler)
        old_handlers = list(ACTUAL_DISTANCE_LOGGER.handlers)
        old_level = ACTUAL_DISTANCE_LOGGER.level
        ACTUAL_DISTANCE_LOGGER.handlers = []
        try:
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'distance.log'
                path.write_text('previous run', encoding='utf-8')
                configure_actual_distance_log(path)
                task = packet()
                policy = KevPolicy()
                monitor = policy.create_distance_monitor(Mock(), task, 2.5)
                monitor.update((0., 0.), [dict(id=4001, xy=(0., 0.), radius_m=86.04)], now=0.)
                monitor.close()
                text = path.read_text(encoding='utf-8')
                self.assertNotIn('previous run', text)
                self.assertIn('actual distance', text)
                self.assertIn('actual summary', text)
                self.assertIn('WARNING', text)
                root_handler.handle.assert_not_called()
                configure_actual_distance_log(path)
                self.assertEqual(path.read_text(encoding='utf-8'), '')
        finally:
            root.removeHandler(root_handler)
            for handler in ACTUAL_DISTANCE_LOGGER.handlers:
                handler.close()
            ACTUAL_DISTANCE_LOGGER.handlers = old_handlers
            ACTUAL_DISTANCE_LOGGER.setLevel(old_level)

    def test_actual_distance_monitor_detects_between_sample_crossing_and_summarizes(self):
        logger = Mock()
        monitor = ActualDistanceMonitor(logger, packet())
        obstacles = [dict(id=4001, xy=(0., 0.), radius_m=86.04, age_s=0.)]
        monitor.update((-100., 0.), obstacles, now=0.)
        self.assertFalse(logger.warning.called)
        monitor.update((100., 0.), obstacles, now=1.)
        self.assertEqual(monitor.active[4001]['min_distance_m'], 0.)
        self.assertEqual(monitor.active[4001]['violations'], 1)
        self.assertTrue(logger.warning.called)
        monitor.close()
        self.assertFalse(monitor.active)
        self.assertIn('actual summary', logger.warning.call_args.args[0])

    def test_actual_distance_monitor_does_not_bridge_stale_samples(self):
        monitor = ActualDistanceMonitor(Mock(), packet())
        obstacles = [dict(id=4001, xy=(0., 0.), radius_m=86.04)]
        monitor.update((-100., 0.), obstacles, now=0.)
        monitor.update((100., 0.), obstacles, now=10.)
        self.assertEqual(monitor.active[4001]['min_distance_m'], 100.)
        self.assertEqual(monitor.active[4001]['violations'], 0)
        monitor.update((600., 0.), obstacles, now=11.)
        self.assertFalse(monitor.active)

    def test_hull_inflation_unknown_types_and_friendly_targets(self):
        config=StaticObstacleConfig()
        self.assertAlmostEqual(config.collision_radius_m,86.04400529476,places=5)
        frame=LocalFrame(packet()['params']['planWaypointList'])
        targets=static_targets([record(1010,0,0),record(4001,500,0),
            dict(record(1002,0,200),sign=3),record(4003,700,0,speed=20.)],1010,frame,config)
        self.assertEqual([r['id'] for r in targets],[1002,4001])
        self.assertEqual(targets[0]['radius_m'],config.collision_radius_m)

    def test_lost_static_detection_is_retained_but_moving_report_replaces_it(self):
        p=packet();policy=KevPolicy(client=choose_heading)
        self.assertEqual(len(policy.observation(p)['static_obstacles']),1)
        p['situation']=p['situation'][:1]
        self.assertEqual(len(policy.observation(p)['static_obstacles']),1)
        p['situation'].append(record(4001,600,0,speed=10.))
        self.assertEqual(policy.observation(p)['static_obstacles'],[])
        p['situation'][1]['targetSpeed']=0.
        policy.observation(p)
        p['id']='new-task';p['situation']=p['situation'][:1]
        self.assertEqual(policy.observation(p)['static_obstacles'],[])

    def test_static_targets_do_not_expire_based_on_an_unsynchronized_source_clock(self):
        p=packet();p['situation'][1].update(timestamp=111000,_received_age_s=300.)
        obs=KevPolicy().observation(p)
        self.assertEqual(obs['static_obstacles'][0]['age_s'],300.)
        self.assertEqual(obs['static_obstacles'][0]['id'],4001)

    def test_detour_segments_clear_the_inflated_circle_and_keep_the_same_side(self):
        policy=KevPolicy();obs=policy.observation(packet());geo=WaypointGeometry(obs)
        navigator=StaticNavigator(policy.obstacle_config)
        nav=navigator.guide(geo,obs['static_obstacles'])
        self.assertTrue(nav['detour'])
        points=[geo.boat_position]+nav['path']
        self.assertTrue(all(clear_segment(a,b,obs['static_obstacles']) for a,b in zip(points,points[1:])))
        before=copy.deepcopy(nav['path'])
        for _ in range(5):
            self.assertEqual(navigator.guide(geo,obs['static_obstacles'])['path'],before)

    def test_obstacle_at_waypoint_uses_safe_part_of_original_arrival_circle(self):
        policy=KevPolicy();obs=policy.observation(packet(obstacle_x=1000.));geo=WaypointGeometry(obs)
        nav=StaticNavigator(policy.obstacle_config).guide(geo,obs['static_obstacles'])
        goal=nav['path'][-1]
        distance=math.dist(goal,geo.waypoint_positions[0])
        self.assertLess(distance,100.)
        self.assertGreater(distance,policy.obstacle_config.collision_radius_m)
        self.assertEqual(obs['task']['arrival_radius_m'],100.)

    def test_fully_occupied_arrival_region_is_reported_instead_of_expanding_it(self):
        policy=KevPolicy(obstacle_config=StaticObstacleConfig(obstacle_radius_m=150.))
        obs=policy.observation(packet(obstacle_x=1000.));geo=WaypointGeometry(obs)
        with self.assertRaises(StaticObstacleBlocked):
            StaticNavigator(policy.obstacle_config).guide(geo,obs['static_obstacles'])

    def test_collision_check_covers_between_samples_and_stopping_distance(self):
        policy=KevPolicy();obs=policy.observation(packet(speed=50.,obstacle_x=250.));geo=WaypointGeometry(obs)
        check=motion_clearance(geo,obs['boat'],50.,90.,obs['static_obstacles'],obs['execution'],1.,policy.obstacle_config)
        self.assertFalse(check['safe'])
        # The next one-second endpoint remains more than 200m from the obstacle.
        self.assertGreater(250-50*KNOT_MPS,policy.obstacle_config.collision_radius_m)
        self.assertFalse(clear_segment((-200.,0.),(200.,0.),[dict(xy=(0.,0.),radius_m=86.)]))

    def test_kev_filters_unsafe_candidates_and_preserves_turn_input_order(self):
        p=packet(x=250.,obstacle_x=500.)
        policy=KevPolicy(client=choose_heading)
        original=copy.deepcopy(p)
        policy(p)
        decision=policy.local.last_decision
        self.assertIn('static_avoidance',decision['state'])
        self.assertEqual(decision['state']['speeds_knots'],[25.])
        self.assertTrue(all(v['obstacle_clearance_m']>=0 for v in decision['options'].values()))
        self.assertTrue(all(list(v)[-1]=='turn_deg' for v in decision['options'].values()))
        self.assertEqual(p,original)

    def test_distant_blocking_obstacle_keeps_straight_cruise_then_turns_nearby(self):
        policy=KevPolicy(client=choose_heading)
        p=packet(speed=50.,obstacle_x=1200.,target=2500.)
        self.assertEqual(policy(p),(50.,90.))
        decision=policy.local.last_decision
        self.assertEqual(len(decision['options']),1)
        geo=WaypointGeometry(policy.observation(p))
        nav=policy.local.navigator.guide(geo,policy.observation(p)['static_obstacles'],
            boat=dict(targetSpeed=50.),execution=dict(max_turn_rate_deg_s=45.),speed_knots=50.)
        self.assertTrue(nav['approaching'])
        self.assertAlmostEqual(nav['guidance_xy_m'][1],geo.boat_position[1])
        self.assertTrue(all(clear_segment(a,b,policy.observation(p)['static_obstacles'],10.)
            for a,b in zip([geo.boat_position]+nav['path'],nav['path'])))
        p['situation'][0]=record(1010,850.,0.,speed=50.,course=90.)
        speed,course=policy(p)
        self.assertEqual(speed,25.)
        self.assertNotEqual(course,90.)
        self.assertTrue(policy.validate_control(p,p['situation'][0],p['situation'],speed,course)['safe'])

    def test_unrelated_nearby_obstacle_does_not_force_ten_knots(self):
        p=packet(speed=50.,target=2500.)
        p['situation'][1]=record(4001,0.,200.)
        policy=KevPolicy(client=choose_heading)
        self.assertEqual(policy(p),(50.,90.))

    def test_returning_to_target_direction_does_not_add_a_speed_cap(self):
        p=packet(speed=50.,obstacle_x=1200.,target=3500.)
        p['situation'][0]=record(1010,1400.,100.,speed=50.,course=54.)
        policy=KevPolicy(client=choose_heading)
        self.assertEqual(policy(p)[0],50.)
        self.assertEqual(policy.local.last_decision['state']['speed_profile']['phase'],'cruise')

    def test_turn_in_reserve_increases_with_speed_and_slower_turning(self):
        p=packet(obstacle_x=2000.,target=4000.)
        policy=KevPolicy()
        obs=policy.observation(p);geo=WaypointGeometry(obs)
        def approach(speed,rate):
            nav=StaticNavigator(policy.obstacle_config).guide(geo,obs['static_obstacles'],
                boat=dict(targetSpeed=speed),execution=dict(max_turn_rate_deg_s=rate),speed_knots=speed)
            self.assertTrue(nav['approaching'])
            return math.dist(nav['guidance_xy_m'],obs['static_obstacles'][0]['xy'])
        self.assertGreater(approach(50.,45.),approach(10.,45.))
        self.assertGreater(approach(50.,1.),approach(50.,45.))

    def test_static_safety_fallback_uses_fastest_feasible_speed(self):
        p=packet(speed=50.,obstacle_x=350.,target=2500.)
        policy=KevPolicy(client=choose_heading)
        original=motion_clearance
        def reject_fast(*args):
            if args[2] > 20.:
                return dict(safe=False,min_clearance_m=-1.,obstacle_id=4001)
            return original(*args)
        with patch('server.policies.kev_policy.motion_clearance',side_effect=reject_fast):
            self.assertEqual(policy(p)[0],20.)
        self.assertEqual(policy.local.last_decision['state']['speed_profile']['phase'],'static_safety_fallback')

    def test_near_obstacle_half_cap_does_not_repeat_halving_actual_speed(self):
        policy=KevPolicy(client=choose_heading)
        p=packet(speed=50.,obstacle_x=250.,target=2500.)
        for actual in (50.,25.,20.):
            p['situation'][0]['targetSpeed']=actual
            self.assertEqual(policy(p)[0],25.)
            self.assertEqual(policy.local.last_decision['state']['speed_profile']['phase'],'static_half_speed')

    def test_half_cap_below_the_normal_minimum_speed_is_supported(self):
        p=packet(speed=10.,obstacle_x=130.,target=2500.)
        policy=KevPolicy(client=choose_heading,max_speed_knots=10.)
        self.assertEqual(policy(p)[0],5.)
        self.assertTrue(all(item['speed_knots']<=5. for item in policy.local.last_decision['options'].values()))

    def test_send_time_check_uses_new_obstacle_positions_after_inference(self):
        policy=KevPolicy();p=packet(speed=50.,obstacle_x=1000.)
        own=p['situation'][0]
        self.assertTrue(policy.validate_control(p,own,p['situation'],50.,90.)['safe'])
        p['situation'][1]=record(4001,200.,0.)
        self.assertFalse(policy.validate_control(p,own,p['situation'],50.,90.)['safe'])

    def test_cache_metadata_does_not_change_external_records(self):
        store=ds.SituationStore();r=record(4001,500,0)
        store.update(r)
        self.assertNotIn('_received_age_s',store.query_state()[0])
        self.assertGreaterEqual(store.query_state(include_age=True)[0]['_received_age_s'],0.)
        self.assertNotIn('_received_age_s',r)

    def test_control_loop_forwards_all_targets_and_vetoes_a_new_unsafe_hold(self):
        p=packet(speed=50.,obstacle_x=800.)
        session=ds.ControlSession(p,('127.0.0.1',7612))
        registry=ds.TaskRegistry();registry.add(session)
        store=ds.SituationStore()
        for r in p['situation']:store.update(r)
        seen=[];commands=[];guard=KevPolicy()

        class Policy:
            def __call__(self,request):
                seen.append(request)
                # Simulate a new detection arriving during model inference.
                store.update(record(4002,200.,0.))
                return 50.,90.

            def validate_control(self,*args):
                return guard.validate_control(*args)

        def send(payload,_):
            command=ds.decode_speed_course_packet(payload);commands.append(command)
            if command['speed']==0:
                store.update(record(1010,0.,0.,speed=0.))
                session.request_stop()

        context=MagicMock();context.__enter__.return_value=Mock(sendto=send)
        with patch('server.control_runtime.socket.socket',return_value=context):
            ds.run_control_loop(session,Mock(),store,('127.0.0.1',18123),Policy(),
                                .01,3,2.5,.1,registry,3,.5)
        self.assertTrue(seen)
        self.assertIn(4001,[r['platformId'] for r in seen[0]['situation']])
        target=next(r for r in seen[0]['situation'] if r['platformId']==4001)
        self.assertIn('_received_age_s',target)
        self.assertTrue(commands)
        self.assertTrue(all(r['speed']==0. for r in commands))
        self.assertTrue(session.stop_confirmed)

    def test_initial_collision_cannot_be_counted_as_a_completed_waypoint(self):
        p=packet(speed=0.,obstacle_x=0.,target=0.)
        session=ds.ControlSession(p,('127.0.0.1',7612))
        registry=ds.TaskRegistry();registry.add(session)
        store=ds.SituationStore()
        for r in p['situation']:store.update(r)
        client=Mock();policy=KevPolicy(client=client)
        control=Mock();context=MagicMock();context.__enter__.return_value=control
        status=Mock()
        with patch('server.control_runtime.socket.socket',return_value=context):
            ds.run_control_loop(session,status,store,('127.0.0.1',18123),policy,
                                .01,3,2.5,.1,registry,3,.5)
        terminal=ds.analyze_task(status.sendto.call_args.args[0])
        self.assertEqual(terminal['status'],3)
        self.assertEqual(terminal['params']['errorCode'],'STATIC_OBSTACLE_BLOCKED')
        self.assertEqual(session.current_route,0)
        client.assert_not_called()
        self.assertEqual(ds.decode_speed_course_packet(control.sendto.call_args.args[0])['speed'],0.)


if __name__=='__main__':
    unittest.main()
