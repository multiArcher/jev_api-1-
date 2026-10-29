"""Regression for the task-one waypoint switch under bounded yaw acceleration."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from server.experiments.field_tasks import TestConfig, load_task, test_task_one as run_one
from server.policies.kev_policy import KevPolicy
from server.policies.turning import alignment_courses
from server.tests.test_kev_policy import answer
from tools.navigation_motion import predict_ground_motion
from tools.waypoint_geometry import KNOT_MPS


def choose_alignment(payload):
    items=payload['questions']['action']['criteria']
    return answer(min(items,key=lambda k:(items[k].get('heading_error_after_preview_deg',
                         items[k]['target_heading_error_deg']),-items[k]['speed_knots'])))


class HeadingAlignmentTests(unittest.TestCase):
    def packet(self, course=0.):
        task=load_task('task_one')
        first=task['params']['planWaypointList'][0]
        return dict(task,situation=[dict(platformId=1010,longitude=first['routeLon']-.002,
                    latitude=first['routeLat'],targetSpeed=0.,targetCourse=course)])

    def test_sustained_target_includes_bearing_but_actual_first_second_obeys_physics(self):
        for initial,target in ((90.,158.),(359.,80.),(0.,180.)):
            courses=alignment_courses(initial,target,12.,1.)
            self.assertEqual(len(set(courses)),16)
            self.assertIn(target,courses)
            motion=predict_ground_motion(5.,initial,5.,target,0.,0.,1.)
            self.assertLessEqual(abs((motion['end_course_deg']-initial+180)%360-180),2.+1e-8)
            self.assertLessEqual(abs(motion['end_turn_rate_deg_s']),4.+1e-8)

    def test_alignment_latch_exits_at_fifteen_degrees_and_resets_for_new_task(self):
        client=Mock(side_effect=choose_alignment);policy=KevPolicy(client=client)
        packet=self.packet()
        policy(packet)
        payload=client.call_args.args[0]
        self.assertTrue(payload['state']['heading_alignment']['active'])
        self.assertEqual(payload['state']['speeds_knots'],[10.])
        self.assertTrue(all(v['heading_gain_deg']>=2. for v in payload['questions']['action']['criteria'].values()))
        self.assertIn('heading_error_after_preview_deg',payload['questions']['action']['instructions'])
        packet['situation'][0]['targetCourse']=70.
        policy(packet)
        self.assertIn('heading_alignment',client.call_args.args[0]['state'])
        packet['id']='new-task'
        policy(packet)
        self.assertNotIn('heading_alignment',client.call_args.args[0]['state'])
        packet['situation'][0]['targetCourse']=0.
        policy(packet)
        packet['situation'][0]['targetCourse']=85.
        policy(packet)
        self.assertNotIn('heading_alignment',client.call_args.args[0]['state'])

    def test_preview_uses_actual_heading_and_future_target_bearing(self):
        packet=self.packet();packet['situation'][0]['targetSpeed']=10.
        client=Mock(side_effect=choose_alignment);policy=KevPolicy(client=client)
        policy(packet);payload=client.call_args.args[0]
        item=min(payload['questions']['action']['criteria'].values(),key=lambda v:v['heading_error_after_preview_deg'])
        target=item['absolute_course_deg']
        motion=predict_ground_motion(10.*KNOT_MPS,0.,10.*KNOT_MPS,target,0.,0.,10.)
        self.assertAlmostEqual(item['heading_after_preview_deg'],motion['end_course_deg'],places=2)
        self.assertNotEqual(item['heading_error_after_preview_deg'],item['target_heading_error_deg'])
        self.assertEqual(list(item)[-1],'turn_deg')

    def test_progress_filter_does_not_restore_statically_unsafe_turns(self):
        client=Mock(side_effect=choose_alignment);policy=KevPolicy(client=client)
        def guard(geometry,boat,speed,course,*args):
            safe=course==0.
            return dict(safe=safe,min_clearance_m=1. if safe else -1.,obstacle_id=4001)
        with patch('server.policies.kev_policy.motion_clearance',side_effect=guard):
            policy(self.packet())
        payload=client.call_args.args[0]
        self.assertFalse(payload['state']['heading_alignment']['progress_filter_applied'])
        self.assertTrue(all(v['absolute_course_deg']==0. for v in payload['questions']['action']['criteria'].values()))

    def site_switch_packet(self, distance_m=20000., age_s=0., target_speed=5.):
        task=load_task('six_waypoint_northwest_20261028')
        first=task['params']['planWaypointList'][0]
        task['params']['next_waypoint_index']=1
        task['situation']=[dict(platformId=1010,longitude=first['routeLon'],latitude=first['routeLat'],
            targetSpeed=10.,targetCourse=113.,_received_age_s=0.),
            dict(platformId=4001,longitude=first['routeLon'],
                 latitude=first['routeLat']+distance_m/111320.,targetSpeed=target_speed,
                 targetCourse=180.,_received_age_s=age_s)]
        return task

    def test_far_moving_or_uncertain_target_keeps_site_p2_alignment(self):
        for age,speed in ((0.,5.),(30.,5.),(0.,None)):
            with self.subTest(age=age,speed=speed):
                packet=self.site_switch_packet(age_s=age,target_speed=speed)
                policy=KevPolicy(client=choose_alignment)
                actual_speed,course=policy(packet)
                decision=policy.local.last_decision
                self.assertTrue(decision['state']['heading_alignment']['active'])
                self.assertEqual(decision['alignment_gate']['reason'],'safe_alignment')
                self.assertNotIn('dynamic_avoidance',decision['state'])
                self.assertLessEqual(actual_speed,10.)
                self.assertGreater(course,125.)
                self.assertTrue(all(v['heading_gain_deg']>=2. for v in decision['options'].values()))
                for v in decision['options'].values():
                    self.assertTrue(policy.validate_dynamic_control(packet,packet['situation'][0],
                        packet['situation'],v['speed_knots'],v['absolute_course_deg'])['safe'])

    def test_warning_only_target_does_not_disable_alignment(self):
        import math
        packet=self.site_switch_packet()
        own,target=packet['situation']
        # Opposing target ahead: collision in the long warning window only.
        heading=math.radians(162.69)
        target.update(longitude=own['longitude']+800.*math.sin(heading)/(111320.*math.cos(math.radians(own['latitude']))),
                      latitude=own['latitude']+800.*math.cos(heading)/111320.,
                      targetSpeed=20.,targetCourse=342.69)
        policy=KevPolicy(client=choose_alignment)
        policy(packet)
        decision=policy.local.last_decision
        self.assertTrue(decision['alignment_gate']['active'])
        self.assertIn('dynamic_warning',decision)
        self.assertNotIn('dynamic_avoidance',decision['state'])

    def test_near_risk_does_not_shrink_avoidance_space_to_alignment_speed(self):
        packet=self.site_switch_packet(distance_m=100.)
        # Target on the desired path, approaching at 20 knots.
        packet['situation'][1].update(targetSpeed=20.,targetCourse=180.)
        policy=KevPolicy(client=choose_alignment)
        # Isolate the alignment risk gate from geometric obstacle guidance.
        with patch('server.policies.kev_policy.slow_obstacle_geometry',return_value=([],[],False)):
            policy(packet)
        gate=policy.local.last_decision['alignment_gate']
        self.assertTrue(gate['requested'])
        self.assertFalse(gate['active'])
        self.assertEqual(gate['reason'],'dynamic_risk')
        self.assertIn(4001,gate['threat_ids'])
        self.assertNotIn('heading_alignment',policy.local.last_decision['state'])

    def test_gate_rechecks_risk_after_target_moves_away(self):
        packet=self.site_switch_packet(distance_m=100.)
        policy=KevPolicy(client=choose_alignment)
        with patch('server.policies.kev_policy.slow_obstacle_geometry',return_value=([],[],False)):
            policy(packet)
        self.assertEqual(policy.local.last_decision['alignment_gate']['reason'],'dynamic_risk')
        packet['situation'][1]['latitude']+=.2
        with patch('server.policies.kev_policy.slow_obstacle_geometry',return_value=([],[],False)):
            policy(packet)
        self.assertTrue(policy.local.last_decision['alignment_gate']['active'])

    def test_guidance_log_explains_target_and_alignment_gate(self):
        packet=self.site_switch_packet()
        policy=KevPolicy(client=choose_alignment)
        with self.assertLogs('server.policies.kev_policy',level='INFO') as captured:
            policy(packet)
        line=next(line for line in captured.output if 'Kev guidance' in line)
        for text in ('target_index=1','target_waypoint=2/6','actual_course_deg=113.00',
                     'target_bearing_deg=162.69','alignment=True',
                     'alignment_reason=safe_alignment',"'id': 4001","'speed_mps':"):
            self.assertIn(text,line)

    def test_original_east_entry_switches_to_p2_without_stall_or_premature_acceleration(self):
        policy=KevPolicy(client=choose_alignment)
        with tempfile.TemporaryDirectory() as folder:
            result=run_one(folder,TestConfig(steps=45,start_offset_m=150.,initial_course_deg=90.),policy,plots=False)
            rows=[json.loads(line) for line in (Path(folder)/'task_one/trajectory.jsonl').read_text().splitlines()]
        self.assertNotEqual(result['reason'],'navigation_no_progress')
        turn=[r for r in rows if r.get('decision',{}).get('state',{}).get('heading_alignment',{}).get('active')]
        self.assertTrue(turn)
        self.assertTrue(all(r['command']['speed']<=10. for r in turn))
        after=[r for r in rows if r['target_index']==1 and r.get('decision',{}).get('state',{}).get('target_relative_bearing_deg') is not None]
        self.assertLess(abs(after[-1]['decision']['state']['target_relative_bearing_deg']),15.)
        self.assertEqual(result['static_collision_count'],0)
        self.assertEqual(result['speed_violation_count'],0)


if __name__=='__main__':unittest.main()
