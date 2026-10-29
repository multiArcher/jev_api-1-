"""Slow moving targets use the same geometric bypass as ordinary obstacles."""
from types import SimpleNamespace
import unittest
import subprocess
import sys
from pathlib import Path
from unittest.mock import patch
from server.policies.kev_policy import KevPolicy
from server.policies.static_obstacles import StaticObstacleBlocked
from server.tests.test_kev_dynamic_avoidance import moving_packet
from server.tests.test_static_obstacles import choose_heading
from server.navigation.dynamic_avoidance import AvoidanceConfig,Target,Vessel
from server.policies.moving_obstacles import slow_obstacle_geometry
from server.experiments.kev_dynamic_avoidance import run_scene
from tools.waypoint_geometry import KNOT_MPS


class SlowObstacleTests(unittest.TestCase):
    def test_direct_service_script_import_without_repository_on_pythonpath(self):
        root=Path(__file__).resolve().parents[2]
        script=("import sys; "
                f"sys.path[:0]={[str(root/'server'),str(root/'jev_api')]!r}; "
                "from policies.kev_policy import KevPolicy; KevPolicy()")
        result=subprocess.run([sys.executable,'-I','-c',script],cwd='/tmp',
                              capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)

    def test_geometry_uses_motion_and_releases_when_safely_astern(self):
        geo=SimpleNamespace(next_index=0,waypoint_positions=[(3000.,0.)])
        target=Target(4001,(300.,0.),9.6*KNOT_MPS,90.)
        config=AvoidanceConfig()
        obstacles,ids,_=slow_obstacle_geometry((Vessel((0.,0.),20.,90.),(target,)),geo,25.,config)
        self.assertEqual(ids,[4001]);self.assertEqual(len(obstacles),3)
        self.assertAlmostEqual(obstacles[-1]['xy'][0],300+20*target.speed_mps)
        self.assertEqual(slow_obstacle_geometry((Vessel((600.,0.),20.,90.),(target,)),geo,25.,config,
                                              initial_position=(0.,0.)),([],[],False))

    def test_unreliable_far_fast_and_opposing_targets_stay_in_dynamic_guard(self):
        geo=SimpleNamespace(next_index=0,waypoint_positions=[(3000.,0.)])
        for target in (Target(1,(300.,0.),25.,90.),Target(1,(300.,0.),5.,270.),
                       Target(1,(20000.,0.),5.,90.),Target(1,(300.,0.),5.,90.,30.),
                       Target(1,(300.,0.),None,None)):
            self.assertEqual(slow_obstacle_geometry((Vessel((0.,0.),20.,90.),(target,)),geo,25.,AvoidanceConfig()),([],[],False))

    def test_moving_geometry_blockage_uses_safe_dynamic_fallback(self):
        policy=KevPolicy(client=choose_heading)
        packet=moving_packet(x=300.,speed=9.6,course=90.)
        with patch('server.policies.kev_policy.StaticNavigator.guide',
                   side_effect=[StaticObstacleBlocked('temporary occupied corridor'),None]):
            speed,heading=policy(packet)
        self.assertGreater(speed,0.)
        self.assertIn('moving_geometry_fallback',policy.local.last_decision)
        self.assertTrue(policy.validate_dynamic_control(packet,packet['situation'][0],packet['situation'],speed,heading)['safe'])

    def test_closed_loop_direct_bypass_and_route_return(self):
        for initial,targets in ((20.,(Target(4001,(300.,0.),9.6*KNOT_MPS,90.),)),
                                (50.,(Target(4001,(300.,0.),9.6*KNOT_MPS,90.),)),
                                (50.,(Target(4001,(300.,0.),9.6*KNOT_MPS,90.),
                                      Target(4003,(700.,-30.),9.6*KNOT_MPS,98.)))):
            with self.subTest(initial=initial,count=len(targets)):
                result=run_scene('direct_bypass',targets,steps=120,mock=True,response='physical',initial_speed_knots=initial)
                self.assertGreater(result['min_clearance_m']['dynamic'],0.)
                for target in targets:
                    self.assertTrue(any(row['own']['position_m'][0]-next(t for t in row['targets']
                        if t['target_id']==target.target_id)['position_m'][0]>150. for row in result['samples']))
                self.assertLess(abs(result['samples'][-1]['own']['position_m'][1]),10.)
                detours=[row for row in result['samples'] if row['decision'].get('state',{}).get('static_avoidance',{}).get('moving_target_ids')]
                self.assertTrue(detours)
                self.assertTrue(all(row['command']['speed_knots'] > 9.6 for row in detours))
                self.assertTrue(all('overtaking' not in row['decision'].get('dynamic_avoidance',{}) for row in result['samples']))
                self.assertTrue(all(row['decision']['state']['speed_profile']['phase'] not in
                    ('static_half_speed','heading_alignment') for row in detours))


if __name__=='__main__':unittest.main()
