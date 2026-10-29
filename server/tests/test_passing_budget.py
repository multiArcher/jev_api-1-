"""Regression for the recorded slow parallel motion near a mandatory waypoint."""
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from server.policies.speed_profile import plan_speed,detour_braking_safe
from server.policies.kev_policy import KevPolicy
from server.navigation.dynamic_avoidance import AvoidanceConfig,Action,Vessel,Target
from server.experiments.field_tasks import load_task,_telemetry
from server.tests.test_heading_alignment import choose_alignment
from tools.waypoint_geometry import KNOT_MPS,xy,circle_interval
from tools.navigation_motion import predict_ground_motion


def observation(distance=320.,speed=40.):
    task=dict(next_waypoint_index=0,arrival_radius_m=100.,planWaypointList=[dict(routeSpeed=10.*KNOT_MPS)])
    obs=dict(task=task,boat=dict(targetSpeed=speed,targetCourse=90.),execution=dict(speed_transition_time_s=.5),planning=dict(action_duration_s=1.))
    geo=SimpleNamespace(target=(distance,0.),waypoint_positions=[(distance,0.)],waypoints=[(distance,0.)])
    return obs,geo


class PassingBudgetTests(unittest.TestCase):
    def test_side_by_side_pass_uses_available_braking_distance(self):
        obs,geo=observation()
        normal=plan_speed(obs,geo,[10.,20.,30.,40.,50.],12.)
        passing=plan_speed(obs,geo,[10.,20.,30.,40.,50.],12.,detouring=True)
        self.assertEqual(normal['speed_cap_knots'],10.)
        self.assertGreater(passing['speed_cap_knots'],10.)
        self.assertTrue(passing['detour_braking_guard'])
        near,near_geo=observation(distance=105.)
        self.assertEqual(plan_speed(near,near_geo,[10.,20.,30.,40.,50.],12.,detouring=True)['speed_cap_knots'],10.)
        obs['execution']['deceleration_curve']=((0.,.5),)
        self.assertLess(plan_speed(obs,geo,[10.,20.,30.,40.,50.],12.,detouring=True)['speed_cap_knots'],passing['speed_cap_knots'])

    def test_braking_tail_rejects_late_acceleration_toward_circle(self):
        obs,geo=observation(distance=250.)
        self.assertFalse(detour_braking_safe(obs,geo,50.,90.))
        self.assertTrue(detour_braking_safe(obs,geo,10.,90.))
        geo.waypoints.append((0.,0.));obs['task']['planWaypointList'].append(dict(routeSpeed=10.*KNOT_MPS))
        obs['boat']['targetSpeed']=9.6
        self.assertFalse(detour_braking_safe(obs,geo,20.,90.))

    def test_recorded_p2_approach_passes_without_circle_speed_violation(self):
        fixture=json.loads((Path(__file__).parent/'fixtures/passing_near_p2.json').read_text())
        task=load_task('six_waypoint_northwest_20261028');task['params']['next_waypoint_index']=1
        origin=tuple(task['params']['planWaypointList'][0][k] for k in ('routeLon','routeLat'))
        initial=fixture['own'];x,y=xy((initial['longitude'],initial['latitude']),origin)
        speed=initial['speed_knots']*KNOT_MPS;course=initial['course_deg'];omega=0.
        policy=KevPolicy(client=choose_alignment)
        def records(tick):
            result=[]
            for target in fixture['targets']:
                h=math.radians(target['course_deg']);p=target['position_xy_m'];v=target['speed_mps']
                r=_telemetry(origin,p[0]+v*math.sin(h)*tick,p[1]+v*math.cos(h)*tick,v,target['course_deg'],target['id'])
                r['_received_age_s']=target['age_s'];result.append(r)
            return result
        own=_telemetry(origin,x,y,speed,course,1010)
        policy.observation(dict(task,situation=[own,*records(0)]))
        passing_caps=[];p2_arrival=None;first_ahead=None
        for tick in range(65):
            own=_telemetry(origin,x,y,speed,course,1010);own['_turn_rate_deg_s']=omega
            packet=dict(task,situation=[own,*records(tick)])
            requested,heading=policy(packet)
            self.assertTrue(policy.validate_dynamic_control(packet,own,packet['situation'],requested,heading)['safe'])
            profile=policy.local.last_decision['state']['speed_profile']
            if profile['phase'] in ('detour_approach','moving_corridor'):passing_caps.append(profile['speed_cap_knots'])
            if policy.current_route()>=2 and p2_arrival is None:p2_arrival=tick
            t=fixture['targets'][0];h=math.radians(t['course_deg'])
            target_xy=(t['position_xy_m'][0]+t['speed_mps']*math.sin(h)*(tick+t['age_s']),t['position_xy_m'][1]+t['speed_mps']*math.cos(h)*(tick+t['age_s']))
            along=(x-target_xy[0])*math.sin(h)+(y-target_xy[1])*math.cos(h)
            if along>100. and first_ahead is None:first_ahead=tick
            motion=predict_ground_motion(speed,course,requested*KNOT_MPS,heading,0.,0.,1.,initial_turn_rate_deg_s=omega)
            for point in task['params']['planWaypointList']:
                center=xy((point['routeLon'],point['routeLat']),origin);offset=(center[0]-x,center[1]-y)
                for i,(a,b) in enumerate(zip(motion['path'],motion['path'][1:])):
                    interval=circle_interval(a,b,offset,100.)
                    if interval is not None:
                        peak=max(motion['path_speeds'][i]+(motion['path_speeds'][i+1]-motion['path_speeds'][i])*u for u in interval)
                        self.assertLessEqual(peak,point['routeSpeed']*KNOT_MPS+1e-7)
            x+=motion['endpoint'][0];y+=motion['endpoint'][1];speed=motion['end_speed'];course=motion['end_course_deg'];omega=motion['end_turn_rate_deg_s']
        self.assertTrue(any(cap>10. for cap in passing_caps))
        self.assertIsNotNone(first_ahead);self.assertLess(first_ahead,25)
        self.assertIsNotNone(p2_arrival);self.assertLess(p2_arrival,40)


if __name__=='__main__':unittest.main()
