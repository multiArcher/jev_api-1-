"""FollowMode and AccompanyMode contracts from integration document §5.2/5.3."""
import copy
import logging
import math
import time

from tools.waypoint_geometry import KNOT_MPS,LocalFrame
from .target_following import TargetUnavailable,finite,relative_state,forecast

FOLLOW_CODE='followMode'
ACCOMPANY_CODE='accompanyMode'
CODES=(FOLLOW_CODE,ACCOMPANY_CODE)
LOG=logging.getLogger(__name__)


class TargetTaskUnavailable(TargetUnavailable):
    error_code='TARGET_UNAVAILABLE'


class TargetTaskUnreachable(ValueError):
    error_code='TARGET_TASK_UNREACHABLE'


def target_id(task):
    return task['params']['targetPlatCode']


def validate_task(task):
    params=task.get('params')
    if not isinstance(params,dict) or task.get('code') not in CODES:
        raise ValueError('expected followMode/accompanyMode with object params')
    own,target=params.get('platCode'),params.get('targetPlatCode')
    if any(type(pid) is not int or not 0<=pid<=65535 for pid in (own,target)) or own==target:
        raise ValueError('platCode and targetPlatCode must be distinct UINT16 integers')
    if any(k in params for k in ('planWaypointList','planWaypointNum','currentRoute','next_waypoint_index')):
        raise ValueError('followMode/accompanyMode do not accept waypoint progress fields')
    if not finite(params.get('maxSpeed',0.)) or params.get('maxSpeed',0.)<0:
        raise ValueError('maxSpeed must be nonnegative; 0 uses the configured vessel maximum')
    for field,default in (('targetMaxAgeS',2.5),('unreachableTimeoutS',60.)):
        if not finite(params.get(field,default)) or params.get(field,default)<=0:
            raise ValueError(f'{field} must be positive')
    if task['code']==FOLLOW_CODE:
        lower,upper=params.get('followDistanceMinM'),params.get('followDistanceMaxM')
        if not all(finite(v) for v in (lower,upper)) or not 0<=lower<=upper or upper<=0:
            raise ValueError('follow distances must satisfy 0 <= followDistanceMinM <= followDistanceMaxM and max > 0')
    else:
        for axis in ('Along','Across'):
            lower,upper=params.get(f'offset{axis}MinM'),params.get(f'offset{axis}MaxM')
            if not all(finite(v) for v in (lower,upper)) or lower>upper:
                raise ValueError(f'offset{axis}MinM must be <= offset{axis}MaxM')
        if type(params.get('syncHeading')) is not int or params['syncHeading'] not in (0,1):
            raise ValueError('syncHeading must be UINT8 0 or 1')


def validate_feasibility(task,safety_radius_m):
    params=task['params']
    if task['code']==FOLLOW_CODE:
        if params['followDistanceMaxM']<=safety_radius_m+.5:
            raise TargetTaskUnreachable('跟随距离上限位于目标避碰范围内，没有可用安全间距')
    else:
        farthest=max(math.hypot(a,c) for a in (params['offsetAlongMinM'],params['offsetAlongMaxM'])
                     for c in (params['offsetAcrossMinM'],params['offsetAcrossMaxM']))
        if farthest<=safety_radius_m+.5:
            raise TargetTaskUnreachable('整个伴随位置范围位于目标避碰范围内，没有可用安全位置')


def target_record(task,records):
    pid=target_id(task)
    target=next((r for r in reversed(tuple(records)) if r.get('platformId')==pid),None)
    if target is None:
        raise TargetTaskUnavailable(f'指定目标{pid}缺失')
    age=target.get('_received_age_s',0.)
    if not finite(age) or not 0<=age<=task['params'].get('targetMaxAgeS',2.5):
        raise TargetTaskUnavailable(f'指定目标{pid}态势已过期')
    for field,lower,upper in (('longitude',-180.,180.),('latitude',-89.999,89.999),
                              ('targetSpeed',0.,math.inf),('targetCourse',0.,359.999999999)):
        if not finite(target.get(field)) or not lower<=target[field]<=upper:
            raise TargetTaskUnavailable(f'指定目标{pid}缺少有效{field}')
    return target


def outside(value,lower,upper):
    return value-lower if value<lower else value-upper if value>upper else 0.


def body_position(delta,course):
    """delta points own->target; return own position in the target frame."""
    h=math.radians(course)
    return (-delta[0]*math.sin(h)-delta[1]*math.cos(h),
            -delta[0]*math.cos(h)+delta[1]*math.sin(h))


def progress(task,own=None,target=None,speed_unit='knots'):
    params=task['params']
    result=dict(targetPlatCode=target_id(task),targetAvailable=target is not None,
                targetSpeed=target['targetSpeed'] if target is not None else None,
                targetCourse=target['targetCourse'] if target is not None else None)
    if task['code']==FOLLOW_CODE:
        result.update(relativeDistanceM=None,relativeBearingDeg=None,inRange=0,distanceOutM=None)
    else:
        result.update(currentAlongM=None,currentAcrossM=None,alongOutM=None,acrossOutM=None,inPosition=0)
    if own is None or target is None:
        return result
    delta,_,distance,_=relative_state(own,target,speed_unit)
    along,across=body_position(delta,target['targetCourse'])
    if task['code']==FOLLOW_CODE:
        error=outside(distance,params['followDistanceMinM'],params['followDistanceMaxM'])
        result.update(relativeDistanceM=round(distance,2),
            relativeBearingDeg=round(math.degrees(math.atan2(across,along))%360,2)%360,
            inRange=int(error==0.),distanceOutM=round(error,2))
    else:
        a=outside(along,params['offsetAlongMinM'],params['offsetAlongMaxM'])
        c=outside(across,params['offsetAcrossMinM'],params['offsetAcrossMaxM'])
        result.update(currentAlongM=round(along,2),currentAcrossM=round(across,2),
                      alongOutM=round(a,2),acrossOutM=round(c,2),inPosition=int(a==0. and c==0.))
    return result


def _trail_point(points,lag,heading):
    for a,b in zip(reversed(points[:-1]),reversed(points[1:])):
        length=math.dist(a,b)
        if length>1e-8:
            if lag<=length:
                return tuple(b[i]+(a[i]-b[i])*lag/length for i in (0,1))
            lag-=length
    h=math.radians(heading)
    a=points[0]
    if len(points)>1 and math.dist(points[0],points[1])>1e-8:
        direction=tuple((points[1][i]-a[i])/math.dist(a,points[1]) for i in (0,1))
    else:
        direction=(math.sin(h),math.cos(h))
    return tuple(a[i]-lag*direction[i] for i in (0,1))


def _offset_reference(params,radius):
    middle=((params['offsetAlongMinM']+params['offsetAlongMaxM'])/2.,
            (params['offsetAcrossMinM']+params['offsetAcrossMaxM'])/2.)
    choices=[middle,*((a,c) for a in (params['offsetAlongMinM'],params['offsetAlongMaxM'])
                     for c in (params['offsetAcrossMinM'],params['offsetAcrossMaxM']))]
    safe=[p for p in choices if math.hypot(*p)>radius+.5]
    return min(safe,key=lambda p:math.dist(p,middle))


def action(policy,packet):
    if __package__.startswith('server.'):
        from ..policies.turning import reachable_courses
        from ..policies.kev_policy import round_model_values
    else:
        from policies.turning import reachable_courses
        from policies.kev_policy import round_model_values
    validate_task(packet)
    params=packet['params'];code=packet['code']
    radius=max(policy.obstacle_config.collision_radius_m,policy.dynamic_config.safe_distance_m)
    validate_feasibility(packet,radius)
    records=packet.get('situation',())
    own=next((dict(r) for r in records if r.get('platformId')==params['platCode']),None)
    if own is None:
        raise ValueError('No own vessel situation')
    own['targetCourse']=policy._observation_course(own)
    if not all(finite(own.get(k)) for k in ('longitude','latitude','targetSpeed')) or own['targetSpeed']<0 or abs(own['longitude'])>180 or abs(own['latitude'])>=90:
        raise ValueError('Invalid own vessel situation')
    target=target_record(packet,records)
    delta,velocity,distance,_=relative_state(own,target,policy.speed_unit)
    factor=KNOT_MPS if policy.speed_unit=='knots' else 1.
    target_knots=target['targetSpeed']*factor/KNOT_MPS
    cap=params.get('maxSpeed',0.)*factor/KNOT_MPS
    cap=min(policy.max_speed,cap) if cap>0 else policy.max_speed
    key=(packet.get('simId'),packet.get('id'),code,params)
    if getattr(policy.local,'target_task_key',None)!=key:
        policy.local.target_task_key=copy.deepcopy(key)
        policy.local.target_trail=[]
        policy.local.target_frame=LocalFrame([dict(routeLon=own['longitude'],routeLat=own['latitude'])])
        policy.local.target_stable_cycles=0
        policy.local.target_best_error=math.inf
        policy.local.target_progress_at=time.monotonic()
    frame=policy.local.target_frame
    own_xy=frame.project(own['longitude'],own['latitude'])
    target_xy=tuple(own_xy[i]+delta[i] for i in (0,1))
    report=progress(packet,own,target,policy.speed_unit)
    in_position=bool(report.get('inRange',report.get('inPosition',0)))
    policy.local.target_stable_cycles=(policy.local.target_stable_cycles+1 if in_position else 0)
    h=math.radians(target['targetCourse'])
    if code==FOLLOW_CODE:
        points=policy.local.target_trail
        if not points or math.dist(points[-1],target_xy)>=1.:
            points.append(target_xy)
            if len(points)>4096:
                del points[:-4096]
        lower=max(params['followDistanceMinM'],radius+.51)
        reference=(lower+params['followDistanceMaxM'])/2.
        def goal_at(seconds):
            lag=reference-math.hypot(*velocity)*seconds
            if lag<0:
                return tuple(target_xy[i]+velocity[i]*(-lag/max(math.hypot(*velocity),1e-8)) for i in (0,1))
            return _trail_point(points,lag,target['targetCourse'])
    else:
        along,across=_offset_reference(params,radius)
        def goal_at(seconds):
            return (target_xy[0]+velocity[0]*seconds+along*math.sin(h)+across*math.cos(h),
                    target_xy[1]+velocity[1]*seconds+along*math.cos(h)-across*math.sin(h))
    goal=goal_at(0.)
    offset=tuple(goal[i]-own_xy[i] for i in (0,1))
    bearing=math.degrees(math.atan2(*offset))%360 if math.hypot(*offset)>.5 else target['targetCourse']
    initial_error=abs((bearing-own['targetCourse']+180)%360-180)
    speeds=sorted({0.,min(1.,cap),min(2.5,cap),min(5.,cap),min(target_knots,cap),cap,cap*.5,cap*.75,
                   *(s for s in policy.speeds if s<=cap)})
    courses=sorted({round(c,2)%360 for c in reachable_courses(own['targetCourse'],policy.duration,policy.max_turn_rate_deg_s)
                    +[own['targetCourse'],bearing,target['targetCourse']]})
    if initial_error>30.:
        courses=sorted(set(courses)|{(own['targetCourse']+side*angle)%360 for side in (-1,1) for angle in (30.,60.,90.)})
    criteria={}
    for speed in speeds:
        for j,course in enumerate(courses if speed else [own['targetCourse']]):
            wire_speed=round(speed*KNOT_MPS if policy.speed_unit=='mps' else speed,2)
            # Respect the requested cap even when the wire rounds upward.
            wire_cap=math.floor(cap*(KNOT_MPS if policy.speed_unit=='mps' else 1.)*100.+1e-8)/100.
            wire_speed=min(wire_speed,wire_cap)
            if not policy.validate_control(packet,own,records,wire_speed,course)['safe'] or not policy.validate_dynamic_control(packet,own,records,wire_speed,course)['safe']:
                continue
            actual=wire_speed/KNOT_MPS if policy.speed_unit=='mps' else wire_speed
            effects=[]
            for seconds in (policy.duration,max(10.,policy.duration)):
                motion=forecast(policy,own,actual,course,seconds)
                predicted=tuple(own_xy[i]+motion['endpoint'][i] for i in (0,1))
                rel=tuple(delta[i]+velocity[i]*seconds-motion['endpoint'][i] for i in (0,1))
                aa,cc=body_position(rel,target['targetCourse'])
                band=(abs(outside(math.hypot(*rel),params['followDistanceMinM'],params['followDistanceMaxM']))
                      if code==FOLLOW_CODE else math.hypot(outside(aa,params['offsetAlongMinM'],params['offsetAlongMaxM']),outside(cc,params['offsetAcrossMinM'],params['offsetAcrossMaxM'])))
                effects.append((math.dist(predicted,goal_at(seconds)),band,motion['end_course_deg']))
            turn=abs((course-own['targetCourse']+180)%360-180)
            criteria[f'speed_{actual:g}__heading_{j:02d}']=dict(speed_knots=actual,absolute_course_deg=course,
                guidance_error_after_m=effects[0][0],guidance_error_preview_m=effects[1][0],
                range_or_position_error_after_m=effects[0][1],range_or_position_error_preview_m=effects[1][1],
                heading_gain_deg=initial_error-abs((bearing-effects[0][2]+180)%360-180),
                target_heading_error_deg=abs((course-target['targetCourse']+180)%360-180),turn_deg=turn)
    sync=code==ACCOMPANY_CODE and params['syncHeading']==1 and policy.local.target_stable_cycles>=3
    def rank(key):
        c=criteria[key]
        if initial_error>60.:
            return (-c['heading_gain_deg'],c['guidance_error_preview_m'],c['turn_deg'])
        if sync:
            return (c['range_or_position_error_preview_m'],c['target_heading_error_deg'],c['guidance_error_preview_m'])
        return (c['range_or_position_error_preview_m'],c['guidance_error_preview_m'],c['turn_deg'])
    objective=math.hypot(*offset)
    settled=(in_position and (code==ACCOMPANY_CODE or
        objective<=max(10.,(params['followDistanceMaxM']-params['followDistanceMinM'])/2.)))
    if objective>policy.local.target_best_error+20.:
        policy.local.target_best_error=objective
    if settled or objective<policy.local.target_best_error-1.:
        policy.local.target_best_error=objective
        policy.local.target_progress_at=time.monotonic()
    if time.monotonic()-policy.local.target_progress_at>=params.get('unreachableTimeoutS',60.):
        raise TargetTaskUnreachable('目标任务持续无法缩小位置误差或找到安全可达动作')
    state=dict(task=code,**report,request_params=copy.deepcopy(params),max_speed_knots=cap,
        actual_speed_knots=own['targetSpeed']*factor/KNOT_MPS,actual_course_deg=own['targetCourse'],
        guidance_offset_m=offset,heading_sync_active=sync,command_duration_s=policy.duration,
        motion_limits=dict(policy.execution,control_delay_s=policy.dynamic_config.control_delay_s,
                           controller_deceleration_mps2=policy.dynamic_config.controller_deceleration_mps2),
        prediction_assumption='Target continues current velocity/course; future turns unknown. Follow uses recorded trail, extending backwards before enough history exists.')
    if not criteria:
        policy.local.last_decision=dict(selection_mode='target_task_safety_hold',state=state,options={},candidate_count=0)
        return 0.,own['targetCourse']
    criteria={k:criteria[k] for k in sorted(criteria,key=rank)[:16]}
    instructions=('This is a continuous target task: inRange/inPosition is not completion. '
        'Choose only supplied collision-checked commands. Respect max_speed_knots and physical motion limits. '
        'Prefer restoring the requested range/position band, then reducing guidance_error_preview_m. '
        'Keep useful turning when initially misaligned, even if distance first increases. '
        'A zero-speed command does not stop an approaching target. ')
    instructions+=('Follow the target recorded path toward a point behind it; do not settle beside or ahead of the target merely because radial range is valid. '
                   if code==FOLLOW_CODE else
                   'Hold the requested along/across bands in the target body frame: forward positive along, starboard positive across. ')
    if sync:
        instructions+='Once position is stable, align target heading while keeping the position band. '
    elif code==ACCOMPANY_CODE:
        instructions+='Position is primary; no mandatory heading synchronization until requested and stable. '
    payload=round_model_values(dict(model='kev-latest',state=state,questions=dict(action=dict(type='choice',instructions=instructions,criteria=criteria))))
    started=time.perf_counter();result=policy._call(payload)
    answers=result.get('answers') if isinstance(result,dict) else None
    answer=answers.get('action') if isinstance(answers,dict) else None
    if isinstance(result,dict) and result.get('truncated'):
        raise RuntimeError('Kev truncated target task input')
    if not isinstance(answer,dict) or answer.get('type')!='choice' or answer.get('choice') not in criteria:
        raise ValueError('Expected Kev Choice among target task candidates')
    chosen=criteria[answer['choice']]
    policy.local.last_decision=dict(selection_mode=code,choice=answer['choice'],state=payload['state'],
        options=payload['questions']['action']['criteria'],candidate_count=len(criteria),
        inference_s=time.perf_counter()-started,result=result)
    LOG.info('Kev target task code=%s id=%s target=%s speed=%s course=%s report=%s',code,packet.get('id'),target_id(packet),chosen['speed_knots'],chosen['absolute_course_deg'],report)
    return chosen['speed_knots']*(KNOT_MPS if policy.speed_unit=='mps' else 1.),chosen['absolute_course_deg']
