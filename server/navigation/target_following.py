"""Distance-band following, adapted to this service's motion and Kev interfaces.

Reference: cocoder777/USV2DSimulator, bda51be, following_geometry.py.
Only current target velocity is known; forecasts are not future target routes.
"""
import math
import time

from tools.navigation_motion import predict_ground_motion
from tools.vessel_dynamics import motion_parameters
from tools.waypoint_geometry import KNOT_MPS, xy

FOLLOWING_CODE = 'targetFollowing'
TARGET_CODES = ('followMode','accompanyMode')


def is_target_task(task):
    return task.get('code') in (FOLLOWING_CODE,*TARGET_CODES)


def target_id(task):
    return task['params']['targetPlatCode' if task.get('code') in TARGET_CODES else 'targetPlatformId']


def finite(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def validate_task(task):
    if task.get('code') in TARGET_CODES:
        from .target_tasks import validate_task as validate_document_task
        return validate_document_task(task)
    params = task.get('params')
    if not isinstance(params, dict):
        raise ValueError('targetFollowing params must be an object')
    own, target = params.get('platCode'), params.get('targetPlatformId')
    if any(type(pid) is not int or not 0 <= pid <= 0xFFFFFFFF for pid in (own, target)) or own == target:
        raise ValueError('platCode and targetPlatformId must be distinct uint32 integers')
    lower, upper = params.get('minFollowingDistanceM'), params.get('maxFollowingDistanceM')
    if not all(finite(v) for v in (lower, upper)) or not 0 < lower < upper:
        raise ValueError('following distances must satisfy 0 < minFollowingDistanceM < maxFollowingDistanceM')
    max_age = params.get('targetMaxAgeS', 2.5)
    if not finite(max_age) or max_age <= 0:
        raise ValueError('targetMaxAgeS must be positive and finite')
    if any(k in params for k in ('planWaypointList', 'planWaypointNum', 'currentRoute', 'next_waypoint_index')):
        raise ValueError('targetFollowing does not accept waypoint progress fields')


class TargetUnavailable(TimeoutError):
    error_code = 'FOLLOWING_TARGET_UNAVAILABLE'


def target_record(task, records):
    if task.get('code') in TARGET_CODES:
        from .target_tasks import target_record as document_target_record
        return document_target_record(task,records)
    params = task['params']
    target = next((r for r in reversed(tuple(records)) if r.get('platformId') == params['targetPlatformId']), None)
    if target is None:
        raise TargetUnavailable('指定追踪目标{}缺失'.format(params['targetPlatformId']))
    age = target.get('_received_age_s', 0.)
    if not finite(age) or age < 0 or age > params.get('targetMaxAgeS', 2.5):
        raise TargetUnavailable('指定追踪目标态势已过期')
    for name, low, high in (('longitude', -180, 180), ('latitude', -89.999, 89.999),
                            ('targetSpeed', 0, float('inf')), ('targetCourse', 0, 359.999999999)):
        value = target.get(name)
        if not finite(value) or not low <= value <= high:
            raise TargetUnavailable('指定追踪目标缺少有效{}'.format(name))
    return target


def relative_state(own, target, speed_unit):
    factor = KNOT_MPS if speed_unit == 'knots' else 1.
    delta = xy((target['longitude'], target['latitude']), (own['longitude'], own['latitude']))
    angle = math.radians(target['targetCourse'])
    velocity = (target['targetSpeed']*factor*math.sin(angle), target['targetSpeed']*factor*math.cos(angle))
    age = target.get('_received_age_s', 0.)
    delta = tuple(delta[i]+velocity[i]*age for i in (0, 1))
    distance = math.hypot(*delta)
    own_angle = math.radians(own.get('targetCourse') or 0.)
    own_v = (own['targetSpeed']*factor*math.sin(own_angle), own['targetSpeed']*factor*math.cos(own_angle))
    rate = sum(delta[i]*(velocity[i]-own_v[i]) for i in (0, 1))/distance if distance else 0.
    return delta, velocity, distance, rate


def progress(task, own=None, target=None, speed_unit='knots'):
    if task.get('code') in TARGET_CODES:
        from .target_tasks import progress as document_progress
        return document_progress(task,own,target,speed_unit)
    params = task['params']
    result = dict(targetPlatformId=params['targetPlatformId'],
                  minFollowingDistanceM=params['minFollowingDistanceM'],
                  maxFollowingDistanceM=params['maxFollowingDistanceM'],
                  targetAvailable=target is not None, targetDistanceM=None,
                  rangeRateMps=None, followingStage='waiting_target')
    if own is not None and target is not None:
        _, _, distance, rate = relative_state(own, target, speed_unit)
        stage = ('too_close' if distance < params['minFollowingDistanceM'] else
                 'too_far' if distance > params['maxFollowingDistanceM'] else 'in_band')
        result.update(targetDistanceM=round(distance, 2), rangeRateMps=round(rate, 3),
                      followingStage=stage)
    return result


def range_facts(motion, delta, velocity):
    # Motion samples include transition boundaries, so use their actual times.
    times = motion.pop('_times')
    relative = [tuple(delta[j]+velocity[j]*t-point[j] for j in (0, 1))
                for t, point in zip(times, motion['path'])]
    minimum, maximum = math.inf, 0.
    for a, b in zip(relative, relative[1:]):
        d = tuple(b[j]-a[j] for j in (0, 1))
        dd = sum(v*v for v in d)
        u = max(0., min(1., -sum(a[j]*d[j] for j in (0, 1))/dd)) if dd else 0.
        minimum = min(minimum, math.hypot(*(a[j]+u*d[j] for j in (0, 1))))
        maximum = max(maximum, math.hypot(*a), math.hypot(*b))
    end = relative[-1]
    distance = math.hypot(*end)
    angle = math.radians(motion['end_course_deg'])
    own_v = (motion['end_speed']*math.sin(angle), motion['end_speed']*math.cos(angle))
    rate = sum(end[j]*(velocity[j]-own_v[j]) for j in (0, 1))/distance if distance else 0.
    return minimum, maximum, distance, rate


def forecast(policy, own, speed, course, duration):
    factor = KNOT_MPS if policy.speed_unit == 'knots' else 1.
    speed_time = policy.execution['speed_transition_time_s']
    course_time = policy.execution['course_transition_time_s']
    dt = .1
    motion = predict_ground_motion(own['targetSpeed']*factor, own['targetCourse'], speed*KNOT_MPS,
        course, speed_time, course_time, duration, dt=dt, **motion_parameters(dict(policy.execution, initial_turn_rate_deg_s=own.get('_turn_rate_deg_s', 0.))))
    motion['_times'] = motion['times_s']
    return motion


def following_action(policy, packet):
    """Compute candidate facts, filter through existing guards, ask Kev to choose."""
    if packet.get('code') in TARGET_CODES:
        from .target_tasks import action
        return action(policy,packet)
    if __package__.startswith('server.'):
        from ..policies.turning import reachable_courses
        from ..policies.kev_policy import round_model_values
    else:
        from policies.turning import reachable_courses
        from policies.kev_policy import round_model_values
    params = packet['params']
    validate_task(packet)
    if params['minFollowingDistanceM'] <= max(policy.obstacle_config.collision_radius_m,
                                                policy.dynamic_config.safe_distance_m):
        raise ValueError('minimum following distance must exceed configured obstacle safety distance')
    records = packet.get('situation', ())
    own = next((r for r in records if r.get('platformId') == params['platCode']), None)
    if own is None:
        raise ValueError('No own vessel situation')
    # Reuse normal own-state validation (also applied by ControlSession at runtime).
    own = dict(own, targetCourse=policy._observation_course(own))
    if (not all(finite(own.get(k)) for k in ('longitude', 'latitude', 'targetSpeed'))
            or own['targetSpeed'] < 0 or abs(own['longitude']) > 180 or abs(own['latitude']) >= 90):
        raise ValueError('Invalid own vessel situation')
    target = target_record(packet, records)
    delta, velocity, distance, _ = relative_state(own, target, policy.speed_unit)
    lower, upper = params['minFollowingDistanceM'], params['maxFollowingDistanceM']
    reference = (lower+upper)/2
    factor = KNOT_MPS if policy.speed_unit == 'knots' else 1.
    target_speed = target['targetSpeed']*factor/KNOT_MPS
    speeds = sorted({0., min(2.5, policy.max_speed), min(5., policy.max_speed),
                     min(target_speed, policy.max_speed), *policy.speeds})
    limit = min(179.99, policy.duration*policy.max_turn_rate_deg_s)
    desired = [math.degrees(math.atan2(delta[0], delta[1])) % 360, target['targetCourse']]
    # Include exact attainable bearings to avoid jitter between coarse bins.
    aligned = [(own['targetCourse']+max(-limit, min(limit,
                (bearing-own['targetCourse']+180)%360-180)))%360 for bearing in desired]
    courses = sorted({round(c, 2) % 360 for c in reachable_courses(
        own['targetCourse'], policy.duration, policy.max_turn_rate_deg_s)+[own['targetCourse']]+aligned})
    criteria = {}
    preview_s = max(10., policy.duration)
    for speed in speeds:
        for j, course in enumerate(courses if speed else [own['targetCourse']]):
            # speed is in knots; wire unit follows the configured protocol.
            wire_speed = round(speed if policy.speed_unit == 'knots' else speed*KNOT_MPS, 2)
            if not policy.validate_control(packet, own, records, wire_speed, course)['safe']:
                continue
            if not policy.validate_dynamic_control(packet, own, records, wire_speed, course)['safe']:
                continue
            actual_knots = wire_speed if policy.speed_unit == 'knots' else wire_speed/KNOT_MPS
            motion = forecast(policy, own, actual_knots, course, policy.duration)
            end_delta = tuple(delta[k]+velocity[k]*policy.duration-motion['endpoint'][k] for k in (0, 1))
            end_bearing = math.degrees(math.atan2(end_delta[0],end_delta[1]))%360
            end_heading_error = abs((end_bearing-motion['end_course_deg']+180)%360-180)
            action = range_facts(motion, delta, velocity)
            preview = range_facts(forecast(policy, own, actual_knots, course, preview_s), delta, velocity)
            closing = max(0., -action[3])
            braking_limit = min([policy.obstacle_config.deceleration_mps2]+[p[1] for p in policy.execution.get('deceleration_curve',())])
            braking = closing*closing/(2*braking_limit)
            criteria[f'speed_{speed:g}__heading_{j:02d}'] = dict(speed_knots=actual_knots,
                absolute_course_deg=course, range_m=list(action[:3]), range_rate_mps=action[3],
                reference_error_m=abs(action[2]-reference), preview_range_m=list(preview[:3]),
                range_reduction_m=distance-action[2],
                band_error_after_m=max(0.,lower-action[2],action[2]-upper),
                preview_reference_error_m=abs(preview[2]-reference),
                end_target_heading_error_deg=end_heading_error,
                radial_braking_margin_m=action[2]-lower-braking,
                target_heading_error_deg=abs((math.degrees(math.atan2(delta[0],delta[1]))-course+180)%360-180))
    recovery = 'unconstrained'
    stopped = {k: c for k, c in criteria.items() if c['speed_knots'] == 0.}
    if target_speed <= policy.obstacle_config.stationary_speed_knots and lower <= distance <= upper and stopped:
        # An in-band stationary target has no tracking velocity to match.
        # A safe stop prevents creeping through the minimum distance.
        criteria, recovery = stopped, 'stationary_hold'
    elif distance > upper:
        # As with waypoint speed stages, safe actions must actually recover the
        # active objective when such an action exists. Otherwise the model can
        # indefinitely match velocity outside the allowed range.
        closing = {k: c for k, c in criteria.items() if c['range_reduction_m'] > .001
                   and c['range_m'][0] >= lower and c['radial_braking_margin_m'] >= 0.}
        bearing = math.degrees(math.atan2(delta[0],delta[1]))%360
        error = abs((bearing-own['targetCourse']+180)%360-180)
        turning = {k: c for k, c in criteria.items() if c['speed_knots'] > 0
                   and c['end_target_heading_error_deg'] < error-.01 and c['range_m'][0] >= lower}
        if error > 90 and turning:
            criteria, recovery = turning, 'reorient'
        elif closing:
            criteria, recovery = closing, 'close_range'
        elif turning:
            criteria, recovery = turning, 'reorient'
    elif distance < lower:
        opening = {k: c for k, c in criteria.items() if c['range_reduction_m'] < -.001
                   and c['range_rate_mps'] > 0}
        if opening:
            criteria, recovery = opening, 'open_range'
    state = dict(task='line_of_sight_distance_following', **progress(packet, own, target, policy.speed_unit),
                 recovery_phase=recovery, reference_distance_m=reference, target_offset_m=list(delta), target_velocity_mps=list(velocity),
                 actual_course_deg=own['targetCourse'], actual_speed_knots=own['targetSpeed']*factor/KNOT_MPS,
                 command_duration_s=policy.duration, preview_duration_s=preview_s, motion_limits=motion_parameters(policy.execution),
                 forecast_assumption='Target constant current velocity; own motion uses motion_limits.acceleration_response; measured_0_50kn is nominal straight acceleration to 50 knots, bounded and other maneuvers use physical limits (12 deg/s at 11 knots; provisional elsewhere). Future target turns unknown.')
    if not criteria:
        policy.local.last_decision = dict(selection_mode='following_safety_hold', state=state, options={}, candidate_count=0)
        return 0., own['targetCourse']
    instructions = (
        'Maintain centre-to-centre range inside minFollowingDistanceM and maxFollowingDistanceM. '
        'Entering the band does not finish the task. Any relative bearing is allowed. '
        'Avoid collisions first, brake early before minimum range; negative range_rate_mps means closing. '
        'If too close, increase separation; zero speed does not stop an incoming target. '
        'If too far, reorient toward target and close range safely. Do not flee merely because target approaches. '
        'Within band match target motion with near-zero range rate; midpoint is a soft preference. '
        'Compare action range_m [min,max,end], end range_rate_mps and radial_braking_margin_m. '
        'preview_range_m assumes holding the command; a preview violation is not an action violation. '
        'A useful turn can initially increase range. Keep a stable useful turn; zero speed cannot turn. '
        'Target stopped: settle within band. Choose only from the supplied safe candidates.')
    if distance > upper:
        instructions = (
            'CURRENTLY TOO FAR: restore the allowed range NOW. Matching target velocity at this distance '
            'freezes the violation and is not successful following. Prefer safe positive range_reduction_m '
            'and smaller band_error_after_m. If heading toward target, accelerate above matching speed '
            'when safe, then brake as the allowed band approaches. A large positive radial_braking_margin_m '
            'means room remains to catch up. A violation only in a long held-command preview is a reason '
            'to replan/brake later, not to keep matching speed forever outside maximum. '
            'If misaligned, make a useful nonzero-speed turn before accelerating away. ' + instructions)
    elif distance < lower:
        instructions = (
            'CURRENTLY TOO CLOSE: restore minimum spacing. Matching velocity freezes this violation. '
            'Compare end range and range rate: actively open separation. If the target moves away, '
            'slowing may help; an incoming target may require a safe escape instead of parking. ' + instructions)
    payload = round_model_values(dict(model='kev-latest', state=state,
        questions=dict(action=dict(type='choice', instructions=instructions, criteria=criteria))))
    started = time.perf_counter()
    result = policy._call(payload)
    if isinstance(result, dict) and result.get('truncated'):
        raise RuntimeError('Kev truncated following input')
    answers = result.get('answers') if isinstance(result, dict) else None
    answer = answers.get('action') if isinstance(answers, dict) else None
    if not isinstance(answer, dict) or answer.get('type') != 'choice' or answer.get('choice') not in criteria:
        raise ValueError('Expected Kev Choice among following candidates')
    chosen = criteria[answer['choice']]
    policy.local.last_decision = dict(selection_mode='target_following', choice=answer['choice'],
        state=payload['state'], options=payload['questions']['action']['criteria'], candidate_count=len(criteria),
        inference_s=time.perf_counter()-started, result=result)
    return chosen['speed_knots']*(KNOT_MPS if policy.speed_unit == 'mps' else 1.), chosen['absolute_course_deg']
