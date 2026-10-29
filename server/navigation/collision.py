"""Task-ending proximity detection from reported positions, not forecasts."""
import math


class CollisionDetected(ValueError):
    error_code = 'COLLISION_DETECTED'

    def __init__(self, target_id, kind, distance_m, threshold_m):
        self.target_id = target_id
        self.kind = kind
        self.distance_m = distance_m
        self.threshold_m = threshold_m
        label = {'static':'静态','dynamic':'动态','unknown':'运动信息未知的'}[kind]
        super().__init__(f'碰撞判定：本艇进入{label}目标{target_id}的避碰范围，'
                         f'实测中心距离{distance_m:.2f}米，判定阈值{threshold_m:.2f}米')


def _finite(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


def check_collision(position_m, frame, records, *, own_id, speed_unit,
                    stationary_speed_knots, static_radius_m, dynamic_radius_m, max_target_age_s):
    """Raise on an occupied safety region; stale moving reports only predict risk.

    Fixed stationary records retain their existing persistence. Moving/unknown
    records require a valid fresh receive age. Neither age extrapolation nor
    forecast uncertainty is used as evidence of an actual threshold crossing.
    """
    factor = 1852./3600. if speed_unit=='knots' else 1.
    latest = {r.get('platformId'):r for r in records}
    for pid,record in latest.items():
        if pid==own_id or type(pid) is not int:
            continue
        lon,lat=record.get('longitude'),record.get('latitude')
        if not (_finite(lon) and _finite(lat) and -180<=lon<=180 and -90<lat<90):
            continue
        speed=record.get('targetSpeed')
        known_speed=_finite(speed) and speed>=0
        stationary=known_speed and speed*factor<=stationary_speed_knots*(1852./3600.)
        kind='static' if stationary else ('dynamic' if known_speed else 'unknown')
        if not stationary:
            age=record.get('_received_age_s',0.)
            if not _finite(age) or not 0<=age<=max_target_age_s:
                continue
        threshold=static_radius_m if stationary else dynamic_radius_m
        distance=math.dist(position_m,frame.project(lon,lat))
        if distance<=threshold:
            raise CollisionDetected(pid,kind,distance,threshold)
