"""Explicit conversion from protocol snapshots; never reads a live store."""
import math
from tools.waypoint_geometry import KNOT_MPS, xy
from .types import Target, Vessel, finite, valid_point


def scene_from_situation(records, *, origin_lon_lat, speed_unit, own_id, max_own_age_s=5.):
    """Return (Vessel, tuple[Target]); preserve bad target data as unknown.

    Ages are local receive ages, not unsynchronised protocol timestamps.
    Duplicate IDs use the last supplied record, as in an incremental store.
    """
    if speed_unit not in ('knots', 'mps'):
        raise ValueError('speed_unit must be knots or mps')
    if (not valid_point(origin_lon_lat) or abs(origin_lon_lat[0]) > 180
            or abs(origin_lon_lat[1]) >= 90):
        raise ValueError('invalid coordinate origin')
    if type(own_id) is not int or not finite(max_own_age_s) or max_own_age_s < 0:
        raise ValueError('invalid own_id or max_own_age_s')
    factor = KNOT_MPS if speed_unit == 'knots' else 1.
    latest = {}
    for record in records:
        pid = record.get('platformId')
        if type(pid) is not int:
            raise ValueError('platformId must be an integer; cannot silently discard target')
        latest[pid] = record
    if own_id not in latest:
        raise ValueError('own vessel missing')
    own = None
    targets = []
    for pid, record in latest.items():
        lon, lat = record.get('longitude'), record.get('latitude')
        position = xy((lon, lat), origin_lon_lat) if (finite(lon) and finite(lat)
                    and abs(lon) <= 180 and abs(lat) < 90) else None
        speed, course = record.get('targetSpeed'), record.get('targetCourse')
        speed = speed*factor if finite(speed) and speed >= 0 else None
        course = course if finite(course) and 0 <= course < 360 else None
        age = record.get('_received_age_s', 0.)
        if not finite(age) or age < 0:
            # Invalid receive age makes a target unlocatable at the present.
            position, age = None, 0.
        if pid == own_id:
            if position is None or speed is None or course is None or age > max_own_age_s:
                raise ValueError('own vessel needs fresh position, speed and course')
            angle = math.radians(course)
            own = Vessel((position[0]+speed*age*math.sin(angle),
                          position[1]+speed*age*math.cos(angle)), speed, course)
        else:
            targets.append(Target(pid, position, speed, course, age))
    return own, tuple(sorted(targets, key=lambda t: t.target_id))
