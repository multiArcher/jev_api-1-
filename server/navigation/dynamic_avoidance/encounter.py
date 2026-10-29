"""Early encounter activation from relative motion, rather than range alone."""
import math
from .prediction import velocity,signed_turn
from .types import finite
from tools.vessel_dynamics import turn_profile


def encounter_window(scene, desired, config):
    own,targets=scene
    speed=max(own.speed_mps,desired.speed_mps)
    braking=min([config.deceleration_mps2,config.controller_deceleration_mps2]
                +[p[1] for p in config.deceleration_curve])
    rate=min([config.max_turn_rate_deg_s]+[p[1] for p in config.turn_rate_curve])
    _,turns=turn_profile(30.,own.turn_rate_deg_s,rate,config.max_turn_acceleration_deg_s2)
    turn_time=max(turns,default=0.)
    lateral_time=(config.safe_distance_m+20.)/max(.5*speed,1.)+turn_time
    turn_window=config.control_delay_s+lateral_time+config.early_avoidance_margin_s
    horizon=min(config.warning_horizon_s,max(config.action_horizon_s,
        config.control_delay_s+max(speed/braking,lateral_time)+config.early_avoidance_margin_s))
    own_velocity=velocity(own.speed_mps,own.course_deg)
    encounters=[]
    for target in targets:
        if (target.position_m is None or not finite(target.speed_mps) or
                not finite(target.course_deg) or target.age_s>config.max_target_age_s or
                abs(signed_turn(own.course_deg,target.course_deg))<150.):
            continue
        tv=velocity(target.speed_mps,target.course_deg)
        p=tuple(target.position_m[i]+tv[i]*target.age_s-own.position_m[i] for i in (0,1))
        rv=tuple(tv[i]-own_velocity[i] for i in (0,1))
        norm=sum(v*v for v in rv)
        dot=sum(p[i]*rv[i] for i in (0,1))
        if norm<1e-8 or dot>=0:
            continue
        time=-dot/norm
        closest=math.hypot(*(p[i]+rv[i]*time for i in (0,1)))
        radius=config.safe_distance_m+20.+config.uncertainty_mps*(target.age_s+time)
        if time<=horizon and closest<=radius:
            entry=max(0.,time-math.sqrt(max(0.,radius*radius-closest*closest)/norm))
            encounters.append(dict(target_id=target.target_id,time_to_cpa_s=time,
                time_to_safety_region_s=entry,turn_trigger_s=turn_window,
                phase='maneuver' if entry<=turn_window else 'prepare',
                distance_at_cpa_m=closest,closing_speed_mps=-dot/max(math.hypot(*p),1.),
                safe_distance_at_cpa_m=radius))
    return (horizon if encounters else config.action_horizon_s),encounters
