"""Extract a repeatable, monotone nominal 50-knot response from INS telemetry.

This is a target-speed controller response, never a maximum capability curve.
No position or platform identifiers are copied into the output.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

KNOT_MPS = 1852/3600


def calibrate(source):
    source = Path(source)
    with source.open(encoding='utf-8-sig', newline='') as stream:
        reader = csv.reader(stream)
        columns = next(reader)
        # Duplicate 时间 columns: the last one retains fractional seconds.
        time_index = max(i for i, name in enumerate(columns) if name == '时间')
        speed_index = columns.index('目标绝对速度')
        records = list(reader)
    times, speeds = [], []
    rollover = 0.
    for row in records:
        parts = list(map(float, row[time_index].split(':')))
        if len(parts) not in (2, 3):
            raise ValueError('expected mm:ss.s or hh:mm:ss.s sample time')
        clock = sum(value*60**i for i, value in enumerate(reversed(parts)))
        if times and clock+rollover < times[-1]:
            period = 3600. if len(parts) == 2 else 86400.
            if clock+rollover-times[-1] > -period/2:
                raise ValueError('sample time moved backwards without a clock rollover')
            rollover += period
        stamp = clock+rollover
        speed = float(row[speed_index])
        if not math.isfinite(stamp) or not math.isfinite(speed) or speed < 0:
            raise ValueError('finite sample times and nonnegative knot speeds required')
        if times and stamp <= times[-1]:
            raise ValueError('sample times must be strictly increasing')
        times.append(stamp); speeds.append(speed)
    if len(times) < 3 or not 0 < speeds[0] < .5 or max(speeds) < 50.:
        raise ValueError('recording must start near rest and reach 50 knots')
    origin = times[0]
    times = [stamp-origin for stamp in times]
    points = [[0., speeds[0]*KNOT_MPS]]
    for index in range(1, 101):
        threshold = .5*index
        j = next(j for j, value in enumerate(speeds) if value >= threshold)
        fraction = (threshold-speeds[j-1])/(speeds[j]-speeds[j-1])
        stamp = times[j-1]+fraction*(times[j]-times[j-1])
        points.append([stamp, threshold*KNOT_MPS])
    # Explicit near-zero extrapolation only, from the first observed interval.
    first_acceleration = (points[1][1]-points[0][1])/(points[1][0]-points[0][0])
    points.insert(0, [-points[0][1]/first_acceleration, 0.])
    max_acceleration = max((v1-v0)/(t1-t0) for (t0,v0),(t1,v1) in zip(points, points[1:]))
    if max_acceleration > 1.46+1e-9:
        raise ValueError('fit exceeds 1.46 m/s²; inspect input rather than treating noise as capability')
    return dict(schema_version=1, id='measured_0_50kn', kind='observed_controller_response',
                command='50-knot target speed; lower controller accelerated the vessel',
                maneuver='straight', target_speed_knots=50.,
                source_file=source.name, source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                source_samples=len(records), observed_duration_s=times[-1],
                observed_start_speed_mps=speeds[0]*KNOT_MPS, first_50_knots_time_s=points[-1][0],
                speed_unit='m/s', time_unit='s',
                method='First-passage time at each 0.5-knot threshold; linear interpolation between thresholds',
                near_zero_extrapolation='Below first observed speed only; uses first interval slope',
                maximum_segment_acceleration_mps2=max_acceleration,
                applicability='Straight acceleration to 50 knots only; not a maximum acceleration envelope',
                points=points)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = calibrate(args.source)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({key: result[key] for key in ('source_samples', 'first_50_knots_time_s', 'maximum_segment_acceleration_mps2')}))


if __name__ == '__main__':
    main()
