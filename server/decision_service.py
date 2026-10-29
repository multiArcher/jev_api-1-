"""UDP decision service with a replaceable policy."""

from __future__ import annotations

import argparse
import os
import datetime
import json
import logging
import math
import sys
from pathlib import Path
import re
import socket
import struct
import threading
import time
from typing import Any, Callable, Dict, Mapping, Optional, Tuple


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "jev_api"))
from tools.waypoint_geometry import LocalFrame
if __package__:
    from .navigation import target_following
    from .control_runtime import error_details, run_task_control
    from .policies.waypoint_radius import waypoint_arrival_radius
    from .jev_policy import JevPolicy, jev_policy
    from .policies.factory import DEFAULT_BACKEND, create_policy
    from .policies.turning import DEFAULT_MAX_TURN_RATE_DEG_S
    from .policies.static_obstacles import StaticObstacleConfig, configure_actual_distance_log
else:
    from navigation import target_following
    from control_runtime import error_details, run_task_control
    from policies.waypoint_radius import waypoint_arrival_radius
    from jev_policy import JevPolicy, jev_policy
    from policies.factory import DEFAULT_BACKEND, create_policy
    from policies.turning import DEFAULT_MAX_TURN_RATE_DEG_S
    from policies.static_obstacles import StaticObstacleConfig, configure_actual_distance_log


LOGGER = logging.getLogger("decision_service")
LOCAL_CONFIG_PATH = Path(__file__).resolve().with_name("decision_service.local.json")
DEFAULT_JEV_ENDPOINT = "https://api.jevcn.cn/v1/systemone"
DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 18014
DEFAULT_PLATFORM_ID = 1010
DEFAULT_BUFFER_SIZE = 65_507
DEFAULT_SITUATION_GROUP = "239.255.0.10"
DEFAULT_SITUATION_PORT = 18010
MULTICAST_RETRY_INTERVAL = 3.0
SITUATION_LOG_INTERVAL = 5.0
DEFAULT_ACTION_HOST = "192.168.3.201"       
DEFAULT_ACTION_PORT = 18123
DEFAULT_MAX_CONSECUTIVE_ERRORS = 3
DEFAULT_COMMAND_HOLD_TIMEOUT = 5.0
FIXED_HEADER_SIZE = 12
HEADER_SIZE = 14  # JSON fragmented-message header
JSON_PREFIX_SIZE = 2
ALGORITHM_TASK_ID = 0x0660
SPEED_COURSE_ID = 0x0662
ALGORITHM_STATUS_ID = 0x0664
ALGORITHM_STOP_ID = 0x0666
SITUATION_ID = 0x0060
JSON_INFO_UNIT_IDS = (ALGORITHM_TASK_ID, ALGORITHM_STATUS_ID, ALGORITHM_STOP_ID)


Policy = Callable[[Mapping[str, Any]], Tuple[float, float]]
EARTH_RADIUS_M = 6_378_137.0
KNOT_TO_MPS = 1852.0 / 3600.0


def route_distance_m(first: Mapping[str, Any], second: Mapping[str, Any]) -> float:
    """Return great-circle distance between two lon/lat records in metres."""

    lon1, lat1 = math.radians(float(first["routeLon"])), math.radians(float(first["routeLat"]))
    lon2, lat2 = math.radians(float(second["routeLon"])), math.radians(float(second["routeLat"]))
    dlon, dlat = lon2 - lon1, lat2 - lat1
    value = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(value)))


def calculate_route_progress(
    task: Mapping[str, Any],
    state: Optional[Mapping[str, Any]],
    current_route: int,
    speed_unit: str = "knots",
    start_state: Optional[Mapping[str, Any]] = None,
    initial_route: int = 0,
) -> Dict[str, Any]:
    """Build behavior-tree progress fields from the task and latest own-ship state."""

    params = task.get("params")
    waypoints = params.get("planWaypointList") if isinstance(params, Mapping) else None
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError("task has no planWaypointList")
    if speed_unit not in ("knots", "mps"):
        raise ValueError("speed_unit must be knots or mps")

    if type(current_route) is not int or not 0 <= current_route <= len(waypoints):
        raise ValueError("currentRoute must be the completed waypoint count in [0, planWaypointNum]")
    next_index = current_route
    total_distance = sum(
        route_distance_m(first, second)
        for first, second in zip(waypoints[initial_route:], waypoints[initial_route + 1:])
    )
    if start_state is not None and initial_route < len(waypoints):
        total_distance += route_distance_m(
            {"routeLon": start_state["longitude"], "routeLat": start_state["latitude"]},
            waypoints[initial_route],
        )
    result: Dict[str, Any] = {
        "nextWaypointIndex": next_index if next_index < len(waypoints) else None,
        "nextWaypoint": dict(waypoints[next_index]) if next_index < len(waypoints) else None,
        "remainingWaypointNum": max(0, len(waypoints) - next_index),
        "remainingDistanceM": None,
        "estimatedRemainingTimeS": None,
        "completionRate": None,
    }
    if state is None or next_index >= len(waypoints):
        if next_index >= len(waypoints):
            result.update(remainingDistanceM=0.0, estimatedRemainingTimeS=0.0, completionRate=100.0)
        return result

    current_position = {
        "routeLon": state["longitude"],
        "routeLat": state["latitude"],
    }
    current_leg = route_distance_m(current_position, waypoints[next_index])
    final_radius = waypoint_arrival_radius(waypoints[next_index])
    if next_index == len(waypoints) - 1 and current_leg <= final_radius:
        result.update(
            nextWaypointIndex=None,
            nextWaypoint=None,
            remainingWaypointNum=0,
            remainingDistanceM=0.0,
            estimatedRemainingTimeS=0.0,
            completionRate=100.0,
        )
        return result

    later_legs = [
        route_distance_m(first, second)
        for first, second in zip(waypoints[next_index:], waypoints[next_index + 1 :])
    ]
    remaining_distance = current_leg + sum(later_legs)
    completion = 1.0 if total_distance <= 1e-9 and remaining_distance <= 1e-9 else (
        0.0 if total_distance <= 1e-9 else 1.0 - remaining_distance / total_distance
    )
    completion = max(0.0, min(1.0, completion))

    factor = KNOT_TO_MPS if speed_unit == "knots" else 1.0
    current_speed = state.get("targetSpeed")
    eta: Optional[float]
    if current_leg > 1e-9 and (
        current_speed is None or not math.isfinite(float(current_speed)) or float(current_speed) <= 0
    ):
        eta = None
    else:
        eta = 0.0 if current_leg <= 1e-9 else current_leg / (float(current_speed) * factor)
        for leg_distance, destination in zip(later_legs, waypoints[next_index + 1 :]):
            planned_speed = destination.get("routeSpeed")
            if leg_distance > 1e-9 and (
                planned_speed is None
                or not math.isfinite(float(planned_speed))
                or float(planned_speed) <= 0
            ):
                eta = None
                break
            if eta is not None and leg_distance > 1e-9:
                eta += leg_distance / (float(planned_speed) * factor)

    result.update(
        remainingDistanceM=round(remaining_distance, 2),
        estimatedRemainingTimeS=None if eta is None else round(eta, 2),
        completionRate=round(completion * 100.0, 2),
    )
    return result


def protocol_timestamp() -> int:
    """Return UTC milliseconds since midnight times 10 (LSB is 0.1 ms)."""

    now = datetime.datetime.now(datetime.timezone.utc)
    midnight = now.replace(hour=0, minute=0, second=0, microsecond=0)
    return int((now - midnight).total_seconds() * 10_000) & 0xFFFFFFFF


def speed_course_timestamp() -> int:
    """Return milliseconds since midnight in Asia/Shanghai for 0x0662."""

    china_tz = datetime.timezone(datetime.timedelta(hours=8))
    now = datetime.datetime.now(china_tz)
    midnight = now.replace(hour=0, minute=0, second=0, microsecond=0)
    return int((now - midnight).total_seconds() * 1_000) & 0xFFFFFFFF


def build_header(
    total_length: int,
    secondary_id: int,
    timestamp: Optional[int] = None,
    total_packets: int = 1,
    packet_index: int = 1,
    fragmented: bool = True,
) -> bytes:
    """Build a 12-byte fixed or 14-byte fragmented big-endian header."""

    if not 1 <= total_packets <= 10:
        raise ValueError("total_packets must be in [1, 10]")
    if not 1 <= packet_index <= total_packets:
        raise ValueError("packet_index must be in [1, total_packets]")
    if not 0 <= total_length <= 0xFFFF:
        raise ValueError("packet length exceeds uint16")
    stamp = protocol_timestamp() if timestamp is None else timestamp
    common = struct.pack(
        ">BBHIHH",
        0,
        0,
        total_length,
        stamp,
        0,
        secondary_id,
    )
    if fragmented:
        return common + struct.pack(">BB", total_packets, packet_index)
    return common


def parse_header(payload: bytes) -> Dict[str, int]:
    """Validate and return fields from a common protocol header."""

    if len(payload) < FIXED_HEADER_SIZE:
        raise ValueError("datagram is shorter than the 12-byte common header")
    seq, unit_id, length, timestamp, reserved, secondary_id = struct.unpack(
        ">BBHIHH", payload[:FIXED_HEADER_SIZE]
    )
    if length != len(payload):
        raise ValueError(
            "header length {} does not match datagram length {}".format(
                length, len(payload)
            )
        )
    if reserved != 0:
        raise ValueError("reserved field must be 0")
    if secondary_id in JSON_INFO_UNIT_IDS:
        if seq != 0 or unit_id != 0:
            raise ValueError(
                "algorithm JSON infoUnitSeq and infoUnitId must both be 0"
            )
        if len(payload) < HEADER_SIZE:
            raise ValueError("fragmented JSON header must be 14 bytes")
        total, index = struct.unpack(">BB", payload[FIXED_HEADER_SIZE:HEADER_SIZE])
        if not 1 <= total <= 10 or not 1 <= index <= total:
            raise ValueError("invalid fragment counters")
    else:
        total, index = 1, 1
    return {
        "infoUnitSeq": seq,
        "infoUnitId": unit_id,
        "timestamp": timestamp,
        "secondaryInfoUnitId": secondary_id,
        "totalPackets": total,
        "packetIndex": index,
    }


def build_json_packet(
    content: Mapping[str, Any],
    secondary_id: int,
    timestamp: Optional[int] = None,
) -> bytes:
    """Encode an unfragmented JSON message using the Excel wire format."""

    body = json.dumps(content, ensure_ascii=False, separators=(",", ":")).encode(
        "utf-8"
    )
    if len(body) > 0xFFFF:
        raise ValueError("JSON content exceeds uint16 contentLength")
    total_length = HEADER_SIZE + JSON_PREFIX_SIZE + len(body)
    return (
        build_header(total_length, secondary_id, timestamp=timestamp)
        + struct.pack(">H", len(body))
        + body
    )


def build_algorithm_status_packet(
    observation: Mapping[str, Any],
    status: int,
    description: str,
    params: Optional[Mapping[str, Any]] = None,
) -> bytes:
    """Encode an Excel-defined 0x0664 algorithm execution status."""

    if status not in (1, 2, 3):
        raise ValueError("algorithm status must be 1, 2, or 3")
    content = {
        "simId": observation.get("simId"),
        "id": observation.get("id"),
        "code": observation.get("code"),
        "status": status,
        "description": description,
        "params": dict(params or {}),
    }
    return build_json_packet(content, ALGORITHM_STATUS_ID)


def analyze_task(payload: bytes) -> Dict[str, Any]:
    """Decode a UTF-8 JSON, framed binary, hexadecimal, or text datagram."""

    if not payload:
        raise ValueError("empty UDP datagram")

    if len(payload) >= HEADER_SIZE and payload[:2] == b"\x00\x00":
        header = parse_header(payload)
        if header["secondaryInfoUnitId"] not in JSON_INFO_UNIT_IDS:
            raise ValueError(
                "unsupported JSON secondaryInfoUnitId 0x{:04X}".format(
                    header["secondaryInfoUnitId"]
                )
            )
        if header["totalPackets"] != 1:
            raise ValueError("fragmented JSON is not yet reassembled")
        if len(payload) < HEADER_SIZE + JSON_PREFIX_SIZE:
            raise ValueError("first JSON packet has no contentLength")
        content_length = struct.unpack(">H", payload[HEADER_SIZE:16])[0]
        payload = payload[16:]
        if content_length != len(payload):
            raise ValueError(
                "contentLength {} does not match JSON length {}".format(
                    content_length, len(payload)
                )
            )

    text = payload.decode("utf-8").strip()
    compact_hex = re.sub(r"(?:0x)|[\s:-]", "", text, flags=re.IGNORECASE)
    if compact_hex and len(compact_hex) % 2 == 0 and re.fullmatch(
        r"[0-9a-fA-F]+", compact_hex
    ):
        try:
            decoded = bytes.fromhex(compact_hex).decode("utf-8").strip()
            if decoded:
                text = decoded
        except (UnicodeDecodeError, ValueError):
            pass

    try:
        value = json.loads(text)
    except json.JSONDecodeError:
        return {"raw": text}

    if isinstance(value, dict):
        return value
    return {"value": value}


class FragmentReassembler:
    """Reassemble fragmented 0x0660/0x0664/0x0666 UTF-8 JSON messages."""

    def __init__(self, timeout: float = 3.0) -> None:
        if timeout <= 0:
            raise ValueError("reassembly timeout must be positive")
        self.timeout = timeout
        self._pending: Dict[Tuple[Any, ...], Dict[str, Any]] = {}

    def add(self, payload: bytes, remote: Tuple[str, int]) -> Optional[bytes]:
        header = parse_header(payload)
        if header["secondaryInfoUnitId"] not in JSON_INFO_UNIT_IDS:
            return payload
        if header["totalPackets"] == 1:
            return payload

        now = time.monotonic()
        self._discard_expired(now)
        index = header["packetIndex"]
        if index == 1:
            if len(payload) < HEADER_SIZE + JSON_PREFIX_SIZE:
                raise ValueError("first JSON fragment has no contentLength")
            content_length = struct.unpack(">H", payload[HEADER_SIZE:16])[0]
            fragment = payload[16:]
        else:
            content_length = None
            fragment = payload[HEADER_SIZE:]

        key = (
            remote,
            header["secondaryInfoUnitId"],
            header["timestamp"],
            header["totalPackets"],
        )
        entry = self._pending.setdefault(
            key,
            {
                "created": now,
                "contentLength": None,
                "fragments": {},
            },
        )
        if content_length is not None:
            previous = entry["contentLength"]
            if previous is not None and previous != content_length:
                raise ValueError("conflicting fragmented JSON contentLength")
            entry["contentLength"] = content_length
        previous_fragment = entry["fragments"].get(index)
        if previous_fragment is not None and previous_fragment != fragment:
            raise ValueError("conflicting duplicate JSON fragment")
        entry["fragments"][index] = fragment

        if entry["contentLength"] is None or len(entry["fragments"]) < header["totalPackets"]:
            return None
        body = b"".join(
            entry["fragments"][i] for i in range(1, header["totalPackets"] + 1)
        )
        del self._pending[key]
        if len(body) != entry["contentLength"]:
            raise ValueError(
                "reassembled JSON length {} does not match contentLength {}".format(
                    len(body), entry["contentLength"]
                )
            )
        return body

    def _discard_expired(self, now: float) -> None:
        for key, entry in tuple(self._pending.items()):
            if now - entry["created"] > self.timeout:
                del self._pending[key]


def decode_situation_packet(payload: bytes) -> Dict[str, Any]:
    """Decode an Excel-defined 0x0060 force-fusion situation datagram."""

    header = parse_header(payload)
    if header["secondaryInfoUnitId"] != SITUATION_ID:
        raise ValueError("packet is not force-fusion situation information")
    if len(payload) != FIXED_HEADER_SIZE + 28:
        raise ValueError("force-fusion situation packet must be 40 bytes")
    (
        direction,
        latitude_raw,
        longitude_raw,
        platform_id,
        sign,
        target_course_raw,
        height,
        target_speed_raw,
        target_type,
        target_type_confid,
        threat_level,
        batch,
    ) = struct.unpack(">hiiHHHhhHBBI", payload[FIXED_HEADER_SIZE:])
    return {
        "direction": direction,
        "latitude": latitude_raw * (90.0 / (2**31)),
        "longitude": longitude_raw * (180.0 / (2**31)),
        "platformId": platform_id,
        "sign": sign,
        "targetCourse": None if target_course_raw == 0xFFFF else target_course_raw,
        "height": height,
        "targetSpeed": None if target_speed_raw == -1 else target_speed_raw * 0.1,
        "type": target_type,
        "targetTypeConfid": (
            None if target_type_confid == 0xFF else target_type_confid
        ),
        "threatLevel": None if threat_level == 0xFF else threat_level,
        "batch": batch,
        "timestamp": header["timestamp"],
    }


def build_situation_packet(record: Mapping[str, Any]) -> bytes:
    """Encode a 0x0060 situation record for local integration testing."""

    latitude_raw = round(float(record["latitude"]) / (90.0 / (2**31)))
    longitude_raw = round(float(record["longitude"]) / (180.0 / (2**31)))
    target_course = record.get("targetCourse")
    target_speed = record.get("targetSpeed")
    body = struct.pack(
        ">hiiHHHhhHBBI",
        int(record["direction"]),
        latitude_raw,
        longitude_raw,
        int(record["platformId"]),
        int(record["sign"]),
        0xFFFF if target_course is None else int(target_course),
        int(record["height"]),
        -1 if target_speed is None else round(float(target_speed) * 10),
        int(record["type"]),
        0xFF if record.get("targetTypeConfid") is None else int(record["targetTypeConfid"]),
        0xFF if record.get("threatLevel") is None else int(record["threatLevel"]),
        int(record["batch"]),
    )
    return (
        build_header(
            FIXED_HEADER_SIZE + len(body),
            SITUATION_ID,
            timestamp=speed_course_timestamp(),
            fragmented=False,
        )
        + body
    )


class SituationLogThrottle:
    """Summarize receipt every five seconds without slowing telemetry updates."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._last_log_at = None
        self._count = 0
        self._platforms = set()

    def received(self, state: Mapping[str, Any], remote: Tuple[str, int]) -> None:
        with self._lock:
            now = time.monotonic()
            self._count += 1
            self._platforms.add(state["platformId"])
            if self._last_log_at is not None and now - self._last_log_at < SITUATION_LOG_INTERVAL:
                return
            LOGGER.info("received situation packets=%d platforms=%s; latest from %s:%d",
                        self._count, sorted(self._platforms), *remote)
            self._last_log_at = now
            self._count = 0
            self._platforms.clear()


class SituationStore:
    """Keep the latest situation record for each platform."""

    def __init__(self) -> None:
        self._records: Dict[int, Tuple[Dict[str, Any], float]] = {}
        self._lock = threading.Lock()
        self._turn_rates = {}

    def update(self, record: Mapping[str, Any]) -> None:
        platform_id = record.get("platformId")
        if not isinstance(platform_id, int):
            raise ValueError("situation platformId must be an integer")
        with self._lock:
            now = time.monotonic()
            previous = self._records.get(platform_id)
            rate = record.get('_turn_rate_deg_s', 0.)
            if '_turn_rate_deg_s' not in record and previous:
                old, received = previous
                elapsed = now-received
                before, after = old.get('targetCourse'), record.get('targetCourse')
                if .05 <= elapsed <= 5. and all(isinstance(v,(int,float)) and math.isfinite(v) for v in (before,after)):
                    rate = max(-12.,min(12.,((after-before+180)%360-180)/elapsed))
                elif elapsed < .05:
                    rate = self._turn_rates.get(platform_id,0.)
            self._turn_rates[platform_id] = rate
            self._records[platform_id] = (dict(record), now)

    def query_state(self, *, include_age: bool = False) -> Tuple[Dict[str, Any], ...]:
        """Return a stable snapshot for policy input."""

        with self._lock:
            now = time.monotonic()
            return tuple(dict(self._records[key][0], **(
                {'_received_age_s': max(0., now-self._records[key][1]), '_turn_rate_deg_s':self._turn_rates.get(key,0.)} if include_age else {}))
                for key in sorted(self._records))

    def get(self, platform_id: int, max_age: float) -> Optional[Dict[str, Any]]:
        """Return a fresh platform record, or None if absent/stale."""

        with self._lock:
            item = self._records.get(platform_id)
            if item is None or time.monotonic() - item[1] > max_age:
                return None
            return dict(item[0], _turn_rate_deg_s=self._turn_rates.get(platform_id,0.))


class ControlSession:
    """Mutable state shared by one task's control and status-report threads."""

    def __init__(
        self,
        task: Mapping[str, Any],
        sender: Tuple[str, int],
        speed_unit: str = "knots",
    ) -> None:
        self.task = dict(task)
        self.sender = sender
        self.stop_requested = threading.Event()
        self.finished = threading.Event()
        self.superseded = threading.Event()
        self.output_lock = threading.RLock()
        self.status_lock = threading.Lock()
        self._lock = threading.Lock()
        params = task.get("params")
        self.is_following = target_following.is_target_task(task)
        if not isinstance(params, Mapping) or not (task.get("code")=="trackingMode" or self.is_following):
            raise ValueError("expected trackingMode, followMode, accompanyMode or targetFollowing with object params")
        plat_code = params.get("platCode")
        if type(plat_code) is not int or not 0 <= plat_code <= 0xFFFFFFFF:
            raise ValueError("params.platCode must be an integer in [0, 4294967295]")
        self.start_state: Optional[Dict[str, Any]] = None
        self.latest_target = None
        if self.is_following:
            target_following.validate_task(task)
            self.waypoints = []
            self.current_route = self.initial_route = 0
        else:
            self.waypoints = params.get("planWaypointList")
            if (not isinstance(self.waypoints, list) or not self.waypoints
                    or type(params.get("planWaypointNum")) is not int
                    or params.get("planWaypointNum") != len(self.waypoints)):
                raise ValueError("planWaypointNum must match a nonempty planWaypointList")
            for index, point in enumerate(self.waypoints):
                if not isinstance(point, Mapping):
                    raise ValueError("waypoint {} must be an object".format(index + 1))
                for field in ("routeLon", "routeLat", "routeSpeed"):
                    value = point.get(field)
                    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                        raise ValueError("waypoint {} has invalid {}".format(index + 1, field))
                if (not -180 <= point["routeLon"] <= 180 or not -90 <= point["routeLat"] <= 90
                        or point["routeSpeed"] < 0):
                    raise ValueError("waypoint {} has invalid coordinates or speed".format(index + 1))
            self.frame = LocalFrame(self.waypoints)
            self.waypoint_positions = [self.frame.project(p['routeLon'], p['routeLat']) for p in self.waypoints]
            self.current_route = params.get("next_waypoint_index", params.get("currentRoute", 0))
            self.initial_route = self.current_route
        self.phase = "tracking"
        self.description = ("伴随中" if task.get("code")=="accompanyMode" else "跟随中" if task.get("code")=="followMode" else "tracking")
        self.errors: Dict[str, Any] = {}
        self.consecutive_errors = 0
        self.stop_command_sent = False
        self.stop_confirmed = False
        self.speed_unit = speed_unit
        self.latest_state: Optional[Dict[str, Any]] = None
        self.progress = (target_following.progress(self.task) if self.is_following else
                         calculate_route_progress(self.task, None, self.current_route, speed_unit))

    @property
    def key(self) -> Tuple[Any, Any]:
        return self.task.get("simId"), self.task.get("id")

    def request_stop(self) -> None:
        with self._lock:
            self.phase = "stopping"
            self.description = "收到停止请求，正在下发零速并等待态势确认停车"
        self.stop_requested.set()

    def supersede(self, replacement: "ControlSession") -> None:
        """Called with the boat's output lock held; never sends a stop command."""
        self.superseded.set()
        self.set_phase("superseded", "任务已被新任务{}覆盖，控制权已移交".format(
            replacement.task.get("id")))
        self.finished.set()
        self.stop_requested.set()  # Wake the old loop without waiting for its next tick.

    def update_route(self, current_route: int) -> None:
        if self.is_following:
            return
        with self._lock:
            self.current_route = max(self.current_route, int(current_route))
            self.progress = calculate_route_progress(
                self.task, self.latest_state, self.current_route, self.speed_unit,
                self.start_state, self.initial_route,
            )

    def update_situation(self, state: Mapping[str, Any]) -> None:
        for field, lower, upper in (("longitude", -180, 180), ("latitude", -90, 90),
                                    ("targetSpeed", 0, float("inf"))):
            value = state.get(field)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or not lower <= value <= upper):
                raise ValueError("invalid situation {}".format(field))
        with self._lock:
            if self.start_state is None:
                self.start_state = dict(state)
            self.latest_state = dict(state)
            if self.is_following:
                self.progress = target_following.progress(self.task, state, self.latest_target, self.speed_unit)
                return
            position = self.frame.project(state['longitude'], state['latitude'])
            while self.current_route < len(self.waypoints):
                point = self.waypoints[self.current_route]
                if math.dist(position, self.waypoint_positions[self.current_route]) > waypoint_arrival_radius(point):
                    break
                self.current_route += 1
            self.progress = calculate_route_progress(
                self.task, self.latest_state, self.current_route, self.speed_unit,
                self.start_state, self.initial_route,
            )

    def update_target(self, records):
        try:
            target = target_following.target_record(self.task, records)
        except target_following.TargetUnavailable:
            with self._lock:
                self.latest_target = None
                self.progress = target_following.progress(self.task)
            raise
        with self._lock:
            self.latest_target = dict(target)
            self.progress = target_following.progress(self.task, self.latest_state, target, self.speed_unit)

    @property
    def route_complete(self) -> bool:
        with self._lock:
            return not self.is_following and self.current_route >= len(self.waypoints)

    def set_phase(self, phase: str, description: str) -> None:
        with self._lock:
            self.phase, self.description = phase, description

    def record_error(self, stage: str, exc: Exception, limit: int) -> int:
        with self._lock:
            self.consecutive_errors += 1
            self.errors = error_details(stage, exc)
            self.errors["maxConsecutiveErrors"] = limit
            labels = dict(situation="态势接收", inference="模型决策", control="控制指令发送", task="任务校验", collision="碰撞检测")
            objective = ("{}目标{}".format('伴随' if self.task.get('code')=='accompanyMode' else '跟随',target_following.target_id(self.task)) if self.is_following else
                         "目标航路点{}/{}".format(min(self.current_route + 1, len(self.waypoints)), len(self.waypoints)))
            self.description = (
                "{}异常：平台{}，{}，连续失败{}/{}次；{}: {}".format(
                    labels.get(stage, stage), self.task["params"].get("platCode"),
                    objective,
                    self.consecutive_errors, limit, self.errors["errorType"], self.errors["lastError"],
                )
            )
            if self.phase not in ("stopping", "finishing"):
                self.phase = "retrying"
            return self.consecutive_errors

    def clear_errors(self) -> None:
        with self._lock:
            self.consecutive_errors = 0
            self.errors = {}
            if self.phase not in ("stopping", "finishing"):
                self.phase, self.description = "tracking", ("伴随中" if self.task.get("code")=="accompanyMode" else "跟随中" if self.task.get("code")=="followMode" else "tracking")

    def status_fields(self) -> Tuple[str, Dict[str, Any]]:
        with self._lock:
            description = self.description
            progress = dict(self.progress)
            if progress.get("nextWaypoint") is not None:
                progress["nextWaypoint"] = dict(progress["nextWaypoint"])
            progress.update(self.errors)
            progress.update(phase=self.phase,
                            consecutiveErrors=self.consecutive_errors,
                            stopCommandSent=self.stop_command_sent, stopConfirmed=self.stop_confirmed)
            if not self.is_following:
                progress["currentRoute"] = self.current_route
        return description, progress


class TaskRegistry:
    """Keep one control owner per boat, replacing it for every valid new task."""

    def __init__(self) -> None:
        self._sessions: Dict[Tuple[Any, Any], ControlSession] = {}
        self._platforms: Dict[int, ControlSession] = {}
        self._output_locks: Dict[int, Any] = {}
        self._lock = threading.Lock()

    def add(self, session: ControlSession) -> bool:
        self.replace(session)
        return True

    def replace(self, session: ControlSession) -> Optional[ControlSession]:
        platform = session.task["params"]["platCode"]
        with self._lock:
            collision = self._sessions.get(session.key)
            if (collision is not None and not collision.finished.is_set()
                    and collision.task["params"]["platCode"] != platform):
                raise ValueError("simId/id is already assigned to another platform")
            output_lock = self._output_locks.setdefault(platform, threading.RLock())
            # A send already in progress finishes before ownership changes. All
            # later old sends, including emergency stops, see superseded=True.
            with output_lock:
                previous = self._platforms.get(platform)
                replaced = None
                if previous is not None:
                    if not previous.finished.is_set():
                        previous.supersede(session)
                        replaced = previous
                    if self._sessions.get(previous.key) is previous:
                        del self._sessions[previous.key]
                session.output_lock = output_lock
                self._sessions[session.key] = session
                self._platforms[platform] = session
                return replaced

    def get(self, key: Tuple[Any, Any]) -> Optional[ControlSession]:
        with self._lock:
            return self._sessions.get(key)

    def remove(self, session: ControlSession) -> None:
        with self._lock:
            if self._sessions.get(session.key) is session:
                del self._sessions[session.key]
            platform = session.task["params"]["platCode"]
            if self._platforms.get(platform) is session:
                del self._platforms[platform]


def build_speed_course_packet(plat_code: int, speed: float, course: float) -> bytes:
    """Encode Excel-defined 0x0662 speed/course information."""

    if not 0 <= plat_code <= 0xFFFFFFFF:
        raise ValueError("platCode exceeds uint32")
    if not 0.0 <= speed <= 655.35:
        raise ValueError("speed must be in [0, 655.35]")
    if not 0.0 <= course <= 655.35:
        raise ValueError("course must be in [0, 655.35]")
    body = struct.pack(">IHH", plat_code, round(speed * 100), round(course * 100))
    return (
        build_header(
            FIXED_HEADER_SIZE + len(body),
            SPEED_COURSE_ID,
            timestamp=speed_course_timestamp(),
            fragmented=False,
        )
        + body
    )


def decode_speed_course_packet(payload: bytes) -> Dict[str, Any]:
    """Decode an Excel-defined 0x0662 speed/course datagram."""

    header = parse_header(payload)
    if header["secondaryInfoUnitId"] != SPEED_COURSE_ID:
        raise ValueError("packet is not speed/course information")
    if len(payload) != FIXED_HEADER_SIZE + 8:
        raise ValueError("speed/course packet must be 20 bytes")
    plat_code, speed_raw, course_raw = struct.unpack(
        ">IHH", payload[FIXED_HEADER_SIZE:]
    )
    return {
        "platCode": plat_code,
        "speed": speed_raw * 0.01,
        "course": course_raw * 0.01,
    }


def service(
    payload: bytes,
    policy: Optional[Policy] = None,
    situation: Tuple[Dict[str, Any], ...] = (),
) -> bytes:
    """Transform an algorithm task into a binary speed/course datagram."""

    if policy is None:
        policy = create_policy()
    observation = analyze_task(payload)
    policy_input = dict(observation)
    policy_input["situation"] = situation
    action = policy(policy_input)
    if len(action) != 2:
        raise ValueError(f"policy must return two values, got {action!r}")
    params = observation.get("params")
    if not isinstance(params, Mapping):
        raise ValueError("algorithm task params must be an object")
    plat_code = params.get("platCode")
    if not isinstance(plat_code, int) or isinstance(plat_code, bool):
        raise ValueError("params.platCode must be an integer")
    return build_speed_course_packet(plat_code, action[0], action[1])


def start_server(
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    policy: Optional[Policy] = None,
    buffer_size: int = DEFAULT_BUFFER_SIZE,
    situation_group: str = DEFAULT_SITUATION_GROUP,
    situation_port: int = DEFAULT_SITUATION_PORT,
    multicast_interface: str = "0.0.0.0",
    control_interval: float = 1.0,
    max_cycles: int = 0,
    action_host: str = DEFAULT_ACTION_HOST,
    action_port: int = DEFAULT_ACTION_PORT,
    platform_id: Optional[int] = None,
    status_interval: float = 1.0,
    situation_max_age: float = 2.5,
    stop_speed_threshold: float = 0.1,
    reassembly_timeout: float = 3.0,
    max_consecutive_errors: int = DEFAULT_MAX_CONSECUTIVE_ERRORS,
    command_hold_timeout: float = DEFAULT_COMMAND_HOLD_TIMEOUT,
) -> None:
    """Subscribe to situations and start periodic task control loops."""

    if status_interval <= 0 or situation_max_age <= 0 or stop_speed_threshold < 0:
        raise ValueError("status interval/max age must be positive and stop threshold nonnegative")
    if (not math.isfinite(control_interval) or control_interval <= 0
            or type(max_consecutive_errors) is not int or max_consecutive_errors < 1
            or not math.isfinite(command_hold_timeout) or command_hold_timeout <= 0):
        raise ValueError("control interval/hold timeout must be positive and max consecutive errors at least 1")
    if policy is None:
        policy = create_policy(duration=control_interval)
    situations = SituationStore()
    receipt_log = SituationLogThrottle()
    registry = TaskRegistry()
    reassembler = FragmentReassembler(reassembly_timeout)
    subscriber = threading.Thread(
        target=subscribe_situations,
        args=(
            situations,
            situation_group,
            situation_port,
            multicast_interface,
            buffer_size,
            receipt_log,
        ),
        daemon=True,
        name="situation-multicast-subscriber",
    )
    subscriber.start()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server:
        server.bind((host, port))
        LOGGER.info("listening on UDP %s:%d", host, port)

        while True:
            payload, remote = server.recvfrom(buffer_size)
            try:
                header = parse_header(payload)
                secondary_id = header["secondaryInfoUnitId"]
                if secondary_id == SITUATION_ID:
                    state = decode_situation_packet(payload)
                    situations.update(state)
                    receipt_log.received(state, remote)
                    continue
                if secondary_id not in (ALGORITHM_TASK_ID, ALGORITHM_STOP_ID):
                    raise ValueError(
                        "unsupported secondaryInfoUnitId 0x{:04X}".format(
                            secondary_id
                        )
                    )

                complete = reassembler.add(payload, remote)
                if complete is None:
                    continue
                task = analyze_task(complete)
                if secondary_id == ALGORITHM_TASK_ID:
                    LOGGER.info("received task payload from %s:%d:\n%s", *remote,
                                json.dumps(task, ensure_ascii=False, indent=2))
                key = task.get("simId"), task.get("id")
                if (not isinstance(task.get("id"), str) or not task["id"].strip()
                        or (task.get("simId") is not None
                            and not isinstance(task["simId"], str))):
                    raise ValueError("task id must be a nonempty string; simId must be a string")
                if secondary_id == ALGORITHM_STOP_ID:
                    LOGGER.info("received stop simId=%s id=%s code=%s from %s:%d",
                                *key, task.get("code"), *remote)
                    session = registry.get(key)
                    if session is None or session.finished.is_set():
                        LOGGER.info("ignored stop for inactive task %s", task.get("id"))
                    else:
                        session.request_stop()
                        LOGGER.info("stop requested for task %s", task.get("id"))
                    continue

                params = task.get("params")
                task_platform = params.get("platCode") if isinstance(params, Mapping) else None
                LOGGER.info("received task simId=%s id=%s code=%s platformId=%r "
                            "waypoints=%r currentRoute=%r from %s:%d", *key, task.get("code"),
                            task_platform, params.get("planWaypointNum") if isinstance(params, Mapping) else None,
                            params.get("currentRoute") if isinstance(params, Mapping) else None, *remote)
                if platform_id is not None and task_platform != platform_id:
                    LOGGER.info("ignored task %s for platform %r; configured platformId=%d",
                                task.get("id"), task_platform, platform_id)
                    continue
                try:
                    session = ControlSession(task, remote, getattr(policy, "speed_unit", "knots"))
                    if session.is_following:
                        if not getattr(type(policy), 'supports_target_following', False):
                            raise ValueError('followMode/accompanyMode/targetFollowing require the Kev backend')
                        if task.get('code') in target_following.TARGET_CODES:
                            if __package__:
                                from .navigation.target_tasks import validate_feasibility
                            else:
                                from navigation.target_tasks import validate_feasibility
                            validate_feasibility(task,max(policy.obstacle_config.collision_radius_m,
                                                          policy.dynamic_config.safe_distance_m))
                        elif task['params']['minFollowingDistanceM'] <= max(
                                policy.obstacle_config.collision_radius_m, policy.dynamic_config.safe_distance_m):
                            raise ValueError('minimum following distance must exceed configured obstacle safety distance')
                    replaced = registry.replace(session)
                except (ValueError, TypeError, KeyError) as exc:
                    details = error_details("task", exc)
                    if task.get('code') in target_following.TARGET_CODES and isinstance(task.get('params'),dict):
                        try:
                            details={**target_following.progress(task),**details}
                        except (KeyError,TypeError,ValueError):
                            pass
                    LOGGER.warning("task %s validation failed: %s", task.get("id"), details["lastError"])
                    server.sendto(build_algorithm_status_packet(
                        task, 3, "任务校验失败：{}；任务未启动，未下发航行指令".format(details["lastError"]), details,
                    ), remote)
                    continue
                if replaced is not None:
                    LOGGER.info("task takeover platformId=%s old=%s/%s new=%s/%s",
                                task_platform, *replaced.key, *session.key)
                    # Same-key replacement shares the BT cache entry. Reporting
                    # the old terminal state would overwrite the new task there.
                    if replaced.key != session.key:
                        description, fields = replaced.status_fields()
                        fields.update(phase="superseded", errorCode="TASK_SUPERSEDED",
                                      replacedByTaskId=task.get("id"), replacedBySimId=task.get("simId"))
                        description = "任务已被新任务{}覆盖，旧任务停止执行，控制权已移交".format(task.get("id"))
                        try:
                            server.sendto(build_algorithm_status_packet(
                                replaced.task, 3, description, fields,
                            ), replaced.sender)
                        except OSError as exc:
                            LOGGER.warning("task %s superseded status send failed: %s", replaced.task.get("id"), exc)
                # A lost initial status must not leave an admitted task unstarted.
                try:
                    description, status_params = session.status_fields()
                    server.sendto(build_algorithm_status_packet(task, 1, description, status_params), remote)
                except OSError as exc:
                    LOGGER.warning("task %s initial status send failed: %s", task.get("id"), exc)
                reporter = threading.Thread(
                    target=report_task_status, args=(session, server, status_interval),
                    daemon=True, name="status-{}".format(task["id"]),
                )
                reporter.start()
                worker = threading.Thread(
                    target=run_control_loop,
                    args=(session, server, situations, (action_host, action_port), policy,
                          control_interval, max_cycles, situation_max_age, stop_speed_threshold,
                          registry, max_consecutive_errors, command_hold_timeout),
                    daemon=True, name="control-{}".format(task["id"]),
                )
                worker.start()
                LOGGER.info("started control loop for task %s on platform %s", task["id"], task_platform)
            except (UnicodeDecodeError, ValueError, TypeError, OSError) as exc:
                LOGGER.warning("failed to handle datagram from %s:%d: %s", *remote, exc)


def report_task_status(
    session: ControlSession,
    status_socket: socket.socket,
    interval: float,
) -> None:
    """Report status on a monotonic schedule independent of Jev latency."""

    deadline = time.monotonic() + interval
    while True:
        wait_time = max(0.0, deadline - time.monotonic())
        if session.finished.wait(wait_time):
            return
        try:
            with session.output_lock, session.status_lock:
                if session.finished.is_set():
                    return
                description, params = session.status_fields()
                status_socket.sendto(
                    build_algorithm_status_packet(session.task, 1, description, params),
                    session.sender,
                )
        except OSError as exc:
            LOGGER.warning("failed to report task %s status: %s", session.task.get("id"), exc)
        deadline += interval
        now = time.monotonic()
        if deadline <= now:
            deadline += (int((now - deadline) / interval) + 1) * interval


def subscribe_situations(
    situations: SituationStore,
    group: str,
    port: int,
    interface_ip: str,
    buffer_size: int = DEFAULT_BUFFER_SIZE,
    receipt_log: Optional[SituationLogThrottle] = None,
) -> None:
    """Keep the situation multicast joined, retrying after socket failures."""

    if receipt_log is None:
        receipt_log = SituationLogThrottle()
    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sub:
                sub.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sub.bind(("", port))
                membership = socket.inet_aton(group) + socket.inet_aton(interface_ip)
                sub.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
                if hasattr(socket, "SIO_UDP_CONNRESET"):
                    sub.ioctl(socket.SIO_UDP_CONNRESET, False)
                LOGGER.info(
                    "subscribed to situation multicast %s:%d via %s",
                    group,
                    port,
                    interface_ip,
                )
                while True:
                    payload, remote = sub.recvfrom(buffer_size)
                    try:
                        state = decode_situation_packet(payload)
                        situations.update(state)
                        receipt_log.received(state, remote)
                    except (ValueError, TypeError) as exc:
                        LOGGER.warning(
                            "invalid situation from %s:%d: %s", *remote, exc
                        )
        except OSError as exc:
            LOGGER.warning(
                "situation multicast %s:%d via %s failed: %s; retrying in %.1f seconds",
                group, port, interface_ip, exc, MULTICAST_RETRY_INTERVAL,
            )
            time.sleep(MULTICAST_RETRY_INTERVAL)


def run_control_loop(
    session: ControlSession,
    status_socket: socket.socket,
    situations: SituationStore,
    action_target: Tuple[str, int],
    policy: Policy,
    control_interval: float,
    max_cycles: int,
    situation_max_age: float,
    stop_speed_threshold: float,
    registry: TaskRegistry,
    max_consecutive_errors: int = DEFAULT_MAX_CONSECUTIVE_ERRORS,
    command_hold_timeout: float = DEFAULT_COMMAND_HOLD_TIMEOUT,
) -> None:
    """Track mandatory waypoints, retry transient errors, and confirm terminal stopping."""
    run_task_control(
        session, status_socket, situations, action_target, policy, control_interval,
        max_cycles, situation_max_age, stop_speed_threshold, registry,
        max_consecutive_errors, command_hold_timeout,
        build_speed_course_packet, build_algorithm_status_packet, LOGGER,
    )


def load_local_config(config_path: Optional[Path] = None) -> None:
    """Load persistent service settings without replacing explicit environment values."""

    path = LOCAL_CONFIG_PATH if config_path is None else config_path
    if not path.is_file():
        return
    config = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("local service configuration must be a JSON object")
    for name in ("JEV_API_KEY", "JEV_ENDPOINT"):
        value = config.get(name)
        if value is None:
            continue
        if not isinstance(value, str) or not value.strip():
            raise ValueError("{} in local configuration must be a nonempty string".format(name))
        os.environ.setdefault(name, value.strip())
    initial_course = config.get("INITIAL_COURSE_DEG")
    if initial_course is not None:
        message = "INITIAL_COURSE_DEG in local configuration must be a number in [0, 360)"
        try:
            course = float(initial_course)
        except (TypeError, ValueError):
            raise ValueError(message) from None
        if isinstance(initial_course, bool) or not math.isfinite(course) or not 0 <= course < 360:
            raise ValueError(message)
        os.environ.setdefault("INITIAL_COURSE_DEG", str(course))
    for name in ("MAX_CONSECUTIVE_ERRORS", "COMMAND_HOLD_TIMEOUT"):
        value = config.get(name)
        if value is None:
            continue
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            raise ValueError("{} must be positive".format(name)) from None
        if (isinstance(value, bool) or not math.isfinite(numeric) or numeric <= 0
                or (name == "MAX_CONSECUTIVE_ERRORS" and not numeric.is_integer())):
            raise ValueError("{} must be positive; error limit must be an integer".format(name))
        os.environ.setdefault(name, str(int(numeric)) if name == "MAX_CONSECUTIVE_ERRORS" else str(numeric))


def parse_args() -> argparse.Namespace:
    load_local_config()
    platform_id_env = os.getenv("PLATFORM_ID", "").strip()
    parser = argparse.ArgumentParser(description="UDP decision service (Kev-4B / Jev / Julia)")
    parser.add_argument("--host", default=os.getenv("TASK_BIND_HOST", DEFAULT_HOST), help="address to bind")
    parser.add_argument("--port", type=int, default=int(os.getenv("TASK_PORT", DEFAULT_PORT)), help="UDP port")
    parser.add_argument("--backend", choices=("kev", "jev", "julia"), default=os.getenv("POLICY_BACKEND", DEFAULT_BACKEND),
                        help="决策后端，默认 kev；可切换为 jev 或 julia")
    parser.add_argument("--local-model-path", default=os.getenv("LOCAL_MODEL_PATH", str(Path(__file__).resolve().parents[1] / "models/Julia-1")))
    parser.add_argument("--kev-endpoint", default=os.getenv("KEV_ENDPOINT", "http://127.0.0.1:18028/v1/systemone"))
    parser.add_argument("--max-turn-rate-deg-s", type=float, default=float(os.getenv("MAX_TURN_RATE_DEG_S", str(DEFAULT_MAX_TURN_RATE_DEG_S))), help="Kev转向角速度预测上限，默认12度/秒（11节工况），角加速度上限4度/秒²；其他航速暂用同一上限")
    parser.add_argument("--acceleration-response", choices=("measured_0_50kn", "bounded"), default=os.getenv("ACCELERATION_RESPONSE", "measured_0_50kn"), help="默认仅在直线加速到50节时使用实测响应；bounded仅使用能力上限")
    parser.add_argument("--motion-curves-file", default=os.getenv("MOTION_CURVES_FILE"), help="可选SI航速能力曲线JSON；acceleration_curve/deceleration_curve/turn_rate_curve")
    parser.add_argument("--initial-course-deg", type=float, default=float(os.environ["INITIAL_COURSE_DEG"]) if os.getenv("INITIAL_COURSE_DEG") else None, help="Kev静止且态势缺少targetCourse时的备用方向；优先使用态势航向")
    parser.add_argument("--kev-timeout", type=float, default=float(os.getenv("KEV_TIMEOUT", "120")))
    parser.add_argument("--max-consecutive-errors", type=int,
                        default=int(os.getenv("MAX_CONSECUTIVE_ERRORS", str(DEFAULT_MAX_CONSECUTIVE_ERRORS))),
                        help="可恢复错误连续达到此次数才失败停车，默认3次")
    parser.add_argument("--command-hold-timeout", type=float,
                        default=float(os.getenv("COMMAND_HOLD_TIMEOUT", str(DEFAULT_COMMAND_HOLD_TIMEOUT))),
                        help="最近有效指令的续发时限及模型结果有效期，默认5秒；超时暂发零速并继续等待/重试")
    parser.add_argument("--device", choices=("cpu", "cuda"), default=os.getenv("LOCAL_MODEL_DEVICE", "cpu"))
    parser.add_argument("--model", default=os.getenv("JEV_MODEL", "jev-latest"))
    parser.add_argument("--endpoint", default=os.getenv("JEV_ENDPOINT", DEFAULT_JEV_ENDPOINT))
    parser.add_argument("--speed-unit", choices=("knots", "mps"), default=os.getenv("SPEED_UNIT", "knots"),
                        help="态势、航路点及控制报文速度单位，默认 knots")
    parser.add_argument("--max-speed-knots", type=float, default=float(os.getenv("MAX_SPEED_KNOTS", "50.0")))
    parser.add_argument("--static-obstacle-radius-m", type=float, default=float(os.getenv("STATIC_OBSTACLE_RADIUS_M", "80")))
    parser.add_argument("--vessel-length-m", type=float, default=float(os.getenv("VESSEL_LENGTH_M", "11.6")))
    parser.add_argument("--vessel-width-m", type=float, default=float(os.getenv("VESSEL_WIDTH_M", "3.4")))
    parser.add_argument("--static-obstacle-margin-m", type=float, default=float(os.getenv("STATIC_OBSTACLE_MARGIN_M", "0")))
    parser.add_argument("--speed-transition-time", type=float, default=float(os.getenv("SPEED_TRANSITION_TIME", "0.5")), help="兼容旧配置；物理预测已改用加减速度约束，此值不再决定变速时间")
    parser.add_argument("--course-transition-time", type=float, default=float(os.getenv("COURSE_TRANSITION_TIME", "0.5")), help="兼容旧配置；物理预测已改用角速度及角加速度约束")
    parser.add_argument("--jev-timeout", type=float, default=float(os.getenv("JEV_TIMEOUT", "30.0")))
    parser.add_argument("--situation-group", default=os.getenv("SITUATION_GROUP", DEFAULT_SITUATION_GROUP))
    parser.add_argument("--situation-port", type=int, default=int(os.getenv("SITUATION_PORT", DEFAULT_SITUATION_PORT)))
    parser.add_argument(
        "--multicast-interface",
        default=os.getenv("MULTICAST_INTERFACE", "0.0.0.0"),
        help="加入态势组播使用的本机网卡 IPv4；默认由系统选择",
    )
    parser.add_argument(
        "--control-interval",
        type=float,
        default=float(os.getenv("CONTROL_INTERVAL", "1.0")),
        help="航速航向控制周期，默认 1 秒",
    )
    parser.add_argument(
        "--max-cycles",
        type=int,
        default=int(os.getenv("MAX_CYCLES", "0")),
        help="0 表示持续控制；正数表示完成指定周期后回复完成",
    )
    parser.add_argument(
        "--action-host",
        default=os.getenv("ACTION_HOST", DEFAULT_ACTION_HOST),
        help="动力系统地址，源码默认 192.168.3.201；环境变量可覆盖",
    )
    parser.add_argument(
        "--action-port",
        type=int,
        default=int(os.getenv("ACTION_PORT", DEFAULT_ACTION_PORT)),
        help="动力系统航速航向接收端口，默认 18123",
    )
    parser.add_argument(
        "--platform-id",
        type=int,
        default=int(platform_id_env) if platform_id_env else DEFAULT_PLATFORM_ID,
        help="本服务负责的平台编码，默认1010；可通过PLATFORM_ID或此参数覆盖",
    )
    parser.add_argument(
        "--status-interval",
        type=float,
        default=float(os.getenv("STATUS_INTERVAL", "1.0")),
        help="0x0664执行中状态严格上报周期，默认1秒",
    )
    parser.add_argument(
        "--situation-max-age",
        type=float,
        default=float(os.getenv("SITUATION_MAX_AGE", "2.5")),
        help="本艇态势最大允许陈旧时间，默认2.5秒",
    )
    parser.add_argument(
        "--stop-speed-threshold",
        type=float,
        default=float(os.getenv("STOP_SPEED_THRESHOLD", "0.1")),
        help="确认停车的实际航速阈值，默认0.1",
    )
    parser.add_argument(
        "--reassembly-timeout",
        type=float,
        default=float(os.getenv("REASSEMBLY_TIMEOUT", "3.0")),
        help="任务/停止分片重组超时，默认3秒",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    configure_actual_distance_log(Path(__file__).resolve().parent / 'logs' /
                                  f'static-obstacle-distance-{args.platform_id}-{args.port}.log')
    obstacle_config = StaticObstacleConfig(obstacle_radius_m=args.static_obstacle_radius_m,
        vessel_length_m=args.vessel_length_m, vessel_width_m=args.vessel_width_m,
        safety_margin_m=args.static_obstacle_margin_m)
    policy = create_policy(
        args.backend, model_path=args.local_model_path, device=args.device,
        kev_endpoint=args.kev_endpoint, kev_timeout=args.kev_timeout,
        kev_max_turn_rate_deg_s=args.max_turn_rate_deg_s, kev_initial_course_deg=args.initial_course_deg,
        kev_obstacle_config=obstacle_config,
        model=args.model, endpoint=args.endpoint, speed_unit=args.speed_unit,
        max_speed_knots=args.max_speed_knots, duration=args.control_interval,
        motion_curves=json.loads(Path(args.motion_curves_file).read_text()) if args.motion_curves_file else None,
        acceleration_response=args.acceleration_response,
        speed_transition=args.speed_transition_time,
        course_transition=args.course_transition_time,
        api_timeout=args.jev_timeout,
    )
    LOGGER.info("using policy backend=%s class=%s", args.backend, type(policy).__name__)

    try:
        start_server(
            args.host,
            args.port,
            policy,
            situation_group=args.situation_group,
            situation_port=args.situation_port,
            multicast_interface=args.multicast_interface,
            control_interval=args.control_interval,
            max_cycles=args.max_cycles,
            action_host=args.action_host,
            action_port=args.action_port,
            platform_id=args.platform_id,
            status_interval=args.status_interval,
            situation_max_age=args.situation_max_age,
            stop_speed_threshold=args.stop_speed_threshold,
            reassembly_timeout=args.reassembly_timeout,
            max_consecutive_errors=args.max_consecutive_errors,
            command_hold_timeout=args.command_hold_timeout,
        )
    except KeyboardInterrupt:
        LOGGER.info("server stopped")
    except OSError as exc:
        if getattr(exc, "winerror", None) == 10048:
            raise SystemExit(
                "启动失败：UDP 端口 {} 已被占用。请关闭原监听程序，或使用 "
                "--port 指定其他端口（例如 --port 18015）。".format(args.port)
            )
        raise SystemExit("启动 UDP 服务失败：{}".format(exc))


if __name__ == "__main__":
    main()
    
