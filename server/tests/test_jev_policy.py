import sys
import json
import struct
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from decision_service import (
    JevPolicy, ALGORITHM_TASK_ID, ALGORITHM_STOP_ID, ControlSession,
    FragmentReassembler, SituationStore, TaskRegistry, build_header,
    build_json_packet, build_algorithm_status_packet, decode_speed_course_packet,
    analyze_task, calculate_route_progress, report_task_status, run_control_loop, service,
)
from simulate_information import sample_information
from tools.waypoint_geometry import KNOT_MPS


class JevIntegrationTests(unittest.TestCase):
    def packet(self):
        packet = sample_information()
        platform_id = packet["params"]["platCode"]
        packet["situation"] = [dict(platformId=99, longitude=0, latitude=0,
                                    targetSpeed=1, targetCourse=90),
                               dict(platformId=platform_id, longitude=120.59, latitude=30.58,
                                    targetSpeed=10, targetCourse=90)]
        return packet

    def test_binary_task_to_jev_and_binary_action(self):
        packet = self.packet()
        situations = packet.pop("situation")
        policy = JevPolicy()
        policy.local.transport = Mock()
        policy.local.transport.call.return_value = {"answers": {"action": {
            "type": "choice", "choice": "speed_7__east"}}}
        with patch.dict("os.environ", {"JEV_API_KEY": "test-only\n"}):
            result = service(build_json_packet(packet, ALGORITHM_TASK_ID), policy, tuple(situations))
        self.assertEqual(
            decode_speed_course_packet(result),
            dict(platCode=packet["params"]["platCode"], speed=50, course=90),
        )
        payload = policy.local.transport.call.call_args.args[0]
        self.assertEqual(policy.local.transport.call.call_args.args[1], "test-only")
        self.assertAlmostEqual(payload["state"]["boat"]["actual_speed_mps"], 10 * KNOT_MPS, places=5)
        self.assertEqual(len(payload["questions"]["action"]["criteria"]), 64)
        self.assertIn("keeps moving", payload["state"]["task"]["background"])

    def test_progress_and_task_reset(self):
        packet = self.packet()
        point = packet["params"]["planWaypointList"][0]
        packet["situation"][1].update(longitude=point["routeLon"], latitude=point["routeLat"])
        policy = JevPolicy()
        self.assertEqual(policy.observation(packet)["task"]["next_waypoint_index"], 1)
        packet["id"] = "another-task"
        packet["situation"][1].update(longitude=120.59, latitude=30.58)
        self.assertEqual(policy.observation(packet)["task"]["next_waypoint_index"], 0)

    def test_missing_own_boat_and_unknown_speed(self):
        packet = self.packet()
        packet["situation"] = packet["situation"][:1]
        with self.assertRaisesRegex(ValueError, "No situation"):
            JevPolicy().observation(packet)
        packet = self.packet()
        packet["situation"][1]["targetSpeed"] = None
        with self.assertRaises(ValueError):
            JevPolicy().observation(packet)

    def test_mps_conversion(self):
        policy = JevPolicy(speed_unit="mps")
        obs = policy.observation(self.packet())
        self.assertAlmostEqual(obs["boat"]["targetSpeed"], 10 / KNOT_MPS)
        self.assertEqual(obs["task"]["planWaypointList"][0]["routeSpeed"], 51.444444444444445)
        policy.local.transport = Mock()
        policy.local.transport.call.return_value = {"answers": {"action": {
            "type": "choice", "choice": "speed_7__north"}}}
        with patch.dict("os.environ", {"JEV_API_KEY": "test-only"}):
            speed, course = policy(self.packet())
        self.assertAlmostEqual(speed, 50 * KNOT_MPS)
        self.assertEqual(course, 0)

    def test_api_failure_propagates(self):
        policy = JevPolicy()
        policy.local.transport = Mock()
        policy.local.transport.call.side_effect = RuntimeError("API failed")
        with patch.dict("os.environ", {"JEV_API_KEY": "test-only"}):
            with self.assertRaisesRegex(RuntimeError, "API failed"):
                policy(self.packet())

    def test_stationary_heading_is_preserved_for_physical_startup(self):
        packet = self.packet()
        packet['situation'][1].update(targetSpeed=0., targetCourse=230.)
        self.assertEqual(JevPolicy().observation(packet)['boat']['targetCourse'],230.)

    def test_fragmented_stop_reassembly(self):
        content = json.dumps(
            {"simId": "sim-1", "id": "node-1", "code": "trackingMode"},
            separators=(",", ":"),
        ).encode("utf-8")
        split = len(content) // 2
        stamp = 12345
        parts = [
            build_header(16 + split, ALGORITHM_STOP_ID, stamp, 2, 1)
            + struct.pack(">H", len(content))
            + content[:split],
            build_header(14 + len(content) - split, ALGORITHM_STOP_ID, stamp, 2, 2)
            + content[split:],
        ]
        reassembler = FragmentReassembler()
        self.assertIsNone(reassembler.add(parts[1], ("127.0.0.1", 7612)))
        complete = reassembler.add(parts[0], ("127.0.0.1", 7612))
        self.assertEqual(analyze_task(complete)["id"], "node-1")

    def test_status_contains_route_progress(self):
        session = ControlSession(self.packet(), ("127.0.0.1", 7612))
        session.update_situation(self.packet()["situation"][1])
        description, params = session.status_fields()
        packet = build_algorithm_status_packet(
            self.packet(), 1, description, params
        )
        decoded = analyze_task(packet)
        self.assertEqual(decoded["status"], 1)
        self.assertEqual(decoded["description"], "tracking")
        self.assertEqual(decoded["params"]["nextWaypointIndex"], 0)
        self.assertEqual(decoded["params"]["remainingWaypointNum"], 3)
        self.assertGreater(decoded["params"]["remainingDistanceM"], 0)
        self.assertGreater(decoded["params"]["estimatedRemainingTimeS"], 0)
        self.assertGreaterEqual(decoded["params"]["completionRate"], 0)
        self.assertLessEqual(decoded["params"]["completionRate"], 100)

    def test_progress_formula_and_boundaries(self):
        task = {
            "params": {
                "planWaypointList": [
                    {"routeLon": 0.01, "routeLat": 0.0, "routeSpeed": 10.0},
                    {"routeLon": 0.02, "routeLat": 0.0, "routeSpeed": 20.0, "routeRadius": 20.0},
                ]
            }
        }
        state = {"longitude": 0.005, "latitude": 0.0, "targetSpeed": 10.0}
        origin = {"longitude": 0., "latitude": 0.}
        progress = calculate_route_progress(task, state, 0, "knots", origin)
        self.assertEqual(progress["nextWaypointIndex"], 0)
        self.assertEqual(progress["remainingWaypointNum"], 2)
        self.assertAlmostEqual(progress["completionRate"], 25.0, places=2)
        self.assertAlmostEqual(progress["remainingDistanceM"], 1669.79, delta=0.2)
        self.assertAlmostEqual(progress["estimatedRemainingTimeS"], 216.4, delta=0.3)

        zero_speed = calculate_route_progress(
            task, state | {"targetSpeed": 0.0}, 0, "knots", origin
        )
        self.assertIsNone(zero_speed["estimatedRemainingTimeS"])

        off_route = calculate_route_progress(
            task, {"longitude": -1.0, "latitude": 0.0, "targetSpeed": 10.0}, 0, "knots", origin
        )
        self.assertEqual(off_route["completionRate"], 0.0)

        completed = calculate_route_progress(
            task, {"longitude": 0.02, "latitude": 0.0, "targetSpeed": 0.0}, 1, "knots"
        )
        self.assertEqual(completed["nextWaypoint"], None)
        self.assertEqual(completed["remainingWaypointNum"], 0)
        self.assertEqual(completed["remainingDistanceM"], 0.0)
        self.assertEqual(completed["estimatedRemainingTimeS"], 0.0)
        self.assertEqual(completed["completionRate"], 100.0)

    def test_same_task_registry_replaces_running_session(self):
        task = self.packet()
        registry = TaskRegistry()
        first = ControlSession(task, ("127.0.0.1", 7612))
        second = ControlSession(task, ("127.0.0.1", 7612))
        self.assertTrue(registry.add(first))
        self.assertTrue(registry.add(second))
        self.assertTrue(first.superseded.is_set())
        self.assertTrue(first.finished.is_set())
        registry.remove(first)
        self.assertIs(registry.get(second.key), second)

    def test_stop_waits_for_actual_speed_and_reports_completion(self):
        task = self.packet()
        session = ControlSession(task, ("127.0.0.1", 7612))
        session.request_stop()
        situations = SituationStore()
        situations.update(task["situation"][1] | {"targetSpeed": 0.05})
        registry = TaskRegistry()
        registry.add(session)
        status_socket = Mock()
        control_socket = Mock()
        context = Mock()
        context.__enter__ = Mock(return_value=control_socket)
        context.__exit__ = Mock(return_value=False)
        with patch("decision_service.socket.socket", return_value=context):
            run_control_loop(
                session, status_socket, situations, ("192.168.3.56", 18123),
                Mock(), 1.0, 0, 2.5, 0.1, registry,
            )
        command = decode_speed_course_packet(control_socket.sendto.call_args.args[0])
        self.assertEqual(command["speed"], 0)
        status = analyze_task(status_socket.sendto.call_args.args[0])
        self.assertEqual(status["status"], 2)
        self.assertEqual(status["params"]["nextWaypointIndex"], 0)
        self.assertEqual(status["params"]["remainingWaypointNum"], 3)

    def test_status_reporter_runs_independently(self):
        session = ControlSession(self.packet(), ("127.0.0.1", 7612))
        status_socket = Mock()
        thread = threading.Thread(
            target=report_task_status, args=(session, status_socket, 0.02)
        )
        thread.start()
        time.sleep(0.10)
        session.finished.set()
        thread.join(1)
        self.assertGreaterEqual(status_socket.sendto.call_count, 2)

    def test_stop_during_jev_call_suppresses_obsolete_tracking_action(self):
        task = self.packet()
        session = ControlSession(task, ("127.0.0.1", 7612))
        situations = SituationStore()
        situations.update(task["situation"][1] | {"targetSpeed": 0.05})
        registry = TaskRegistry()
        registry.add(session)
        status_socket = Mock()
        control_socket = Mock()
        context = Mock()
        context.__enter__ = Mock(return_value=control_socket)
        context.__exit__ = Mock(return_value=False)

        def delayed_policy(_):
            session.request_stop()
            return 30.0, 90.0

        with patch("decision_service.socket.socket", return_value=context):
            run_control_loop(
                session, status_socket, situations, ("192.168.3.56", 18123),
                delayed_policy, 1.0, 0, 2.5, 0.1, registry,
            )
        commands = [
            decode_speed_course_packet(call.args[0])
            for call in control_socket.sendto.call_args_list
        ]
        self.assertEqual(
            commands,
            [{"platCode": task["params"]["platCode"], "speed": 0.0, "course": 90.0}],
        )

    def test_task_failure_sends_stop_before_failure_status(self):
        task = self.packet()
        session = ControlSession(task, ("127.0.0.1", 7612))
        situations = SituationStore()
        situations.update(task["situation"][1])
        registry = TaskRegistry()
        registry.add(session)
        status_socket = Mock()
        control_socket = Mock()
        context = Mock()
        context.__enter__ = Mock(return_value=control_socket)
        context.__exit__ = Mock(return_value=False)
        failed_policy = Mock(side_effect=RuntimeError("API timed out"))

        with patch("decision_service.socket.socket", return_value=context):
            run_control_loop(
                session, status_socket, situations, ("192.168.3.56", 18123),
                failed_policy, .01, 0, 2.5, 0.1, registry,
            )

        command = decode_speed_course_packet(control_socket.sendto.call_args.args[0])
        self.assertEqual(
            command,
            {"platCode": task["params"]["platCode"], "speed": 0.0, "course": 90.0},
        )
        status = analyze_task(status_socket.sendto.call_args.args[0])
        self.assertEqual(status["status"], 3)
        self.assertIn("API timed out", status["description"])
        self.assertIn("连续失败3/3", status["description"])
        self.assertIn("尚未确认实际停车", status["description"])
        self.assertEqual(failed_policy.call_count, 3)
        self.assertEqual(status["params"]["errorCode"], "MODEL_TIMEOUT")


if __name__ == "__main__":
    unittest.main()
