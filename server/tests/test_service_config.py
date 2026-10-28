"""Persistent service configuration tests without credentials or network calls."""

import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from server import decision_service
from server.jev_policy import JevPolicy
from server.policies.kev_policy import KevPolicy


class ServiceConfigTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name) / "decision_service.local.json"
        self.path.write_text(json.dumps({
            "JEV_API_KEY": "test-only-key",
            "JEV_ENDPOINT": "https://configured.example/v1/systemone",
        }), encoding="utf-8")

    def test_startup_loads_key_and_endpoint_from_local_config(self):
        with (
            patch.dict(os.environ, {}, clear=True),
            patch.object(decision_service, "LOCAL_CONFIG_PATH", self.path),
            patch("sys.argv", ["decision_service.py"]),
        ):
            args = decision_service.parse_args()
            self.assertEqual(os.environ["JEV_API_KEY"], "test-only-key")
            self.assertEqual(args.endpoint, "https://configured.example/v1/systemone")

    def test_environment_and_command_line_override_local_config(self):
        with (
            patch.dict(os.environ, {"JEV_API_KEY": "environment-key",
                                   "JEV_ENDPOINT": "https://environment.example"}, clear=True),
            patch.object(decision_service, "LOCAL_CONFIG_PATH", self.path),
            patch("sys.argv", ["decision_service.py"]),
        ):
            self.assertEqual(decision_service.parse_args().endpoint, "https://environment.example")
            self.assertEqual(os.environ["JEV_API_KEY"], "environment-key")
            with patch("sys.argv", ["decision_service.py", "--endpoint", "https://cli.example"]):
                self.assertEqual(decision_service.parse_args().endpoint, "https://cli.example")

    def test_missing_local_config_keeps_code_default_and_environment(self):
        with (
            patch.dict(os.environ, {}, clear=True),
            patch.object(decision_service, "LOCAL_CONFIG_PATH", self.path.with_name("missing.json")),
            patch("sys.argv", ["decision_service.py"]),
        ):
            self.assertEqual(decision_service.parse_args().endpoint,
                             "https://api.jevcn.cn/v1/systemone")
            self.assertNotIn("JEV_API_KEY", os.environ)

    def test_main_selects_kev_by_default_and_preserves_jev_overrides(self):
        for environment, arguments, expected in (
            ({}, [], KevPolicy),
            ({}, ["--backend", "jev"], JevPolicy),
            ({"POLICY_BACKEND": "jev"}, [], JevPolicy),
            ({"POLICY_BACKEND": "jev"}, ["--backend", "kev"], KevPolicy),
        ):
            with (
                self.subTest(environment=environment, arguments=arguments),
                patch.dict(os.environ, environment, clear=True),
                patch.object(decision_service, "LOCAL_CONFIG_PATH", self.path),
                patch("sys.argv", ["decision_service.py", *arguments]),
                patch.object(decision_service, "start_server") as start_server,
            ):
                decision_service.main()
                policy = start_server.call_args.args[2]
                self.assertIsInstance(policy, expected)
                if isinstance(policy, KevPolicy):
                    self.assertEqual(policy.endpoint, "http://127.0.0.1:18028/v1/systemone")
                    self.assertEqual(policy.timeout, 120.)
                    self.assertEqual(policy.max_turn_rate_deg_s, 45.)
                    self.assertEqual(policy.max_speed, 50.)
                else:
                    self.assertEqual(policy.endpoint, "https://configured.example/v1/systemone")
                    self.assertEqual(os.environ["JEV_API_KEY"], "test-only-key")

    def test_default_kev_startup_does_not_require_jev_credentials(self):
        with (
            patch.dict(os.environ, {}, clear=True),
            patch.object(decision_service, "LOCAL_CONFIG_PATH", self.path.with_name("missing.json")),
            patch("sys.argv", ["decision_service.py"]),
            patch.object(decision_service, "start_server") as start_server,
        ):
            decision_service.main()
            self.assertIsInstance(start_server.call_args.args[2], KevPolicy)
            self.assertNotIn("JEV_API_KEY", os.environ)

    def test_explicit_turn_rate_overrides_default(self):
        with (
            patch.dict(os.environ, {"MAX_TURN_RATE_DEG_S": "10"}, clear=True),
            patch.object(decision_service, "LOCAL_CONFIG_PATH", self.path),
            patch("sys.argv", ["decision_service.py", "--max-turn-rate-deg-s", "5"]),
            patch.object(decision_service, "start_server") as start_server,
        ):
            self.assertEqual(decision_service.parse_args().max_turn_rate_deg_s, 5.)
            decision_service.main()
            self.assertEqual(start_server.call_args.args[2].max_turn_rate_deg_s, 5.)

    def test_static_geometry_configuration_reaches_kev_with_cli_precedence(self):
        with (
            patch.dict(os.environ, {'STATIC_OBSTACLE_RADIUS_M':'90', 'VESSEL_LENGTH_M':'12',
                                    'VESSEL_WIDTH_M':'4'}, clear=True),
            patch.object(decision_service, 'LOCAL_CONFIG_PATH', self.path),
            patch('sys.argv', ['decision_service.py','--static-obstacle-radius-m','80']),
            patch.object(decision_service, 'start_server') as start_server,
        ):
            decision_service.main()
            config=start_server.call_args.args[2].obstacle_config
            self.assertEqual(config.obstacle_radius_m,80.)
            self.assertEqual((config.vessel_length_m,config.vessel_width_m),(12.,4.))
            self.assertAlmostEqual(config.collision_radius_m,86.32455532)

    def test_local_initial_course_accepts_zero_and_reaches_kev_policy(self):
        self.path.write_text(json.dumps({"INITIAL_COURSE_DEG": 0}), encoding="utf-8")
        with (
            patch.dict(os.environ, {}, clear=True),
            patch.object(decision_service, "LOCAL_CONFIG_PATH", self.path),
            patch("sys.argv", ["decision_service.py"]),
            patch.object(decision_service, "start_server") as start_server,
        ):
            decision_service.main()
            self.assertEqual(start_server.call_args.args[2].initial_course_deg, 0.)

    def test_initial_course_environment_and_cli_override_local_config(self):
        self.path.write_text(json.dumps({"INITIAL_COURSE_DEG": 90}), encoding="utf-8")
        with (
            patch.dict(os.environ, {"INITIAL_COURSE_DEG": "270"}, clear=True),
            patch.object(decision_service, "LOCAL_CONFIG_PATH", self.path),
            patch("sys.argv", ["decision_service.py"]),
        ):
            self.assertEqual(decision_service.parse_args().initial_course_deg, 270.)
            with patch("sys.argv", ["decision_service.py", "--initial-course-deg", "180"]):
                self.assertEqual(decision_service.parse_args().initial_course_deg, 180.)

    def test_invalid_local_initial_course_is_rejected(self):
        for value in (-1, 360, True, "east", float("nan"), float("inf")):
            with self.subTest(value=value), patch.dict(os.environ, {}, clear=True):
                self.path.write_text(json.dumps({"INITIAL_COURSE_DEG": value}), encoding="utf-8")
                with self.assertRaisesRegex(ValueError, "INITIAL_COURSE_DEG"):
                    decision_service.load_local_config(self.path)

    def test_local_retry_and_hold_settings_reach_task_server(self):
        self.path.write_text(json.dumps({"MAX_CONSECUTIVE_ERRORS": 3, "COMMAND_HOLD_TIMEOUT": 7.5}), encoding="utf-8")
        with (
            patch.dict(os.environ, {}, clear=True),
            patch.object(decision_service, "LOCAL_CONFIG_PATH", self.path),
            patch("sys.argv", ["decision_service.py"]),
            patch.object(decision_service, "start_server") as start_server,
        ):
            decision_service.main()
            self.assertEqual(start_server.call_args.kwargs['max_consecutive_errors'], 3)
            self.assertEqual(start_server.call_args.kwargs['command_hold_timeout'], 7.5)


if __name__ == "__main__":
    unittest.main()
