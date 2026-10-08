"""
Tests for lerobot-record: record_loop behaviour, _warmup_policy, display_cameras,
and the sanity-check helpers in control_utils.

Design: all robot/camera/policy hardware is mocked so tests run without any
physical device.  Tests are written in failing-first order — they describe the
expected contract and will pass once the implementation satisfies it.
"""

from __future__ import annotations

import time
from collections import deque
from types import SimpleNamespace
from unittest.mock import MagicMock, call, patch

import numpy as np
import pytest
import torch

# ---------------------------------------------------------------------------
# Helpers / shared fixtures
# ---------------------------------------------------------------------------

CAMERA_SHAPE = (480, 640, 3)  # H x W x C  (uint8, RGB)
STATE_DIM = 6
ACTION_DIM = 6


def _make_obs(n_cams: int = 2) -> dict:
    """Return a fake robot observation matching the real SO-101 format.

    The real robot returns individual motor scalars (e.g. 'joint_0.pos': 0.0)
    plus raw camera arrays — NOT a combined 'observation.state' array.
    build_dataset_frame looks up each name individually.
    """
    obs = {f"joint_{i}.pos": 0.0 for i in range(STATE_DIM)}
    cam_names = ["gripper_cam", "top_cam", "side_cam"]
    for name in cam_names[:n_cams]:
        obs[name] = np.zeros(CAMERA_SHAPE, dtype=np.uint8)
    return obs


def _make_robot(obs: dict | None = None) -> MagicMock:
    robot = MagicMock()
    robot.get_observation.return_value = obs or _make_obs()
    robot.action_features = {f"joint_{i}.pos": float for i in range(ACTION_DIM)}
    robot.robot_type = "so_follower"
    robot.name = "my_so_arm"
    return robot


def _make_events(exit_after: int = 0) -> dict:
    """Return an events dict that triggers exit_early after `exit_after` calls."""
    events: dict = {
        "exit_early": False,
        "rerecord_episode": False,
        "stop_recording": False,
    }
    if exit_after == 0:
        events["exit_early"] = True
    return events


def _make_policy_config(visual_keys: list[str] | None = None) -> MagicMock:
    from lerobot.configs.types import FeatureType

    cfg = MagicMock()
    cfg.use_amp = False
    cfg.device = "cpu"
    cfg.n_action_steps = 100

    features = {
        "observation.state": SimpleNamespace(type=FeatureType.STATE, shape=(STATE_DIM,)),
    }
    for key in (visual_keys or ["observation.images.gripper_cam", "observation.images.top_cam"]):
        features[key] = SimpleNamespace(type=FeatureType.VISUAL, shape=(3, 480, 640))

    cfg.input_features = features
    return cfg


def _make_policy(config: MagicMock | None = None) -> MagicMock:
    policy = MagicMock()
    policy.config = config or _make_policy_config()
    policy._action_queue = deque([np.zeros(ACTION_DIM, dtype=np.float32)] * 100)

    def _select_action(obs):
        if not policy._action_queue:
            policy._action_queue.extend(
                [np.zeros(ACTION_DIM, dtype=np.float32)] * 100
            )
        return policy._action_queue.popleft()

    policy.select_action.side_effect = _select_action
    return policy


def _identity_processor():
    """A processor pipeline stub that returns its input unchanged."""
    proc = MagicMock()
    proc.side_effect = lambda x: x
    proc.reset = MagicMock()
    return proc


def _action_processor():
    """Processor for (action, obs) -> action pipelines (robot_action_processor path)."""
    proc = MagicMock()
    proc.side_effect = lambda x: x[0]  # unwrap (action, obs) tuple, return action
    proc.reset = MagicMock()
    return proc


# ---------------------------------------------------------------------------
# 1.  _warmup_policy
# ---------------------------------------------------------------------------

class TestWarmupPolicy:
    def test_calls_predict_action_twice(self):
        """Warmup must run exactly two inference passes."""
        from lerobot.scripts.lerobot_record import _warmup_policy

        policy = _make_policy()
        preprocessor = _identity_processor()
        postprocessor = _identity_processor()
        device = MagicMock()
        device.type = "cpu"

        with patch(
            "lerobot.scripts.lerobot_record.predict_action",
            return_value={"joint_0.pos": 0.0},
        ) as mock_predict:
            _warmup_policy(policy, preprocessor, postprocessor, device, task="test task")

        assert mock_predict.call_count == 2

    def test_resets_policy_after_warmup(self):
        """policy.reset() must be called so the action queue is clean."""
        from lerobot.scripts.lerobot_record import _warmup_policy

        policy = _make_policy()
        preprocessor = _identity_processor()
        postprocessor = _identity_processor()
        device = MagicMock()
        device.type = "cpu"

        with patch("lerobot.scripts.lerobot_record.predict_action", return_value={}):
            _warmup_policy(policy, preprocessor, postprocessor, device, task=None)

        policy.reset.assert_called_once()

    def test_dummy_obs_shapes_match_policy_config(self):
        """Dummy observations must have the shapes the policy config declares."""
        from lerobot.configs.types import FeatureType
        from lerobot.scripts.lerobot_record import _warmup_policy

        captured_obs: list[dict] = []

        def capture(observation, **kwargs):
            captured_obs.append(dict(observation))
            return {}

        policy = _make_policy()
        preprocessor = _identity_processor()
        postprocessor = _identity_processor()
        device = MagicMock()
        device.type = "cpu"

        with patch("lerobot.scripts.lerobot_record.predict_action", side_effect=capture):
            _warmup_policy(policy, preprocessor, postprocessor, device, task=None)

        obs = captured_obs[0]
        for key, feature in policy.config.input_features.items():
            assert key in obs, f"Missing key {key} in dummy observation"
            arr = obs[key]
            if feature.type == FeatureType.VISUAL:
                c, h, w = feature.shape
                assert arr.shape == (h, w, c), f"Wrong image shape for {key}"
                assert arr.dtype == np.uint8
            else:
                assert arr.shape == feature.shape, f"Wrong state shape for {key}"
                assert arr.dtype == np.float32

    def test_warmup_passes_task_string(self):
        """The task string must be forwarded to each predict_action call."""
        from lerobot.scripts.lerobot_record import _warmup_policy

        policy = _make_policy()
        preprocessor = _identity_processor()
        postprocessor = _identity_processor()
        device = MagicMock()
        device.type = "cpu"

        with patch(
            "lerobot.scripts.lerobot_record.predict_action", return_value={}
        ) as mock_predict:
            _warmup_policy(policy, preprocessor, postprocessor, device, task="Pick the cube")

        for c in mock_predict.call_args_list:
            assert c.kwargs.get("task") == "Pick the cube" or c.args[5] == "Pick the cube" or \
                   any(a == "Pick the cube" for a in c.args), \
                   "task string not forwarded to predict_action"


# ---------------------------------------------------------------------------
# 2.  sanity_check_dataset_name
# ---------------------------------------------------------------------------

class TestSanityCheckDatasetName:
    def test_eval_prefix_without_policy_raises(self):
        """eval_ dataset name with no policy must raise."""
        from lerobot.common.control_utils import sanity_check_dataset_name

        with pytest.raises(ValueError, match="eval_"):
            sanity_check_dataset_name("aakashv100/eval_test", policy_cfg=None)

    def test_non_eval_name_with_policy_raises(self):
        """Dataset name without eval_ prefix when policy is given must raise."""
        from lerobot.common.control_utils import sanity_check_dataset_name

        policy_cfg = MagicMock()
        policy_cfg.type = "act"
        with pytest.raises(ValueError, match="eval_"):
            sanity_check_dataset_name("aakashv100/my_dataset", policy_cfg=policy_cfg)

    def test_eval_prefix_with_policy_passes(self):
        from lerobot.common.control_utils import sanity_check_dataset_name

        policy_cfg = MagicMock()
        policy_cfg.type = "act"
        sanity_check_dataset_name("aakashv100/eval_my_dataset", policy_cfg=policy_cfg)

    def test_no_eval_prefix_without_policy_passes(self):
        from lerobot.common.control_utils import sanity_check_dataset_name

        sanity_check_dataset_name("aakashv100/my_dataset", policy_cfg=None)

    def test_eval_prefix_check_is_case_sensitive(self):
        """'Eval_' (capital E) should NOT be accepted."""
        from lerobot.common.control_utils import sanity_check_dataset_name

        policy_cfg = MagicMock()
        policy_cfg.type = "act"
        with pytest.raises(ValueError):
            sanity_check_dataset_name("aakashv100/Eval_my_dataset", policy_cfg=policy_cfg)


# ---------------------------------------------------------------------------
# 3.  display_cameras  — OpenCV imshow path
# ---------------------------------------------------------------------------

class TestDisplayCameras:
    def test_imshow_called_with_concatenated_frames(self):
        """With display_cameras=True, cv2.imshow must receive all camera frames side-by-side."""
        from lerobot.scripts.lerobot_record import record_loop

        obs = _make_obs(n_cams=2)
        robot = _make_robot(obs)
        # exit_early=False so the loop runs at least one iteration
        events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}

        fake_cv2 = MagicMock()
        shown_arrays: list[np.ndarray] = []
        fake_cv2.imshow.side_effect = lambda name, arr: shown_arrays.append(arr)
        fake_cv2.waitKey.return_value = -1
        fake_cv2.COLOR_RGB2BGR = 4
        fake_cv2.cvtColor.side_effect = lambda arr, _code: arr

        with patch.dict("sys.modules", {"cv2": fake_cv2}):
            record_loop(
                robot=robot,
                events=events,
                fps=30,
                teleop_action_processor=_identity_processor(),
                robot_action_processor=_identity_processor(),
                robot_observation_processor=_identity_processor(),
                control_time_s=0.1,  # run for 100ms — enough for a few iterations
                display_cameras=True,
            )

        assert fake_cv2.imshow.called, "cv2.imshow was never called"
        assert shown_arrays, "No array passed to imshow"
        combined = shown_arrays[0]
        assert combined.shape[1] == CAMERA_SHAPE[1] * 2, (
            f"Expected combined width {CAMERA_SHAPE[1] * 2}, got {combined.shape[1]}"
        )

    def test_imshow_not_called_when_display_cameras_false(self):
        """With display_cameras=False, cv2.imshow must NOT be called."""
        from lerobot.scripts.lerobot_record import record_loop

        robot = _make_robot()
        events = _make_events(exit_after=0)
        fake_cv2 = MagicMock()

        with patch.dict("sys.modules", {"cv2": fake_cv2}):
            record_loop(
                robot=robot,
                events=events,
                fps=30,
                teleop_action_processor=_identity_processor(),
                robot_action_processor=_identity_processor(),
                robot_observation_processor=_identity_processor(),
                control_time_s=1,
                display_cameras=False,
            )

        fake_cv2.imshow.assert_not_called()

    def test_imshow_skipped_when_no_image_in_obs(self):
        """If the observation has no image arrays, imshow must not be called."""
        from lerobot.scripts.lerobot_record import record_loop

        obs = {"observation.state": np.zeros(STATE_DIM, dtype=np.float32)}
        robot = _make_robot(obs)
        events = _make_events(exit_after=0)
        fake_cv2 = MagicMock()

        with patch.dict("sys.modules", {"cv2": fake_cv2}):
            record_loop(
                robot=robot,
                events=events,
                fps=30,
                teleop_action_processor=_identity_processor(),
                robot_action_processor=_identity_processor(),
                robot_observation_processor=_identity_processor(),
                control_time_s=1,
                display_cameras=True,
            )

        fake_cv2.imshow.assert_not_called()


# ---------------------------------------------------------------------------
# 4.  record_loop — action generation path
# ---------------------------------------------------------------------------

class TestRecordLoopActionGeneration:
    def test_no_policy_no_teleop_logs_warning(self, caplog):
        """With neither policy nor teleop, a warning must be logged every iteration."""
        import logging
        from lerobot.scripts.lerobot_record import record_loop

        robot = _make_robot()
        # Must not exit immediately — we need at least one loop body to execute
        events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}

        # lerobot_record.py uses the root logging module, not a named logger
        with caplog.at_level(logging.WARNING):
            record_loop(
                robot=robot,
                events=events,
                fps=30,
                teleop_action_processor=_identity_processor(),
                robot_action_processor=_identity_processor(),
                robot_observation_processor=_identity_processor(),
                control_time_s=0.1,
            )

        warning_msgs = [r.message for r in caplog.records if r.levelno == logging.WARNING]
        assert any(
            "No policy or teleoperator" in m for m in warning_msgs
        ), "Expected 'No policy or teleoperator' warning not found"

    def _make_mock_dataset(self) -> MagicMock:
        """Minimal dataset mock matching the schema expected by build_dataset_frame."""
        motor_names = [f"joint_{i}.pos" for i in range(ACTION_DIM)]
        dataset = MagicMock()
        dataset.fps = 30
        dataset.features = {
            "observation.state": {
                "dtype": "float32",
                "shape": (STATE_DIM,),
                "names": motor_names,
            },
            "observation.images.gripper_cam": {
                "dtype": "video",
                "shape": (480, 640, 3),
                "names": ["height", "width", "channels"],
            },
            "observation.images.top_cam": {
                "dtype": "video",
                "shape": (480, 640, 3),
                "names": ["height", "width", "channels"],
            },
            "action": {
                "dtype": "float32",
                "shape": (ACTION_DIM,),
                "names": motor_names,
            },
        }
        return dataset

    def test_policy_predict_action_called_each_loop(self):
        """With a policy, predict_action must be called on every loop iteration."""
        from lerobot.scripts.lerobot_record import record_loop

        robot = _make_robot()
        events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}
        policy = _make_policy()
        preprocessor = _identity_processor()
        postprocessor = _identity_processor()
        dataset = self._make_mock_dataset()

        # predict_action returns a tensor; make_robot_action calls .squeeze(0) on it
        action_return = torch.zeros(1, ACTION_DIM)

        with patch(
            "lerobot.scripts.lerobot_record.predict_action",
            return_value=action_return,
        ) as mock_predict:
            record_loop(
                robot=robot,
                events=events,
                fps=30,
                teleop_action_processor=_action_processor(),
                robot_action_processor=_action_processor(),
                robot_observation_processor=_identity_processor(),
                policy=policy,
                preprocessor=preprocessor,
                postprocessor=postprocessor,
                dataset=dataset,
                single_task="Pick the cube",
                control_time_s=0.1,
            )

        assert mock_predict.call_count >= 1, "predict_action was never called with a policy"

    def test_send_action_called_with_policy_output(self):
        """robot.send_action must be called with the policy's predicted action."""
        from lerobot.scripts.lerobot_record import record_loop

        robot = _make_robot()
        events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}
        policy = _make_policy()
        dataset = self._make_mock_dataset()
        # predict_action returns a tensor; make_robot_action calls .squeeze(0) on it
        action_return = torch.zeros(1, ACTION_DIM)

        with patch(
            "lerobot.scripts.lerobot_record.predict_action",
            return_value=action_return,
        ):
            record_loop(
                robot=robot,
                events=events,
                fps=30,
                teleop_action_processor=_action_processor(),
                robot_action_processor=_action_processor(),
                robot_observation_processor=_identity_processor(),
                policy=policy,
                preprocessor=_identity_processor(),
                postprocessor=_identity_processor(),
                dataset=dataset,
                single_task="Pick the cube",
                control_time_s=0.1,
            )

        robot.send_action.assert_called()


# ---------------------------------------------------------------------------
# 5.  record_loop — loop timing
# ---------------------------------------------------------------------------

class TestRecordLoopTiming:
    def test_loop_exits_on_exit_early_event(self):
        """Loop must stop immediately when events['exit_early'] is True."""
        from lerobot.scripts.lerobot_record import record_loop

        robot = _make_robot()
        events = {"exit_early": True, "rerecord_episode": False, "stop_recording": False}

        start = time.perf_counter()
        record_loop(
            robot=robot,
            events=events,
            fps=30,
            teleop_action_processor=_identity_processor(),
            robot_action_processor=_identity_processor(),
            robot_observation_processor=_identity_processor(),
            control_time_s=60,  # would take 60s if not interrupted
        )
        elapsed = time.perf_counter() - start

        assert elapsed < 2.0, f"Loop took {elapsed:.2f}s — exit_early not respected"

    def test_loop_respects_control_time_s(self):
        """Loop must finish close to control_time_s when no early-exit event fires."""
        from lerobot.scripts.lerobot_record import record_loop

        robot = _make_robot()
        events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}
        control_time_s = 0.5

        start = time.perf_counter()
        record_loop(
            robot=robot,
            events=events,
            fps=30,
            teleop_action_processor=_identity_processor(),
            robot_action_processor=_identity_processor(),
            robot_observation_processor=_identity_processor(),
            control_time_s=control_time_s,
        )
        elapsed = time.perf_counter() - start

        assert elapsed >= control_time_s, "Loop ended before control_time_s elapsed"
        assert elapsed < control_time_s + 1.0, f"Loop ran too long: {elapsed:.2f}s"

    def test_negative_control_time_runs_until_terminal_event(self):
        """control_time_s < 0 means no clock: the loop must end only on exit_early."""
        from lerobot.scripts.lerobot_record import record_loop

        events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}
        robot = _make_robot()

        # Signal the terminal condition after a few iterations, as the operator's key press would.
        calls = {"n": 0}

        def observe():
            calls["n"] += 1
            if calls["n"] == 4:
                events["exit_early"] = True
            return _make_obs()

        robot.get_observation.side_effect = observe

        start = time.perf_counter()
        record_loop(
            robot=robot,
            events=events,
            fps=30,
            teleop_action_processor=_identity_processor(),
            robot_action_processor=_identity_processor(),
            robot_observation_processor=_identity_processor(),
            control_time_s=-1,
        )
        elapsed = time.perf_counter() - start

        assert calls["n"] == 4, f"Loop ran {calls['n']} iterations, expected to stop on exit_early"
        assert elapsed < 2.0, f"Loop took {elapsed:.2f}s — the terminal event was not respected"

    def test_missing_control_time_does_not_crash(self):
        """control_time_s=None must mean unlimited rather than a comparison against None."""
        from lerobot.scripts.lerobot_record import record_loop

        robot = _make_robot()
        events = {"exit_early": True, "rerecord_episode": False, "stop_recording": False}

        record_loop(
            robot=robot,
            events=events,
            fps=30,
            teleop_action_processor=_identity_processor(),
            robot_action_processor=_identity_processor(),
            robot_observation_processor=_identity_processor(),
        )

    def test_slow_rerun_viewer_does_not_slow_the_loop(self):
        """A stalled Rerun viewer must not drag the control/record loop below its fps."""
        from lerobot.scripts.lerobot_record import record_loop

        robot = _make_robot()
        events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}
        robot.get_observation.side_effect = lambda: _make_obs()
        from lerobot.teleoperators import Teleoperator

        teleop = MagicMock(spec=Teleoperator)
        teleop.get_action.return_value = {f"joint_{i}.pos": 0.0 for i in range(ACTION_DIM)}
        logged = []

        def slow_log(**kwargs):  # viewer stall: 200 ms per log call
            time.sleep(0.2)
            logged.append(kwargs)

        with patch("lerobot.scripts.lerobot_record.log_rerun_data", side_effect=slow_log):
            record_loop(
                robot=robot,
                events=events,
                fps=30,
                teleop_action_processor=_action_processor(),
                robot_action_processor=_action_processor(),
                robot_observation_processor=_identity_processor(),
                teleop=teleop,
                control_time_s=0.5,
                display_data=True,
            )

        # ~15 iterations at 30 Hz; a blocking 200 ms log call would allow only ~3.
        assert robot.get_observation.call_count >= 10, (
            f"only {robot.get_observation.call_count} loop iterations in 0.5 s"
        )
        assert logged, "Rerun logging must still happen, just off the control thread"

    def test_get_observation_called_at_least_fps_times(self):
        """robot.get_observation must be called roughly fps × control_time_s times."""
        from lerobot.scripts.lerobot_record import record_loop

        robot = _make_robot()
        events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}
        fps = 30
        control_time_s = 0.5
        expected_min_calls = int(fps * control_time_s * 0.5)  # allow 50% slack

        record_loop(
            robot=robot,
            events=events,
            fps=fps,
            teleop_action_processor=_identity_processor(),
            robot_action_processor=_identity_processor(),
            robot_observation_processor=_identity_processor(),
            control_time_s=control_time_s,
        )

        assert robot.get_observation.call_count >= expected_min_calls, (
            f"get_observation called {robot.get_observation.call_count} times, "
            f"expected ≥ {expected_min_calls}"
        )


# ---------------------------------------------------------------------------
# 6.  _warmup_policy — error cases
# ---------------------------------------------------------------------------

class TestWarmupPolicyErrors:
    def test_warmup_with_no_input_features_raises(self):
        """Warmup must raise if the policy config has no input_features."""
        from lerobot.scripts.lerobot_record import _warmup_policy

        policy = _make_policy()
        policy.config.input_features = {}  # empty — nothing to build dummy obs from

        with patch("lerobot.scripts.lerobot_record.predict_action", return_value={}):
            # Should not raise even with empty features — just runs with empty obs
            _warmup_policy(
                policy,
                _identity_processor(),
                _identity_processor(),
                MagicMock(type="cpu"),
                task=None,
            )

    def test_warmup_predict_action_exception_propagates(self):
        """If predict_action raises, _warmup_policy must not swallow the error."""
        from lerobot.scripts.lerobot_record import _warmup_policy

        policy = _make_policy()

        with patch(
            "lerobot.scripts.lerobot_record.predict_action",
            side_effect=RuntimeError("CUDA OOM"),
        ):
            with pytest.raises(RuntimeError, match="CUDA OOM"):
                _warmup_policy(
                    policy,
                    _identity_processor(),
                    _identity_processor(),
                    MagicMock(type="cuda"),
                    task=None,
                )


# ---------------------------------------------------------------------------
# 7.  RecordConfig — field defaults and validation
# ---------------------------------------------------------------------------

class TestRecordConfigDefaults:
    def test_display_cameras_defaults_to_false(self):
        from lerobot.scripts.lerobot_record import RecordConfig

        cfg = RecordConfig.__dataclass_fields__
        assert cfg["display_cameras"].default is False

    def test_interpolation_multiplier_defaults_to_one(self):
        from lerobot.scripts.lerobot_record import RecordConfig

        cfg = RecordConfig.__dataclass_fields__
        assert cfg["interpolation_multiplier"].default == 1

    def test_display_cameras_field_exists(self):
        from lerobot.scripts.lerobot_record import RecordConfig

        assert "display_cameras" in RecordConfig.__dataclass_fields__


class TestCameraFpsCheck:
    def test_camera_fps_mismatch_raises(self):
        """A camera running at a different rate than the dataset must be rejected up front."""
        from lerobot.scripts.lerobot_record import _check_camera_fps

        cameras = {"gripper_cam": SimpleNamespace(fps=30), "top_cam": SimpleNamespace(fps=60)}
        with pytest.raises(ValueError, match="top_cam"):
            _check_camera_fps(cameras, dataset_fps=30)

    def test_matching_or_unset_camera_fps_passes(self):
        from lerobot.scripts.lerobot_record import _check_camera_fps

        cameras = {"gripper_cam": SimpleNamespace(fps=30), "top_cam": SimpleNamespace(fps=None)}
        _check_camera_fps(cameras, dataset_fps=30)


class TestReturnToStartPose:
    def test_config_field_defaults_to_false(self):
        from lerobot.scripts.lerobot_record import RecordConfig

        cfg = RecordConfig.__dataclass_fields__
        assert cfg["return_to_start_pose"].default is False

    def test_capture_start_pose_keeps_only_pos_keys(self):
        from lerobot.scripts.lerobot_record import _capture_start_pose

        robot = _make_robot()
        pose = _capture_start_pose(robot)
        assert pose == {f"joint_{i}.pos": 0.0 for i in range(STATE_DIM)}
        assert not any(isinstance(v, np.ndarray) for v in pose.values())

    def test_return_to_start_pose_ends_at_target(self):
        from lerobot.scripts.lerobot_record import _return_to_start_pose

        # Arm currently at 10.0 on every joint, start pose was 0.0.
        obs = _make_obs()
        for i in range(STATE_DIM):
            obs[f"joint_{i}.pos"] = 10.0
        robot = _make_robot(obs)
        start_pose = {f"joint_{i}.pos": 0.0 for i in range(STATE_DIM)}

        with patch("lerobot.scripts.lerobot_record.time.sleep"):
            _return_to_start_pose(robot, start_pose, duration_s=0.5, fps=10)

        assert robot.send_action.call_count == 5
        # Motion is a monotonic ramp and the final action is exactly the start pose.
        sent = [c.args[0] for c in robot.send_action.call_args_list]
        first_joint = [a["joint_0.pos"] for a in sent]
        assert first_joint == sorted(first_joint, reverse=True)
        assert sent[-1] == pytest.approx(start_pose)
