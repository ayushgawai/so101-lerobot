"""Episodic rollout (lerobot-rollout --strategy.type=episodic) behaviors the SO-101 eval protocol relies on.

run_eval.ps1 ran policy evals through lerobot-record before LeRobot 0.6.1; these are the record-side fixes
carried over: no-time-limit episodes, stale key presses, empty episodes, serial (Windows-safe) encoding,
the OpenCV camera window, and eval_-prefixed dataset names. No hardware needed.
"""

from __future__ import annotations

import threading
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from lerobot.rollout.configs import EpisodicStrategyConfig
from lerobot.rollout.strategies.episodic import EpisodicStrategy

EPISODIC = "lerobot.rollout.strategies.episodic"


def _strategy(events: dict) -> EpisodicStrategy:
    strategy = EpisodicStrategy(EpisodicStrategyConfig(reset_to_initial_position=False))
    strategy._events = events
    strategy._engine = MagicMock()
    strategy._interpolator = MagicMock(get_control_interval=lambda fps: 1 / fps)
    return strategy


def _ctx(*, num_episodes=2, episode_time_s=-1, display_cameras=False):
    ctx = MagicMock()
    ctx.runtime.shutdown_event = threading.Event()
    cfg = ctx.runtime.cfg
    cfg.fps = 30
    cfg.play_sounds = False
    cfg.display_data = False
    cfg.display_cameras = display_cameras
    cfg.use_torch_compile = False
    cfg.dataset.num_episodes = num_episodes
    cfg.dataset.episode_time_s = episode_time_s
    cfg.dataset.reset_time_s = 0
    ctx.hardware.teleop = None
    ctx.processors.robot_observation_processor.side_effect = lambda obs: obs
    return ctx


def _events(**overrides):
    events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}
    events.update(overrides)
    return events


def test_negative_episode_time_runs_until_right_arrow():
    """run_eval.ps1 -EpisodeTime -1: the episode must run until the operator ends it, not record 0 frames."""
    events = _events()
    strategy = _strategy(events)
    ctx = _ctx()
    robot = MagicMock()
    calls = {"n": 0}

    def observe():
        calls["n"] += 1
        if calls["n"] == 4:
            events["exit_early"] = True
        return {"shoulder_pan.pos": 0.0}

    robot.get_observation.side_effect = observe
    with (
        patch(f"{EPISODIC}.send_next_action", return_value=None),
        patch.object(strategy, "_handle_warmup", return_value=False),
        patch.object(strategy, "_process_observation_and_notify", side_effect=lambda p, o: o),
    ):
        strategy._policy_loop(ctx, robot, events, features={}, fps=30, control_time_s=-1, dataset=MagicMock(), single_task="t")

    assert calls["n"] == 4


def test_stale_key_is_cleared_and_empty_episode_is_not_saved_serially():
    events = _events(exit_early=True)  # a right-arrow pressed while the previous episode was saving
    strategy = _strategy(events)
    ctx = _ctx(num_episodes=2)
    dataset = ctx.data.dataset
    dataset.num_episodes = 0
    pending = iter([False, True, True, False])  # 1st episode empty (skipped), then 2 real ones, then none left

    dataset.has_pending_frames.side_effect = lambda: next(pending)
    exit_early_at_start = []

    def policy_loop(**kwargs):
        exit_early_at_start.append(events["exit_early"])

    with (
        patch(f"{EPISODIC}.VideoEncodingManager", MagicMock()),
        patch.object(strategy, "_policy_loop", side_effect=lambda **kw: policy_loop(**kw)),
        patch.object(strategy, "_reset_loop"),
    ):
        strategy.run(ctx)

    assert exit_early_at_start[0] is False, "stale exit_early must be cleared before an episode starts"
    saves = dataset.save_episode.call_args_list
    assert len(saves) == 2, f"expected 2 saved episodes (empty one skipped), got {len(saves)}"
    assert all(c.kwargs.get("parallel_encoding") is False for c in saves), "parallel encoding crashes on Windows"


def test_display_cameras_shows_side_by_side_window():
    events = _events()
    strategy = _strategy(events)
    ctx = _ctx(display_cameras=True)
    robot = MagicMock()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    robot.get_observation.side_effect = lambda: (events.__setitem__("exit_early", True), {"a": frame, "b": frame})[1]
    fake_cv2 = MagicMock()
    fake_cv2.cvtColor.side_effect = lambda arr, _code: arr
    shown = []
    fake_cv2.imshow.side_effect = lambda name, arr: shown.append(arr)

    with (
        patch.dict("sys.modules", {"cv2": fake_cv2}),
        patch(f"{EPISODIC}.send_next_action", return_value=None),
        patch.object(strategy, "_handle_warmup", return_value=False),
        patch.object(strategy, "_process_observation_and_notify", side_effect=lambda p, o: o),
    ):
        strategy._policy_loop(ctx, robot, events, features={}, fps=30, control_time_s=1, dataset=MagicMock(), single_task="t")

    assert shown and shown[0].shape == (480, 1280, 3)


@pytest.mark.parametrize("name, ok", [("eval_so101-pick-cube-v2-fixed", True), ("rollout_x", True), ("my_data", False)])
def test_eval_and_rollout_dataset_prefixes(name, ok):
    from lerobot.rollout.context import check_rollout_dataset_name

    if ok:
        check_rollout_dataset_name(f"aakashv100/{name}")
    else:
        with pytest.raises(ValueError, match="eval_"):
            check_rollout_dataset_name(f"aakashv100/{name}")
