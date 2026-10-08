"""Episode flow of scripts/record_episodes.py (run_record.ps1): Enter -> handover -> record -> keys. No hardware."""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pandas as pd
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import record_episodes  # noqa: E402

TASK = "Pick up the cube and place it in the box"


class FakeDataset:
    def __init__(self):
        self.num_episodes = 0
        self.buffer = 0
        self.saves = []

    def has_pending_frames(self):
        return self.buffer > 0

    def clear_episode_buffer(self):
        self.buffer = 0

    def save_episode(self, **kwargs):
        self.saves.append(kwargs)
        self.num_episodes += 1
        self.buffer = 0


def _run(script, *, inputs, num_episodes=0, stale_keys=None):
    """script: one dict per record_loop call -> {'frames': n, 'key': None|'right'|'left'|'esc'}."""
    events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}
    events.update(stale_keys or {})
    dataset = FakeDataset()
    calls = iter(script)
    seen_events_at_start = []
    order = []

    def fake_record_loop(**kw):
        order.append("record")
        seen_events_at_start.append(dict(events))
        assert kw["control_time_s"] == -1 and kw["single_task"] == TASK and kw["dataset"] is dataset
        step = next(calls)
        dataset.buffer += step["frames"]
        if step["key"] == "left":
            events["rerecord_episode"] = True
        elif step["key"] == "esc":
            events["stop_recording"] = True

    with (
        patch.object(record_episodes, "record_loop", side_effect=fake_record_loop),
        patch.object(record_episodes, "hand_over_to_leader", side_effect=lambda *a: order.append("handover")),
        patch.object(record_episodes, "log_say"),
        patch("builtins.input", side_effect=list(inputs)),
    ):
        saved = record_episodes.collect_episodes(
            robot=MagicMock(), teleop=MagicMock(), dataset=dataset, events=events,
            processors=(MagicMock(), MagicMock(), MagicMock()), task=TASK, fps=30,
            num_episodes=num_episodes, display_data=False, display_cameras=False, play_sounds=False,
        )
    return SimpleNamespace(saved=saved, dataset=dataset, seen=seen_events_at_start, order=order)


def test_right_arrow_saves_serially_and_prompts_again():
    r = _run([{"frames": 100, "key": "right"}, {"frames": 80, "key": "right"}], inputs=["", "", "q"])
    assert r.saved == 2 and r.dataset.num_episodes == 2
    assert all(s == {"parallel_encoding": False} for s in r.dataset.saves)


def test_left_arrow_discards_and_redoes_the_same_episode():
    r = _run([{"frames": 50, "key": "left"}, {"frames": 90, "key": "right"}], inputs=["", "", "q"])
    assert r.saved == 1 and r.dataset.num_episodes == 1


def test_escape_discards_in_progress_episode_and_stops():
    r = _run([{"frames": 100, "key": "right"}, {"frames": 60, "key": "esc"}], inputs=["", ""])
    assert r.saved == 1 and r.dataset.buffer == 0


def test_q_at_prompt_quits_without_recording():
    r = _run([], inputs=["q"])
    assert r.saved == 0 and r.order == []


def test_stale_arrow_keys_from_the_prompt_are_cleared():
    r = _run([{"frames": 100, "key": "right"}], inputs=["", "q"], stale_keys={"exit_early": True, "rerecord_episode": True})
    assert r.seen[0]["exit_early"] is False and r.seen[0]["rerecord_episode"] is False
    assert r.saved == 1


def test_follower_is_handed_over_before_every_recording():
    r = _run([{"frames": 10, "key": "left"}, {"frames": 10, "key": "right"}], inputs=["", "", "q"])
    assert r.order == ["handover", "record", "handover", "record"]


def test_empty_episode_is_not_saved():
    r = _run([{"frames": 0, "key": "right"}, {"frames": 30, "key": "right"}], inputs=["", "", "q"])
    assert r.saved == 1


def test_stops_after_num_episodes_without_another_prompt():
    r = _run([{"frames": 10, "key": "right"}, {"frames": 10, "key": "right"}], inputs=["", ""], num_episodes=2)
    assert r.saved == 2  # a third input() would raise StopIteration


def test_mixed_task_strings_are_refused():
    ds = SimpleNamespace(meta=SimpleNamespace(tasks=pd.DataFrame(index=["Pick up the cube from sheet position 3 and place it in the box"])))
    with pytest.raises(ValueError, match="task"):
        record_episodes.check_task_consistency(ds, TASK)
    record_episodes.check_task_consistency(SimpleNamespace(meta=SimpleNamespace(tasks=pd.DataFrame(index=[TASK]))), TASK)


def test_handover_slides_follower_from_its_pose_to_the_leader_pose():
    robot, teleop = MagicMock(), MagicMock()
    robot.get_observation.return_value = {"gripper.pos": 5.0, "shoulder_pan.pos": 1.0, "top_cam": object()}
    teleop.get_action.return_value = {"gripper.pos": 40.0, "shoulder_pan.pos": 20.0}
    passthrough = MagicMock(side_effect=lambda x: x[0])
    with patch.object(record_episodes, "follower_smooth_move_to") as move:
        record_episodes.hand_over_to_leader(robot, teleop, passthrough, passthrough, fps=30)
    _, current, target = move.call_args.args
    assert current == {"gripper.pos": 5.0, "shoulder_pan.pos": 1.0}
    assert target == {"gripper.pos": 40.0, "shoulder_pan.pos": 20.0}
    assert move.call_args.kwargs["duration_s"] == 1.0
