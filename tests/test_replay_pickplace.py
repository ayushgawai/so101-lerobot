"""Unit tests for replay_pickplace_lib pure helpers (no robot/hardware)."""

from __future__ import annotations

import json
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from replay_pickplace_lib import (  # noqa: E402
    ReplayConfigError,
    available_positions,
    episodes_for_position,
    load_position_log,
    sample_episodes,
)


def _write_log(tmp_path: Path, rows: list[dict]) -> Path:
    path = tmp_path / "pickup_positions.jsonl"
    path.write_text("\n".join(json.dumps(r) for r in rows) + "\n", encoding="utf-8")
    return path


def test_load_position_log_ok(tmp_path: Path):
    path = _write_log(
        tmp_path,
        [
            {"episode_index": 0, "pickup_position": 6},
            {"episode_index": 1, "pickup_position": 6},
            {"episode_index": 2, "pickup_position": 7},
        ],
    )
    rows = load_position_log(path)
    assert len(rows) == 3
    assert available_positions(rows) == [6, 7]
    assert episodes_for_position(rows, 6) == [0, 1]
    assert episodes_for_position(rows, 7) == [2]
    assert episodes_for_position(rows, 99) == []


def test_load_position_log_missing(tmp_path: Path):
    with pytest.raises(ReplayConfigError, match="not found"):
        load_position_log(tmp_path / "missing.jsonl")


def test_load_position_log_empty(tmp_path: Path):
    path = tmp_path / "pickup_positions.jsonl"
    path.write_text("\n\n", encoding="utf-8")
    with pytest.raises(ReplayConfigError, match="empty"):
        load_position_log(path)


def test_load_position_log_bad_json(tmp_path: Path):
    path = tmp_path / "pickup_positions.jsonl"
    path.write_text("{not-json}\n", encoding="utf-8")
    with pytest.raises(ReplayConfigError, match="Invalid JSON"):
        load_position_log(path)


def test_load_position_log_missing_keys(tmp_path: Path):
    path = _write_log(tmp_path, [{"episode_index": 0}])
    with pytest.raises(ReplayConfigError, match="missing"):
        load_position_log(path)


def test_sample_episodes_without_replacement():
    rng = random.Random(0)
    out = sample_episodes([10, 11, 12, 13, 14], 3, rng=rng)
    assert len(out) == 3
    assert len(set(out)) == 3
    assert set(out).issubset({10, 11, 12, 13, 14})


def test_sample_episodes_reproducible_seed():
    a = sample_episodes(list(range(20)), 5, rng=random.Random(42))
    b = sample_episodes(list(range(20)), 5, rng=random.Random(42))
    assert a == b


def test_sample_episodes_n_too_large():
    with pytest.raises(ReplayConfigError, match="only 2 available"):
        sample_episodes([1, 2], 5)


def test_sample_episodes_allow_fewer():
    out = sample_episodes([1, 2], 5, allow_fewer=True, rng=random.Random(1))
    assert sorted(out) == [1, 2]


def test_sample_episodes_empty():
    with pytest.raises(ReplayConfigError, match="No episodes"):
        sample_episodes([], 1)


def test_sample_episodes_n_zero():
    with pytest.raises(ReplayConfigError, match=">= 1"):
        sample_episodes([1, 2], 0)


def test_episodes_for_position_dedupes():
    log = [
        {"episode_index": 5, "pickup_position": 3},
        {"episode_index": 5, "pickup_position": 3},
        {"episode_index": 8, "pickup_position": 3},
    ]
    assert episodes_for_position(log, 3) == [5, 8]


def test_wait_for_right_arrow_ready_skip_stop():
    from replay_pickplace_lib import wait_for_right_arrow
    import threading

    events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}

    def press_right():
        time.sleep(0.05)
        events["exit_early"] = True

    import time

    t = threading.Thread(target=press_right)
    t.start()
    assert wait_for_right_arrow(events, poll_s=0.01) == "ready"
    t.join()

    def press_left():
        time.sleep(0.05)
        events["rerecord_episode"] = True
        events["exit_early"] = True

    t = threading.Thread(target=press_left)
    t.start()
    assert wait_for_right_arrow(events, poll_s=0.01) == "skip"
    t.join()

    def press_esc():
        time.sleep(0.05)
        events["stop_recording"] = True

    t = threading.Thread(target=press_esc)
    t.start()
    assert wait_for_right_arrow(events, poll_s=0.01) == "stop"
    t.join()
