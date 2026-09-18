"""Pure helpers for pick-and-place replay (no robot / LeRobot imports)."""

from __future__ import annotations

import json
import random
import time
from pathlib import Path


class ReplayConfigError(ValueError):
    """Invalid user/dataset configuration for replay."""


def load_position_log(path: Path) -> list[dict]:
    if not path.is_file():
        raise ReplayConfigError(f"Position log not found: {path}")
    rows: list[dict] = []
    for i, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        line = line.strip()
        if not line:
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError as e:
            raise ReplayConfigError(f"Invalid JSON on line {i} of {path}: {e}") from e
        if "episode_index" not in row or "pickup_position" not in row:
            raise ReplayConfigError(
                f"Line {i} missing episode_index or pickup_position: {row!r}"
            )
        rows.append(row)
    if not rows:
        raise ReplayConfigError(f"Position log is empty: {path}")
    return rows


def episodes_for_position(log: list[dict], position: int) -> list[int]:
    return sorted(
        {int(r["episode_index"]) for r in log if int(r["pickup_position"]) == position}
    )


def available_positions(log: list[dict]) -> list[int]:
    return sorted({int(r["pickup_position"]) for r in log})


def sample_episodes(
    candidates: list[int],
    n: int,
    *,
    rng: random.Random | None = None,
    allow_fewer: bool = False,
) -> list[int]:
    """Sample ``n`` episode indices without replacement."""
    if not candidates:
        raise ReplayConfigError("No episodes available to sample.")
    if n < 1:
        raise ReplayConfigError(f"Number of episodes must be >= 1 (got {n}).")
    rng = rng or random.Random()
    if n > len(candidates):
        if not allow_fewer:
            raise ReplayConfigError(
                f"Requested {n} episodes but only {len(candidates)} available "
                f"for this position. Choose <= {len(candidates)}, or pass allow_fewer."
            )
        n = len(candidates)
    return rng.sample(list(candidates), n)


def prompt_int(
    message: str,
    *,
    min_v: int,
    max_v: int | None = None,
    default: int | None = None,
) -> int | None:
    hint = f" [{default}]" if default is not None else ""
    range_txt = f"{min_v}-{max_v}" if max_v is not None else f">={min_v}"
    while True:
        try:
            raw = input(f"{message} ({range_txt}){hint} (q=quit): ").strip()
        except EOFError:
            return None
        if raw.lower() in {"q", "quit", "exit"}:
            return None
        if raw == "" and default is not None:
            return default
        try:
            value = int(raw)
        except ValueError:
            print(f"  Not a number: {raw!r}")
            continue
        if value < min_v or (max_v is not None and value > max_v):
            print(f"  Must be in {range_txt}.")
            continue
        return value


def wait_for_right_arrow(events: dict, *, poll_s: float = 0.05) -> str:
    """Block until Right arrow (ready), Left arrow (skip), or Esc (stop).

    Returns: 'ready' | 'skip' | 'stop'
    """
    events["exit_early"] = False
    events["rerecord_episode"] = False
    events["stop_recording"] = False
    print("  Press Right arrow when ready (Left=skip episode, Esc=quit session).")
    while True:
        if events.get("stop_recording"):
            events["exit_early"] = False
            events["rerecord_episode"] = False
            return "stop"
        if events.get("exit_early"):
            skip = bool(events.get("rerecord_episode"))
            events["exit_early"] = False
            events["rerecord_episode"] = False
            return "skip" if skip else "ready"
        time.sleep(poll_s)
