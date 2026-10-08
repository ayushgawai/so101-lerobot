"""Trim the idle lead-in and trailing frames from every episode of a LeRobot dataset.

Why: in `aakashv100/so101-pick-cube-v2` the operator holds the start pose for a median of
95 frames before beginning the reach, and holds still again after releasing the cube. Those
frames are 16% of the dataset and they teach "at the start pose, output the start pose".
ACT regresses to a conditional mean and leaks out of that mode; SmolVLA samples from the
learned distribution and can reproduce it faithfully, which is the mechanism behind the
11-51 s start-up stalls in SmolVLA_training_report.md. Removing the frames removes the mode.

Two passes. The first reads only the parquet columns (no video decode, ~3 s) and reports the
trim boundaries; with --dry-run it stops there. The second writes a new dataset frame by
frame, which requires decoding and re-encoding every kept frame and takes hours.

Examples:
    # Report boundaries only
    python scripts/trim_idle_frames.py --dry-run

    # Build the trimmed dataset
    python scripts/trim_idle_frames.py --new-repo-id aakashv100/so101-pick-cube-v2-trimmed

    # Validate the write path on a couple of episodes first
    python scripts/trim_idle_frames.py --episodes 0 1 --new-repo-id local/trim-smoke
"""

import argparse
import os
from pathlib import Path

import numpy as np
import pandas as pd
from tqdm import tqdm

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.constants import DEFAULT_FEATURES

DEFAULT_REPO_ID = "aakashv100/so101-pick-cube-v2"


def default_root(repo_id: str) -> Path:
    return Path(os.environ["USERPROFILE"]) / ".cache" / "huggingface" / "lerobot" / Path(repo_id)


def episode_states(root: Path) -> dict[int, np.ndarray]:
    """Return {episode_index: (T, 6) state array}, read straight from the parquet files."""
    frames = [
        pd.read_parquet(path, columns=["episode_index", "frame_index", "observation.state"])
        for path in sorted(root.glob("data/**/*.parquet"))
    ]
    df = pd.concat(frames, ignore_index=True).sort_values(["episode_index", "frame_index"])
    return {
        int(ep): np.stack(group["observation.state"].to_numpy())
        for ep, group in df.groupby("episode_index", sort=True)
    }


def trim_bounds(
    states: np.ndarray, motion_threshold: float, static_threshold: float, keep_lead: int, keep_trail: int
) -> tuple[int, int]:
    """Return the [start, end) frame range to keep for one episode.

    `start` is `keep_lead` frames before the first frame whose joint deviation from the start
    pose exceeds `motion_threshold`. `end` is `keep_trail` frames after the last frame that
    moved by more than `static_threshold`. Both are clamped to the episode.
    """
    deviation = np.abs(states - states[0]).sum(axis=1)
    moving = np.flatnonzero(deviation > motion_threshold)
    start = max(0, int(moving[0]) - keep_lead) if len(moving) else 0

    step = np.abs(np.diff(states, axis=0)).sum(axis=1)
    stepping = np.flatnonzero(step > static_threshold)
    end = min(len(states), int(stepping[-1]) + 1 + keep_trail) if len(stepping) else len(states)

    # A degenerate episode (never moved) would otherwise produce an empty range.
    if end <= start:
        start, end = 0, len(states)
    return start, end


def report(bounds: dict[int, tuple[int, int, int]]) -> None:
    print(f"\n{'episode':>8}  {'frames':>7}  {'keep':>7}  {'cut lead':>9}  {'cut trail':>10}")
    for ep in sorted(bounds):
        start, end, length = bounds[ep]
        print(f"{ep:>8}  {length:>7}  {end - start:>7}  {start:>9}  {length - end:>10}")

    total = sum(length for _, _, length in bounds.values())
    kept = sum(end - start for start, end, _ in bounds.values())
    lead = sum(start for start, _, _ in bounds.values())
    trail = sum(length - end for _, end, length in bounds.values())
    print(
        f"\n{len(bounds)} episodes: {total} frames -> {kept} kept "
        f"({100 * kept / total:.1f}%), cut {lead} lead-in + {trail} trailing "
        f"({100 * (total - kept) / total:.1f}%)"
    )


def build(
    src: LeRobotDataset,
    bounds: dict[int, tuple[int, int, int]],
    episodes: list[int],
    new_repo_id: str,
    new_root: Path | None,
) -> None:
    features = {k: v for k, v in src.meta.features.items() if k not in DEFAULT_FEATURES}
    dst = LeRobotDataset.create(
        new_repo_id,
        fps=src.meta.fps,
        features=features,
        root=new_root,
        robot_type=src.meta.robot_type,
        use_videos=len(src.meta.video_keys) > 0,
    )

    # Episodes are stored consecutively, so an episode's frames start at the sum of the
    # lengths before it. Offsets span every episode even when only some are written,
    # because `src` holds the whole dataset. Asserted per episode rather than trusted.
    offsets, running = {}, 0
    for ep in sorted(bounds):
        offsets[ep] = running
        running += bounds[ep][2]

    copy_keys = set(features)
    for ep in episodes:
        start, end, length = bounds[ep]
        base = offsets[ep]
        first = src[base]
        assert int(first["episode_index"]) == ep and int(first["frame_index"]) == 0, (
            f"episode {ep} does not begin at dataset index {base}; the offset assumption is wrong"
        )

        for i in tqdm(range(start, end), desc=f"episode {ep} ({end - start}/{length})", leave=False):
            item = src[base + i]
            frame = {key: item[key] for key in copy_keys}
            # The source returns frames channel-first; 0.6.1's add_frame accepts only the
            # channel-last layout the cameras record in.
            for key in src.meta.video_keys:
                frame[key] = frame[key].permute(1, 2, 0).numpy()
            frame["task"] = item["task"]
            dst.add_frame(frame)
        dst.save_episode()

    dst.finalize()
    print(f"\nwrote {new_repo_id} to {dst.root}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-id", default=DEFAULT_REPO_ID)
    parser.add_argument("--root", default=None, help="source dataset dir (default: LeRobot cache)")
    parser.add_argument("--new-repo-id", default=None, help="default: <repo-id>-trimmed")
    parser.add_argument("--new-root", default=None)
    parser.add_argument("--episodes", type=int, nargs="*", default=None, help="default: all")
    parser.add_argument(
        "--motion-threshold",
        type=float,
        default=15.0,
        help="summed joint deviation from the start pose that counts as motion (units of the action space)",
    )
    parser.add_argument(
        "--static-threshold",
        type=float,
        default=0.5,
        help="summed per-frame joint change below which a frame counts as still",
    )
    parser.add_argument("--keep-lead", type=int, default=15, help="idle frames to keep before first motion")
    parser.add_argument("--keep-trail", type=int, default=15, help="idle frames to keep after last motion")
    parser.add_argument("--dry-run", action="store_true", help="report boundaries and exit")
    args = parser.parse_args()

    root = Path(args.root) if args.root else default_root(args.repo_id)
    states = episode_states(root)
    if args.episodes is not None:
        missing = sorted(set(args.episodes) - set(states))
        if missing:
            raise SystemExit(f"episodes not in dataset: {missing}")

    bounds = {}
    for ep, ep_states in states.items():
        start, end = trim_bounds(
            ep_states, args.motion_threshold, args.static_threshold, args.keep_lead, args.keep_trail
        )
        bounds[ep] = (start, end, len(ep_states))

    report(bounds)

    if args.dry_run:
        print("\n--dry-run: nothing written")
        return

    episodes = sorted(args.episodes) if args.episodes is not None else sorted(bounds)
    new_repo_id = args.new_repo_id or f"{args.repo_id}-trimmed"
    src = LeRobotDataset(args.repo_id, root=root, return_uint8=True)
    build(src, bounds, episodes, new_repo_id, Path(args.new_root) if args.new_root else None)


if __name__ == "__main__":
    main()
