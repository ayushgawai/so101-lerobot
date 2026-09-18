"""Interactive random episode replay for the pick-and-place dataset.

Flow (mirrors the record script UX):
  1) Enter how many random episodes to replay
  2) Enter which sheet position to place the cube on
     (only episodes recorded at that position are sampled)
  3) For each episode: reset cube → Right arrow → replay → return to start pose
  4) Prompt again for another block, or q to quit

Keyboard:
  Right arrow — ready / continue after reset
  Left arrow  — skip this episode
  Esc         — stop session
  q at prompt — quit

Example:
  .\\scripts\\replay_pickplace.ps1
"""

from __future__ import annotations

import argparse
import logging
import random
import sys
import time
from pathlib import Path

# Allow `python scripts/replay_pickplace.py` to find sibling helpers.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from replay_pickplace_lib import (
    ReplayConfigError,
    available_positions,
    episodes_for_position,
    load_position_log,
    prompt_int,
    sample_episodes,
    wait_for_right_arrow,
)

from lerobot.common.control_utils import init_keyboard_listener, is_headless
from lerobot.datasets import LeRobotDataset
from lerobot.processor import make_default_robot_action_processor
from lerobot.robots import make_robot_from_config
from lerobot.robots.so_follower import SO101FollowerConfig
from lerobot.scripts.lerobot_record import _capture_start_pose, _return_to_start_pose
from lerobot.utils.constants import ACTION
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, log_say

POSITION_LOG_NAME = "pickup_positions.jsonl"


def replay_episode_on_robot(
    robot,
    *,
    repo_id: str,
    root: Path,
    episode: int,
    action_processor,
    play_sounds: bool,
) -> None:
    dataset = LeRobotDataset(repo_id, root=root, episodes=[episode])
    if dataset.num_frames < 1:
        raise ReplayConfigError(f"Episode {episode} has zero frames.")

    actions = dataset.select_columns(ACTION)
    names = dataset.features[ACTION]["names"]
    fps = dataset.fps

    log_say(f"Replaying episode {episode}", play_sounds, blocking=False)
    print(f"  Replaying episode {episode} ({dataset.num_frames} frames @ {fps} fps)...")

    for idx in range(dataset.num_frames):
        start_t = time.perf_counter()
        action_array = actions[idx][ACTION]
        action = {name: float(action_array[i]) for i, name in enumerate(names)}
        robot_obs = robot.get_observation()
        processed = action_processor((action, robot_obs))
        robot.send_action(processed)
        precise_sleep(max(1 / fps - (time.perf_counter() - start_t), 0.0))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--repo-id", default="aakashv100/so101-pick-place-positions")
    p.add_argument("--root", default="hf_data/so101-pick-place-positions")
    p.add_argument("--robot-port", default="COM3")
    p.add_argument("--robot-id", default="my_so_arm")
    p.add_argument("--robot-calib-dir", default="./calibration/robots/so_follower")
    p.add_argument("--seed", type=int, default=None, help="RNG seed for reproducible sampling.")
    p.add_argument("--play-sounds", action="store_true", default=True)
    p.add_argument("--no-play-sounds", action="store_false", dest="play_sounds")
    p.add_argument(
        "--allow-fewer",
        action="store_true",
        help="If requested N > available episodes for a position, replay all of them.",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    register_third_party_plugins()
    init_logging()

    root = Path(args.root).resolve()
    log_path = root / POSITION_LOG_NAME
    info_path = root / "meta" / "info.json"

    try:
        if not info_path.is_file():
            raise ReplayConfigError(
                f"Dataset not found at {root} (missing meta/info.json). "
                "Record demos first with scripts/record_pickplace.ps1."
            )
        log = load_position_log(log_path)
    except ReplayConfigError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 2

    positions = available_positions(log)
    by_pos = {pos: episodes_for_position(log, pos) for pos in positions}
    rng = random.Random(args.seed)

    print("\n=== Pick-and-place random replay ===")
    print(f"dataset : {args.repo_id}")
    print(f"root    : {root}")
    print(f"episodes in log: {len(log)}")
    print(f"positions available: {positions}")
    for pos in positions:
        print(f"  position {pos}: {len(by_pos[pos])} episode(s)")
    print("keys: Right=ready after reset | Left=skip episode | Esc=stop | q at prompt=quit")
    print("")

    robot = None
    listener = None
    start_pose = None
    play_sounds = args.play_sounds

    try:
        robot_cfg = SO101FollowerConfig(
            port=args.robot_port,
            id=args.robot_id,
            calibration_dir=Path(args.robot_calib_dir),
            cameras={},
        )
        robot = make_robot_from_config(robot_cfg)
        action_processor = make_default_robot_action_processor()
        robot.connect()
        start_pose = _capture_start_pose(robot)
        print(
            "Start pose captured (arm returns here after each episode): "
            f"{ {k: round(v, 1) for k, v in start_pose.items()} }"
        )

        listener, events = init_keyboard_listener()
        if listener is None and is_headless():
            print("WARNING: headless — Right arrow unavailable; type Enter instead.")

        last_n: int | None = 5
        last_pos: int | None = positions[0] if positions else None

        while not events.get("stop_recording"):
            print("\n--- Replay block ---")
            n = prompt_int(
                "How many random episodes to replay",
                min_v=1,
                max_v=None,
                default=last_n,
            )
            if n is None:
                print("Quit requested.")
                break

            pos = prompt_int(
                "Sheet position to place the cube",
                min_v=min(positions),
                max_v=max(positions),
                default=last_pos,
            )
            if pos is None:
                print("Quit requested.")
                break

            if pos not in by_pos or not by_pos[pos]:
                print(f"  No episodes recorded for position {pos}. Available: {positions}")
                continue

            candidates = by_pos[pos]
            try:
                chosen = sample_episodes(
                    candidates, n, rng=rng, allow_fewer=args.allow_fewer
                )
            except ReplayConfigError as e:
                print(f"  {e}")
                print(f"  Position {pos} has {len(candidates)} episode(s): {candidates}")
                use_all = input(f"  Replay all {len(candidates)} instead? [y/N]: ").strip().lower()
                if use_all in {"y", "yes"}:
                    chosen = sample_episodes(
                        candidates, len(candidates), rng=rng, allow_fewer=True
                    )
                else:
                    continue

            last_n, last_pos = n, pos
            print(f"\nSelected {len(chosen)} episode(s) at position {pos}: {chosen}")
            log_say(f"Replaying {len(chosen)} episodes at position {pos}", play_sounds)

            for i, episode in enumerate(chosen, start=1):
                if events.get("stop_recording"):
                    break

                print(
                    f"\n>>> [{i}/{len(chosen)}] Episode {episode} "
                    f"(place cube on sheet position {pos})"
                )
                log_say("Reset the environment", play_sounds)
                print(f"  Reset: put the cube on sheet position {pos}, clear the workspace.")

                if listener is None:
                    try:
                        ans = input("  Press Enter when ready (or q to quit): ").strip().lower()
                    except EOFError:
                        return 0
                    if ans in {"q", "quit"}:
                        return 0
                else:
                    status = wait_for_right_arrow(events)
                    if status == "stop":
                        print("Stop requested.")
                        break
                    if status == "skip":
                        print(f"  Skipped episode {episode}.")
                        continue

                try:
                    replay_episode_on_robot(
                        robot,
                        repo_id=args.repo_id,
                        root=root,
                        episode=episode,
                        action_processor=action_processor,
                        play_sounds=play_sounds,
                    )
                except Exception as e:
                    logging.exception("Replay failed for episode %s", episode)
                    print(f"  ERROR replaying episode {episode}: {e}")
                    cont = input("  Continue to next episode? [Y/n]: ").strip().lower()
                    if cont in {"n", "no"}:
                        break
                    continue

                if start_pose is not None:
                    print("  Returning to start pose...")
                    log_say("Returning to start pose", play_sounds)
                    try:
                        _return_to_start_pose(robot, start_pose)
                    except Exception as e:
                        logging.exception("Failed to return to start pose")
                        print(f"  WARNING: could not return to start pose: {e}")

                print(f"  Done episode {episode}.")

            if events.get("stop_recording"):
                break
            print("\nBlock finished. Enter another count/position, or q to quit.")

    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    except Exception as e:
        logging.exception("Fatal replay error")
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    finally:
        try:
            log_say("Exiting", play_sounds, blocking=False)
        except Exception:
            pass
        if robot is not None and getattr(robot, "is_connected", False):
            try:
                robot.disconnect()
            except Exception as e:
                print(f"WARNING: disconnect failed: {e}", file=sys.stderr)
        if listener is not None and not is_headless():
            try:
                listener.stop()
            except Exception:
                pass

    print("Session ended.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
