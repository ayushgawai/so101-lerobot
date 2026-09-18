"""Record pick-and-place demos with interactive position + iteration count.

Before each position block you enter:
  1) sheet pickup position (1-50)
  2) how many iterations (demos) to record for that position

Drop is always the fixed box (no id). Same dataset is appended across sessions.
Same position later = more iterations for that position; new position = new block.

Position is stored in:
  1) the per-frame `task` string inside the LeRobot dataset
  2) sidecar JSONL / summary next to the dataset root

Keyboard while recording / resetting:
  Right arrow  — end current phase (save episode / end reset)
  Left arrow   — discard and re-record the episode
  Escape       — stop the session

Example:
  python scripts/record_pickplace.py
  # or: .\\scripts\\record_pickplace.ps1
"""

from __future__ import annotations

import argparse
import json
import logging
from datetime import datetime, timezone
from pathlib import Path
from pprint import pformat

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.common.control_utils import (
    init_keyboard_listener,
    is_headless,
    sanity_check_dataset_name,
    sanity_check_dataset_robot_compatibility,
)
from lerobot.datasets import (
    LeRobotDataset,
    VideoEncodingManager,
    aggregate_pipeline_dataset_features,
    create_initial_features,
)
from lerobot.processor import make_default_processors
from lerobot.robots import make_robot_from_config
from lerobot.robots.so_follower import SO101FollowerConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.teleoperators.so_leader import SO101LeaderConfig
from lerobot.utils.feature_utils import combine_feature_dicts
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging, log_say
from lerobot.utils.visualization_utils import init_rerun


BASE_TASK = "Pick up the cube and place it in the box"
POSITION_LOG_NAME = "pickup_positions.jsonl"
SUMMARY_NAME = "pickup_positions_summary.json"


def show_camera_preview(robot, title: str = "cameras") -> None:
    """Open/update a side-by-side OpenCV window from the current robot observation."""
    import cv2
    import numpy as np

    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 1280, 480)
    try:
        cv2.setWindowProperty(title, cv2.WND_PROP_TOPMOST, 1)
    except Exception:
        pass

    obs = robot.get_observation()
    frames = [
        cv2.cvtColor(v, cv2.COLOR_RGB2BGR)
        for _, v in obs.items()
        if isinstance(v, np.ndarray) and v.ndim == 3
    ]
    if frames:
        cv2.imshow(title, np.concatenate(frames, axis=1))
    cv2.waitKey(1)


def task_for_position(position: int) -> str:
    return f"Pick up the cube from sheet position {position} and place it in the box"


def prompt_int(message: str, *, min_v: int, max_v: int | None = None, default: int | None = None) -> int | None:
    """Prompt for an int. Returns None if user quits."""
    hint = f" [{default}]" if default is not None else ""
    range_txt = f"{min_v}-{max_v}" if max_v is not None else f">={min_v}"
    while True:
        raw = input(f"{message} ({range_txt}){hint} (q=quit): ").strip()
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


def prompt_position_block(min_pos: int, max_pos: int, default_pos: int | None, default_n: int | None) -> tuple[int, int] | None:
    print("\n--- Next position block ---")
    position = prompt_int("Pickup sheet position", min_v=min_pos, max_v=max_pos, default=default_pos)
    if position is None:
        return None
    n_iters = prompt_int("Iterations for this position", min_v=1, max_v=None, default=default_n)
    if n_iters is None:
        return None
    return position, n_iters


def append_position_log(path: Path, record: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(record) + "\n")


def rewrite_summary(path: Path, records: list[dict], repo_id: str, root: Path) -> None:
    by_pos: dict[str, int] = {}
    for r in records:
        key = str(r["pickup_position"])
        by_pos[key] = by_pos.get(key, 0) + 1
    summary = {
        "repo_id": repo_id,
        "root": str(root),
        "base_task": BASE_TASK,
        "drop_target": "fixed box (no position id)",
        "num_episodes_logged": len(records),
        "counts_by_pickup_position": dict(sorted(by_pos.items(), key=lambda kv: int(kv[0]))),
        "episodes": records,
        "updated_at": datetime.now(timezone.utc).isoformat(),
    }
    path.write_text(json.dumps(summary, indent=2), encoding="utf-8")


def load_existing_log(path: Path) -> list[dict]:
    if not path.exists():
        return []
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            rows.append(json.loads(line))
    return rows


def is_resumable_dataset(root: Path) -> bool:
    """True only if local dataset can be appended (has real episode metadata).

    A failed/empty create leaves meta/info.json with 0 episodes and no
    tasks.parquet. resume() then crashes and incorrectly tries the Hub.
    """
    info_path = root / "meta" / "info.json"
    tasks_path = root / "meta" / "tasks.parquet"
    if not info_path.is_file() or not tasks_path.is_file():
        return False
    try:
        info = json.loads(info_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return False
    return int(info.get("total_episodes", 0)) > 0


def wipe_incomplete_dataset(root: Path) -> None:
    """Remove a non-resumable stub so we can create a fresh local dataset."""
    import shutil

    print(f"Incomplete/empty dataset at {root} — removing stub and starting fresh.")
    shutil.rmtree(root)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--repo-id", default="aakashv100/so101-pick-place-positions")
    p.add_argument("--root", default="hf_data/so101-pick-place-positions",
                   help="Local dataset root. Existing data is appended automatically.")
    p.add_argument("--fresh", action="store_true",
                   help="Refuse to start if --root already has a dataset (safety).")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--episode-time-s", type=float, default=-1,
                   help="Seconds per episode; -1 = until right arrow.")
    p.add_argument("--reset-time-s", type=float, default=-1,
                   help="Seconds for reset between episodes; -1 = until right arrow.")
    p.add_argument("--robot-port", default="COM3")
    p.add_argument("--teleop-port", default="COM4")
    p.add_argument("--robot-id", default="my_so_arm")
    p.add_argument("--teleop-id", default="my_so_arm")
    p.add_argument("--robot-calib-dir", default="./calibration/robots/so_follower")
    p.add_argument("--teleop-calib-dir", default="./calibration/teleoperators/so_leader")
    p.add_argument("--gripper-cam", type=int, default=0)
    p.add_argument("--top-cam", type=int, default=1)
    p.add_argument("--min-position", type=int, default=1)
    p.add_argument("--max-position", type=int, default=50)
    p.add_argument("--push-to-hub", action="store_true")
    p.add_argument("--display-cameras", action="store_true", default=True)
    p.add_argument("--no-display-cameras", action="store_false", dest="display_cameras")
    p.add_argument("--display-data", action="store_true", help="Also open Rerun.")
    p.add_argument("--play-sounds", action="store_true", default=True)
    p.add_argument("--no-play-sounds", action="store_false", dest="play_sounds")
    p.add_argument("--vcodec", default="h264", help="h264 is more reliable on Windows.")
    p.add_argument("--streaming-encoding", action="store_true", default=True)
    p.add_argument("--no-streaming-encoding", action="store_false", dest="streaming_encoding")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    register_third_party_plugins()
    init_logging()

    root = Path(args.root).resolve()
    position_log = root / POSITION_LOG_NAME
    summary_path = root / SUMMARY_NAME

    if args.fresh and is_resumable_dataset(root):
        raise FileExistsError(
            f"Dataset already exists at {root}. Delete it first, or omit --fresh to append."
        )

    if root.exists() and not is_resumable_dataset(root):
        # Empty stub from a crashed/test create: wipe so create() can run.
        if (root / "meta" / "info.json").is_file() or any(root.iterdir()):
            wipe_incomplete_dataset(root)

    existing_log = load_existing_log(position_log)
    resume = is_resumable_dataset(root)

    cameras = {
        "gripper_cam": OpenCVCameraConfig(
            index_or_path=args.gripper_cam,
            fps=args.fps,
            width=640,
            height=480,
            fourcc="MJPG",
        ),
        "top_cam": OpenCVCameraConfig(
            index_or_path=args.top_cam,
            fps=args.fps,
            width=640,
            height=480,
            fourcc="MJPG",
        ),
    }

    robot_cfg = SO101FollowerConfig(
        port=args.robot_port,
        id=args.robot_id,
        calibration_dir=Path(args.robot_calib_dir),
        cameras=cameras,
    )
    teleop_cfg = SO101LeaderConfig(
        port=args.teleop_port,
        id=args.teleop_id,
        calibration_dir=Path(args.teleop_calib_dir),
    )

    logging.info("Robot config:\n%s", pformat(robot_cfg))
    logging.info("Teleop config:\n%s", pformat(teleop_cfg))

    if args.display_data:
        init_rerun(session_name="record_pickplace")

    robot = make_robot_from_config(robot_cfg)
    teleop = make_teleoperator_from_config(teleop_cfg)
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    dataset_features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_action_processor,
            initial_features=create_initial_features(action=robot.action_features),
            use_videos=True,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(observation=robot.observation_features),
            use_videos=True,
        ),
    )

    listener = None
    dataset = None
    records = list(existing_log)

    try:
        if resume:
            dataset = LeRobotDataset.resume(
                args.repo_id,
                root=root,
                batch_encoding_size=1,
                vcodec=args.vcodec,
                streaming_encoding=args.streaming_encoding,
                encoder_threads=2,
                image_writer_processes=0,
                image_writer_threads=4 * len(robot.cameras),
            )
            sanity_check_dataset_robot_compatibility(dataset, robot, args.fps, dataset_features)
            print(f"Appending to existing dataset ({dataset.num_episodes} episodes) at {root}")
        else:
            sanity_check_dataset_name(args.repo_id, policy_cfg=None)
            dataset = LeRobotDataset.create(
                args.repo_id,
                args.fps,
                root=root,
                robot_type=robot.name,
                features=dataset_features,
                use_videos=True,
                image_writer_processes=0,
                image_writer_threads=4 * len(robot.cameras),
                batch_encoding_size=1,
                vcodec=args.vcodec,
                streaming_encoding=args.streaming_encoding,
                encoder_threads=2,
            )
            print(f"Created new dataset at {root}")

        robot.connect()
        teleop.connect()
        listener, events = init_keyboard_listener()

        print("\n=== Pick-and-place recording ===")
        print(f"dataset : {args.repo_id}")
        print(f"root    : {root}")
        print(f"task    : {BASE_TASK}")
        print("drop    : fixed box (no position id)")
        print("flow    : enter position + iterations → record that many → repeat or q")
        print("keys    : Right=end episode/reset | Left=rerecord | Esc=stop")
        print(f"sidecar : {position_log}")

        if args.display_cameras:
            print("\nOpening side-by-side camera window (gripper | top)...")
            show_camera_preview(robot)
            print("Camera popup ready. It keeps updating while you teleop.")
            print("Press Enter here to continue to position prompts...")
            input()
            show_camera_preview(robot)

        with VideoEncodingManager(dataset):
            last_position: int | None = None
            last_n: int | None = 5
            session_saved = 0

            while not events["stop_recording"]:
                block = prompt_position_block(
                    args.min_position, args.max_position, last_position, last_n
                )
                if block is None:
                    print("Quit requested — ending session.")
                    break

                position, n_iters = block
                last_position, last_n = position, n_iters
                episode_task = task_for_position(position)
                already = sum(1 for r in records if r["pickup_position"] == position)

                print(
                    f"\n>>> Position {position}: recording {n_iters} iteration(s) "
                    f"(already have {already} in dataset)"
                )

                saved_in_block = 0
                while saved_in_block < n_iters and not events["stop_recording"]:
                    events["exit_early"] = False
                    episode_index = dataset.num_episodes
                    iter_num = saved_in_block + 1

                    print(
                        f"\n  Episode {episode_index} | position {position} | "
                        f"iteration {iter_num}/{n_iters}"
                    )
                    print("  Teleop the demo, then press Right arrow to save.")
                    log_say(
                        f"Position {position}, iteration {iter_num} of {n_iters}",
                        args.play_sounds,
                    )

                    record_loop(
                        robot=robot,
                        events=events,
                        fps=args.fps,
                        teleop_action_processor=teleop_action_processor,
                        robot_action_processor=robot_action_processor,
                        robot_observation_processor=robot_observation_processor,
                        teleop=teleop,
                        dataset=dataset,
                        control_time_s=args.episode_time_s,
                        single_task=episode_task,
                        display_data=args.display_data,
                        display_cameras=args.display_cameras,
                    )

                    more_in_block = saved_in_block + 1 < n_iters
                    if not events["stop_recording"] and (more_in_block or events["rerecord_episode"]):
                        log_say("Reset the environment", args.play_sounds)
                        print("  Reset: put cube back on the pickup spot, then Right arrow.")
                        record_loop(
                            robot=robot,
                            events=events,
                            fps=args.fps,
                            teleop_action_processor=teleop_action_processor,
                            robot_action_processor=robot_action_processor,
                            robot_observation_processor=robot_observation_processor,
                            teleop=teleop,
                            control_time_s=args.reset_time_s,
                            single_task=episode_task,
                            display_data=args.display_data,
                            display_cameras=args.display_cameras,
                        )

                    if events["rerecord_episode"]:
                        log_say("Re-record episode", args.play_sounds)
                        events["rerecord_episode"] = False
                        events["exit_early"] = False
                        dataset.clear_episode_buffer()
                        print("  Discarded — redo this iteration.")
                        continue

                    if events["stop_recording"]:
                        if dataset.has_pending_frames():
                            dataset.clear_episode_buffer()
                        print("  Stop requested — leaving this block early.")
                        break

                    if not dataset.has_pending_frames():
                        logging.warning("Episode ended with zero frames; not saving.")
                        continue

                    dataset.save_episode(parallel_encoding=False)

                    record = {
                        "episode_index": episode_index,
                        "pickup_position": position,
                        "drop_target": "fixed_box",
                        "task": episode_task,
                        "saved_at": datetime.now(timezone.utc).isoformat(),
                        "block_iteration": iter_num,
                        "block_iterations_planned": n_iters,
                    }
                    append_position_log(position_log, record)
                    records.append(record)
                    rewrite_summary(summary_path, records, args.repo_id, root)

                    saved_in_block += 1
                    session_saved += 1
                    print(
                        f"  Saved. block {saved_in_block}/{n_iters} | "
                        f"dataset total {dataset.num_episodes} | session +{session_saved}"
                    )

                if events["stop_recording"]:
                    break

                print(
                    f"\nFinished position {position} block "
                    f"({saved_in_block}/{n_iters} saved). "
                    "Enter next position, or q to quit."
                )

    finally:
        log_say("Stop recording", args.play_sounds, blocking=True)
        if args.display_cameras:
            try:
                import cv2

                cv2.destroyWindow("cameras")
                cv2.waitKey(1)
            except Exception:
                pass
        if dataset is not None:
            dataset.finalize()
        if robot.is_connected:
            robot.disconnect()
        if teleop.is_connected:
            teleop.disconnect()
        if not is_headless() and listener is not None:
            listener.stop()
        if args.push_to_hub and dataset is not None:
            dataset.push_to_hub()
        log_say("Exiting", args.play_sounds)

    print(f"\nDone. Dataset: {root}")
    print(f"Position log: {position_log}")
    print(f"Summary:     {summary_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted.")
        raise SystemExit(130)
