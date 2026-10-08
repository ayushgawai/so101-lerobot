"""Record teleop demos one episode at a time: Enter to start, right arrow to save.

For free-placement data (cube anywhere, no position ids), recorded for ACT training and SmolVLA
fine-tuning. Per episode:

  1. Prompt: place the cube, put the leader at the start pose, press Enter (q = quit).
  2. The follower slides to the leader pose over 1 s (not recorded), so the first frames don't jump.
  3. Recording runs with no time limit:
       Right arrow  - save the episode, then prompt for the next one
       Left arrow   - discard it and redo the same episode
       Escape       - stop the session (the in-progress episode is discarded)

No reset phase. Every episode uses the same task string; appending to a dataset recorded with a
different task is refused, so SmolVLA's language conditioning stays consistent.

Dataset layout matches the project's other SO-101 datasets: observation.state / action (6 joints),
observation.images.{gripper_cam,top_cam} 480x640 video, 30 fps, MJPG cameras, h264 encoding.

Example:
  .\\scripts\\run_record.ps1 -NumEpisodes 50
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

# Allow `python scripts/record_episodes.py` to find sibling helpers.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from record_pickplace import is_resumable_dataset, show_camera_preview, wipe_incomplete_dataset

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.common.control_utils import (
    follower_smooth_move_to,
    sanity_check_dataset_name,
    sanity_check_dataset_robot_compatibility,
)
from lerobot.configs.video import RGBEncoderConfig
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
from lerobot.utils.keyboard_input import init_keyboard_listener, is_headless
from lerobot.utils.utils import init_logging, log_say
from lerobot.utils.visualization_utils import init_rerun

DEFAULT_TASK = "Pick up the cube and place it in the box"


def check_task_consistency(dataset, task: str) -> None:
    """Refuse to mix task strings in one dataset (SmolVLA conditions on the task text)."""
    existing = list(dataset.meta.tasks.index) if dataset.meta.tasks is not None else []
    others = [t for t in existing if t != task]
    if others:
        raise ValueError(
            f"Dataset already holds other task string(s) {others}; this session would add {task!r}. "
            "Use the same --task, or record into a separate dataset."
        )


def hand_over_to_leader(robot, teleop, teleop_action_processor, robot_action_processor, fps: int) -> None:
    """Slide the follower to the leader's current pose (not recorded) so episode starts don't jump."""
    obs = robot.get_observation()
    current = {k: v for k, v in obs.items() if k.endswith(".pos")}
    target = robot_action_processor((teleop_action_processor((teleop.get_action(), obs)), obs))
    follower_smooth_move_to(robot, current, target, duration_s=1.0, fps=fps)


def collect_episodes(
    *,
    robot,
    teleop,
    dataset,
    events: dict,
    processors: tuple,
    task: str,
    fps: int,
    num_episodes: int,
    display_data: bool,
    display_cameras: bool,
    play_sounds: bool,
) -> int:
    """Prompt / hand over / record until num_episodes are saved (0 = until q or Esc). Returns saved count."""
    teleop_action_processor, robot_action_processor, robot_observation_processor = processors
    saved = 0
    while not events["stop_recording"] and (num_episodes <= 0 or saved < num_episodes):
        target = f"/{num_episodes}" if num_episodes > 0 else ""
        print(f"\n--- Episode {dataset.num_episodes} (session {saved + 1}{target}) ---")
        answer = input("Place the cube, put the leader at the start pose, then Enter to record (q=quit): ")
        if answer.strip().lower() in {"q", "quit", "exit"}:
            print("Quit requested - ending session.")
            break

        # Arrow keys pressed at the prompt are caught by the listener; they must not end/discard this episode.
        events["exit_early"] = False
        events["rerecord_episode"] = False

        hand_over_to_leader(robot, teleop, teleop_action_processor, robot_action_processor, fps)
        log_say(f"Recording episode {dataset.num_episodes}", play_sounds)
        print("  Recording... Right arrow = save | Left arrow = discard & redo | Esc = stop")
        record_loop(
            robot=robot,
            events=events,
            fps=fps,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
            teleop=teleop,
            dataset=dataset,
            control_time_s=-1,
            single_task=task,
            display_data=display_data,
            display_compressed_images=True,
            display_cameras=display_cameras,
        )

        if events["rerecord_episode"]:
            events["rerecord_episode"] = False
            dataset.clear_episode_buffer()
            log_say("Discarded, redo", play_sounds)
            print("  Discarded - redo this episode.")
            continue

        if events["stop_recording"]:
            if dataset.has_pending_frames():
                dataset.clear_episode_buffer()
            print("  Stop requested - in-progress episode discarded.")
            break

        if not dataset.has_pending_frames():
            logging.warning("Episode ended with zero frames; not saving.")
            continue

        # Serial in-process encoding: the parallel encoder's process pool crashes on Windows.
        dataset.save_episode(parallel_encoding=False)
        saved += 1
        print(f"  Saved. dataset total {dataset.num_episodes} | session +{saved}")
    return saved


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--repo-id", default="aakashv100/so101-pick-place-random")
    p.add_argument("--root", default="hf_data/so101-pick-place-random",
                   help="Local dataset root. Existing data is appended automatically.")
    p.add_argument("--task", default=DEFAULT_TASK, help="Same string for every episode (SmolVLA reads it).")
    p.add_argument("--num-episodes", type=int, default=0, help="Episodes to save this session; 0 = until q/Esc.")
    p.add_argument("--fresh", action="store_true", help="Refuse to start if --root already has a dataset.")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--robot-port", default="COM3")
    p.add_argument("--teleop-port", default="COM4")
    p.add_argument("--robot-id", default="my_so_arm")
    p.add_argument("--teleop-id", default="my_so_arm")
    p.add_argument("--robot-calib-dir", default="./calibration/robots/so_follower")
    p.add_argument("--teleop-calib-dir", default="./calibration/teleoperators/so_leader")
    p.add_argument("--gripper-cam", type=int, default=0)
    p.add_argument("--top-cam", type=int, default=1)
    p.add_argument("--push-to-hub", action="store_true")
    p.add_argument("--display-cameras", action="store_true", default=True)
    p.add_argument("--no-display-cameras", action="store_false", dest="display_cameras")
    p.add_argument("--display-data", action="store_true", help="Also open Rerun.")
    p.add_argument("--teleop-test", action="store_true",
                   help="Free teleop (nothing recorded) to check arm + cameras, then confirm before collecting.")
    p.add_argument("--play-sounds", action="store_true", default=True)
    p.add_argument("--no-play-sounds", action="store_false", dest="play_sounds")
    p.add_argument("--vcodec", default="h264", help="h264 is more reliable on Windows.")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    register_third_party_plugins()
    init_logging()

    root = Path(args.root).resolve()
    if args.fresh and is_resumable_dataset(root):
        raise FileExistsError(f"Dataset already exists at {root}. Delete it first, or omit --fresh to append.")
    if root.exists() and not is_resumable_dataset(root) and any(root.iterdir()):
        wipe_incomplete_dataset(root)
    resume = is_resumable_dataset(root)

    # Same camera setup as record_pickplace / run_eval: names, 640x480, MJPG, one fps everywhere.
    cameras = {
        name: OpenCVCameraConfig(index_or_path=idx, fps=args.fps, width=640, height=480, fourcc="MJPG")
        for name, idx in (("gripper_cam", args.gripper_cam), ("top_cam", args.top_cam))
    }
    robot = make_robot_from_config(
        SO101FollowerConfig(port=args.robot_port, id=args.robot_id, calibration_dir=Path(args.robot_calib_dir),
                            cameras=cameras)
    )
    teleop = make_teleoperator_from_config(
        SO101LeaderConfig(port=args.teleop_port, id=args.teleop_id, calibration_dir=Path(args.teleop_calib_dir))
    )
    processors = make_default_processors()
    teleop_action_processor, _, robot_observation_processor = processors
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
    if args.display_data:
        init_rerun(session_name="record_episodes")

    encoding = dict(
        batch_encoding_size=1,
        rgb_encoder=RGBEncoderConfig(vcodec=args.vcodec),
        streaming_encoding=True,
        encoder_threads=2,
        image_writer_processes=0,
        image_writer_threads=4 * len(cameras),
    )
    dataset = listener = None
    try:
        if resume:
            dataset = LeRobotDataset.resume(args.repo_id, root=root, **encoding)
            sanity_check_dataset_robot_compatibility(dataset, robot, args.fps, dataset_features)
            check_task_consistency(dataset, args.task)
            print(f"Appending to existing dataset ({dataset.num_episodes} episodes) at {root}")
        else:
            sanity_check_dataset_name(args.repo_id, policy_cfg=None)
            dataset = LeRobotDataset.create(
                args.repo_id, args.fps, root=root, robot_type=robot.name, features=dataset_features,
                use_videos=True, **encoding,
            )
            print(f"Created new dataset at {root}")

        teleop.connect()  # leader first, as upstream does, so the follower isn't left idle during teleop init
        robot.connect()
        listener, events = init_keyboard_listener()

        print("\n=== Episode recording (free cube placement) ===")
        print(f"dataset : {args.repo_id}  ({root})")
        print(f"task    : {args.task}")
        print("flow    : Enter = start episode | Right = save | Left = discard & redo | Esc = stop | q = quit")

        if args.display_cameras:
            show_camera_preview(robot)
            input("Check the camera window (gripper | top), then press Enter...")

        if args.teleop_test:
            print("\n=== Teleop test (nothing is recorded) === Right arrow = done | Esc = quit")
            events["exit_early"] = False
            record_loop(
                robot=robot, events=events, fps=args.fps,
                teleop_action_processor=processors[0], robot_action_processor=processors[1],
                robot_observation_processor=processors[2], teleop=teleop, control_time_s=-1,
                display_data=args.display_data, display_compressed_images=True,
                display_cameras=args.display_cameras,
            )
            if not events["stop_recording"] and input("\nStart data collection? [Y/n]: ").strip().lower() in {
                "n", "no", "q", "quit",
            }:
                events["stop_recording"] = True

        with VideoEncodingManager(dataset):
            collect_episodes(
                robot=robot, teleop=teleop, dataset=dataset, events=events, processors=processors,
                task=args.task, fps=args.fps, num_episodes=args.num_episodes,
                display_data=args.display_data, display_cameras=args.display_cameras,
                play_sounds=args.play_sounds,
            )
    finally:
        log_say("Stop recording", args.play_sounds, blocking=True)
        if args.display_cameras:
            try:
                import cv2

                cv2.destroyAllWindows()
                cv2.waitKey(1)
            except Exception:
                pass
        if dataset is not None:
            dataset.finalize()
        # Disconnect each device independently so one failure (e.g. a latched motor fault) can't leave
        # the other connected or skip the Hub push below.
        for device in (robot, teleop):
            if device.is_connected:
                try:
                    device.disconnect()
                except Exception as e:
                    logging.error(f"Failed to disconnect {device}: {e}")
        if listener is not None and not is_headless():
            listener.stop()
        if args.push_to_hub and dataset is not None and dataset.num_episodes > 0:
            dataset.push_to_hub()

    print(f"\nDone. Dataset: {root}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted.")
        raise SystemExit(130)
