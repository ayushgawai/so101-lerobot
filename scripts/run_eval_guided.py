"""Guided real-robot eval: one connect, voice + confirm between protocol trials.

Keeps robot/cameras/policy loaded for the whole session (unlike launching
lerobot_record once per trial). Protocol plan: positions 1-10 x 2 attempts.

Example (usually via scripts/run_eval.ps1):
  python scripts/run_eval_guided.py \\
    --policy.path=outputs/train/.../pretrained_model \\
    --dataset.repo_id=aakashv100/eval_act_so101_pick_place_positions \\
    --dataset.num_episodes=20 --dataset.episode_time_s=-1 \\
    --policy.n_action_steps=25
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path
from pprint import pformat

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.common.control_utils import (
    init_keyboard_listener,
    is_headless,
    sanity_check_dataset_name,
    sanity_check_dataset_robot_compatibility,
)
from lerobot.configs import PreTrainedConfig
from lerobot.datasets import (
    LeRobotDataset,
    VideoEncodingManager,
    aggregate_pipeline_dataset_features,
    create_initial_features,
)
from lerobot.policies import make_policy, make_pre_post_processors
from lerobot.processor import make_default_processors, rename_stats
from lerobot.robots import make_robot_from_config
from lerobot.robots.so_follower import SO101FollowerConfig  # noqa: F401 — alias of SO101FollowerRobotConfig
from lerobot.scripts.lerobot_record import (
    _capture_start_pose,
    _return_to_start_pose,
    _warmup_policy,
    record_loop,
)
from lerobot.utils.device_utils import get_safe_torch_device
from lerobot.utils.feature_utils import combine_feature_dicts
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging, log_say


def protocol_trial_plan() -> list[dict]:
    plan = []
    trial = 0
    for pos in range(1, 11):
        for attempt in (1, 2):
            trial += 1
            plan.append({"trial": trial, "init_pos": pos, "attempt": attempt})
    return plan


def prompt_action(prompt: str, choices: dict[str, str], default: str) -> str:
    """choices maps normalized input -> action name. Empty input uses default."""
    hint = " / ".join(f"[{k.upper()}]{v}" for k, v in choices.items() if k)
    while True:
        raw = input(f"{prompt} {hint}: ").strip().lower()
        if raw == "":
            return default
        if raw in choices:
            return choices[raw]
        # allow full words
        for key, action in choices.items():
            if raw == action or raw.startswith(key):
                return action
        print(f"  Please enter one of: {', '.join(sorted(set(choices.values())))}")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--policy.path", dest="policy_path", required=True)
    p.add_argument("--policy.device", dest="device", default="cuda")
    p.add_argument("--policy.use_amp", dest="use_amp", default="true")
    p.add_argument("--policy.n_action_steps", dest="n_action_steps", type=int, default=0)
    p.add_argument("--policy.num_steps", dest="denoise_steps", type=int, default=0)
    p.add_argument("--dataset.repo_id", dest="repo_id", required=True)
    p.add_argument("--dataset.root", dest="root", default="")
    p.add_argument("--dataset.num_episodes", dest="num_episodes", type=int, default=20)
    p.add_argument("--dataset.single_task", dest="task", default="Pick up the cube and place it in the bowl")
    p.add_argument("--dataset.episode_time_s", dest="episode_time_s", type=float, default=-1)
    p.add_argument("--dataset.rename_map", dest="rename_map", default="")
    p.add_argument("--resume", action="store_true")
    p.add_argument("--robot.port", dest="port", default="COM3")
    p.add_argument("--robot.id", dest="robot_id", default="my_so_arm")
    p.add_argument("--robot.calibration_dir", dest="calib_dir", default="./calibration/robots/so_follower")
    p.add_argument("--gripper-cam", type=int, default=0)
    p.add_argument("--top-cam", type=int, default=1)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--display_cameras", dest="display_cameras", default="true")
    p.add_argument("--return_to_start_pose", dest="return_to_start", default="true")
    p.add_argument("--compile_policy", action="store_true")
    p.add_argument("--play-sounds", dest="play_sounds", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--start-trial", type=int, default=0, help="1-based trial index; 0 = auto from dataset")
    p.add_argument("--skip-count", type=int, default=-1, help="Override how many plan trials to skip")
    return p.parse_args()


def _as_bool(v: str | bool) -> bool:
    if isinstance(v, bool):
        return v
    return str(v).strip().lower() in {"1", "true", "yes", "y"}


def _parse_rename_map(raw: str) -> dict[str, str]:
    raw = (raw or "").strip()
    if not raw:
        return {}
    # "{a: b, c: d}" from PowerShell
    inner = raw.strip("{} ")
    out: dict[str, str] = {}
    for part in inner.split(","):
        part = part.strip()
        if not part or ":" not in part:
            continue
        k, v = part.split(":", 1)
        out[k.strip()] = v.strip()
    return out


def main() -> int:
    register_third_party_plugins()
    init_logging()
    args = parse_args()

    display_cameras = _as_bool(args.display_cameras)
    return_to_start = _as_bool(args.return_to_start)
    use_amp = _as_bool(args.use_amp)
    rename_map = _parse_rename_map(args.rename_map)
    root = Path(args.root) if args.root else None

    policy_cfg = PreTrainedConfig.from_pretrained(args.policy_path)
    policy_cfg.pretrained_path = args.policy_path
    policy_cfg.device = args.device
    policy_cfg.use_amp = use_amp
    if args.n_action_steps > 0 and hasattr(policy_cfg, "n_action_steps"):
        policy_cfg.n_action_steps = args.n_action_steps
        logging.info("Using n_action_steps=%s", args.n_action_steps)
    if args.denoise_steps > 0 and hasattr(policy_cfg, "num_steps"):
        policy_cfg.num_steps = args.denoise_steps

    cameras = {
        "gripper_cam": OpenCVCameraConfig(
            index_or_path=args.gripper_cam, width=640, height=480, fps=args.fps, fourcc="MJPG"
        ),
        "top_cam": OpenCVCameraConfig(
            index_or_path=args.top_cam, width=640, height=480, fps=args.fps, fourcc="MJPG"
        ),
    }
    robot_cfg = SO101FollowerConfig(
        port=args.port,
        id=args.robot_id,
        calibration_dir=Path(args.calib_dir),
        cameras=cameras,
    )
    logging.info("Robot config:\n%s", pformat(robot_cfg))

    robot = make_robot_from_config(robot_cfg)
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

    plan = protocol_trial_plan()
    total_plan = len(plan)
    num_episodes = min(args.num_episodes, total_plan)

    dataset = None
    listener = None
    saved = 0

    try:
        if args.resume:
            if root is None:
                raise SystemExit("--resume requires --dataset.root")
            dataset = LeRobotDataset.resume(
                args.repo_id,
                root=root,
                batch_encoding_size=1,
                image_writer_processes=0,
                image_writer_threads=4 * len(robot.cameras),
            )
            sanity_check_dataset_robot_compatibility(dataset, robot, args.fps, dataset_features)
            print(f"Resuming dataset ({dataset.num_episodes} episodes) at {root}")
        else:
            sanity_check_dataset_name(args.repo_id, policy_cfg)
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
            )
            print(f"Created dataset at {dataset.root}")

        if args.skip_count >= 0:
            skip = args.skip_count
        elif args.start_trial > 0:
            skip = args.start_trial - 1
        else:
            skip = dataset.num_episodes
        remaining = plan[skip : skip + num_episodes]
        if not remaining:
            raise SystemExit(f"No trials left (skip={skip}, plan={total_plan}, num_episodes={num_episodes})")

        policy = make_policy(policy_cfg, ds_meta=dataset.meta, rename_map=rename_map)
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=policy_cfg,
            pretrained_path=args.policy_path,
            dataset_stats=rename_stats(dataset.meta.stats, rename_map),
            preprocessor_overrides={
                "device_processor": {"device": args.device},
                "rename_observations_processor": {"rename_map": rename_map},
            },
        )
        if args.compile_policy:
            import sys
            import torch

            if sys.platform == "win32":
                policy.model = torch.compile(policy.model, backend="cudagraphs")
            else:
                policy.model = torch.compile(policy.model, mode="reduce-overhead")

        _warmup_policy(
            policy=policy,
            preprocessor=preprocessor,
            postprocessor=postprocessor,
            device=get_safe_torch_device(args.device),
            task=args.task,
            n_passes=5 if args.compile_policy else 2,
        )

        print("Connecting robot + cameras once for the whole session...")
        robot.connect()
        start_pose = _capture_start_pose(robot) if return_to_start else None
        if start_pose:
            logging.info("Start pose captured: %s", {k: round(v, 1) for k, v in start_pose.items()})

        listener, events = init_keyboard_listener()

        log_say(
            f"Starting guided evaluation. {len(remaining)} trials. Cube on taped positions. Bowl stays fixed.",
            args.play_sounds,
            blocking=True,
        )

        with VideoEncodingManager(dataset):
            for item in remaining:
                if events.get("stop_recording"):
                    print("Stop requested (Esc). Ending session.")
                    break

                trial = item["trial"]
                pos = item["init_pos"]
                attempt = item["attempt"]
                print(
                    f"\n--------------------------------------------------\n"
                    f"Trial {trial} of {total_plan}\n"
                    f"  init_pos_id : {pos}\n"
                    f"  attempt     : {attempt} of 2\n"
                    f"  setup       : put CUBE on taped position {pos}; BOWL stays fixed\n"
                    f"  controls    : Right arrow = end & save | Left = re-record | Esc = stop\n"
                    f"--------------------------------------------------"
                )
                log_say(
                    f"Trial {trial} of {total_plan}. Place the cube at position {pos}. "
                    f"Attempt {attempt}. Keep the bowl fixed at its mark.",
                    args.play_sounds,
                    blocking=True,
                )
                log_say("Ready to start?", args.play_sounds, blocking=True)

                action = prompt_action(
                    f"Start Trial {trial} of {total_plan}?",
                    {"y": "start", "yes": "start", "s": "skip", "skip": "skip", "q": "quit", "quit": "quit"},
                    default="start",
                )
                if action == "quit":
                    log_say("Stopping evaluation.", args.play_sounds, blocking=True)
                    break
                if action == "skip":
                    log_say(f"Skipping trial {trial}.", args.play_sounds)
                    print(f"Skipped trial {trial} (not recorded).")
                    continue

                while True:
                    events["exit_early"] = False
                    events["rerecord_episode"] = False
                    log_say(f"Recording trial {trial}", args.play_sounds)
                    record_loop(
                        robot=robot,
                        events=events,
                        fps=args.fps,
                        teleop_action_processor=teleop_action_processor,
                        robot_action_processor=robot_action_processor,
                        robot_observation_processor=robot_observation_processor,
                        policy=policy,
                        preprocessor=preprocessor,
                        postprocessor=postprocessor,
                        dataset=dataset,
                        control_time_s=args.episode_time_s,
                        single_task=args.task,
                        display_cameras=display_cameras,
                    )

                    if events.get("stop_recording"):
                        break

                    if start_pose is not None:
                        log_say("Returning to start pose", args.play_sounds)
                        _return_to_start_pose(robot, start_pose)

                    if events.get("rerecord_episode"):
                        log_say("Re-record episode", args.play_sounds)
                        dataset.clear_episode_buffer()
                        events["rerecord_episode"] = False
                        events["exit_early"] = False
                        continue

                    if not dataset.has_pending_frames():
                        logging.warning("No frames captured; not saving.")
                        retry = prompt_action(
                            "No frames saved. Retry?",
                            {"r": "retry", "retry": "retry", "y": "retry", "s": "skip", "q": "quit"},
                            default="retry",
                        )
                        if retry == "quit":
                            events["stop_recording"] = True
                        if retry != "retry":
                            break
                        continue

                    dataset.save_episode(parallel_encoding=False)
                    saved += 1
                    log_say(f"Trial {trial} saved.", args.play_sounds)
                    print(f"Saved. Dataset episodes now: {dataset.num_episodes}")
                    break

                if events.get("stop_recording"):
                    break

        log_say(f"Evaluation session finished. Saved {saved} episodes this session.", args.play_sounds, blocking=True)
        print(f"Done. Saved this session: {saved} | dataset episodes: {dataset.num_episodes}")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    finally:
        log_say("Stop recording", args.play_sounds, blocking=False)
        if display_cameras:
            try:
                import cv2

                cv2.destroyWindow("cameras")
                cv2.waitKey(1)
            except Exception:
                pass
        if dataset is not None:
            try:
                dataset.finalize()
            except Exception as e:
                logging.warning("dataset.finalize failed: %s", e)
        if robot.is_connected:
            robot.disconnect()
        if not is_headless() and listener is not None:
            listener.stop()
        log_say("Exiting", args.play_sounds, blocking=False)


if __name__ == "__main__":
    sys.exit(main())
