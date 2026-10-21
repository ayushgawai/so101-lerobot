"""Offline inference smoke test for a trained SO-101 policy (ACT, SmolVLA, ...).

Loads a trained checkpoint and runs it on real frames from the local dataset,
comparing predicted actions against ground-truth actions. This validates the
full inference pipeline (preprocess -> policy -> postprocess) WITHOUT needing
the physical arm, and reports per-query latency against the control-loop budget.

Metrics: per-joint MAE plus action accuracy, the share of predicted joint targets
landing within --tolerance of the demonstrated one. Sampling frames the policy
trained on makes this a training accuracy (a fitting check, not a generalization
one); pass --episodes with the episodes held out of training to measure the latter.

The policy type comes from the checkpoint config, so nothing here is ACT-specific.

Usage:
    uv run python test_inference_offline.py
    uv run python test_inference_offline.py \
        --checkpoint outputs/train/smolvla_so101_pick_cube/checkpoints/last/pretrained_model \
        --repo-id aakashv100/so101-pick-cube-v2 --device cuda
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import torch

from lerobot.common.control_utils import predict_action
from lerobot.configs import PreTrainedConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies import make_policy, make_pre_post_processors
from lerobot.processor import rename_stats

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def frame_to_robot_observation(frame: dict) -> dict[str, np.ndarray]:
    """Convert a LeRobotDataset frame into the raw robot-style observation dict
    that `predict_action` expects (images as HWC uint8, state as 1D float32)."""
    obs: dict[str, np.ndarray] = {}
    for key, value in frame.items():
        if not key.startswith("observation."):
            continue
        if "image" in key:
            # dataset stores CHW float in [0,1]; robot path expects HWC uint8
            img = (value.numpy() * 255).clip(0, 255).astype(np.uint8)
            img = np.transpose(img, (1, 2, 0))
            obs[key] = img
        else:
            obs[key] = value.numpy().astype(np.float32)
    return obs


def checkpoint_rename_map(checkpoint: str) -> dict[str, str]:
    """Return the observation rename map baked into a checkpoint's preprocessor.

    SmolVLA checkpoints finetuned from smolvla_base expect the pretrained camera
    slots (observation.images.camera1, ...), so the map recorded at training time
    is the one that must be replayed here.
    """
    config = Path(checkpoint) / "policy_preprocessor.json"
    if not config.is_file():
        return {}
    steps = json.loads(config.read_text()).get("steps", [])
    for step in steps:
        rename_map = step.get("config", {}).get("rename_map")
        if rename_map:
            return rename_map
    return {}


def compute_offline_metrics(
    checkpoint: str,
    repo_id: str,
    root: str,
    *,
    device: str = "cpu",
    num_frames: int = 5,
    task: str | None = None,
    fps: int = 30,
    tolerance: float = 5.0,
    rename_map: dict[str, str] | None = None,
    episodes: list[int] | None = None,
    verbose: bool = True,
) -> dict:
    """Run offline inference and return MAE / accuracy / latency metrics.

    Frames are sampled evenly across the dataset. When those frames are from
    training episodes this is a fitting check, not a generalization measure;
    pass `episodes` to restrict sampling to the episodes held out of training.
    """
    if rename_map is None:
        rename_map = checkpoint_rename_map(checkpoint)
    if rename_map and verbose:
        print(f"Renaming observations: {rename_map}")

    torch_device = torch.device(device)

    if verbose:
        print(f"Loading dataset {repo_id} from {root}")
    dataset = LeRobotDataset(repo_id, root=root, episodes=episodes)

    if verbose:
        print(f"Loading policy config from {checkpoint}")
    cfg = PreTrainedConfig.from_pretrained(checkpoint)
    cfg.pretrained_path = checkpoint
    cfg.device = device

    policy = make_policy(cfg, ds_meta=dataset.meta, rename_map=rename_map)
    policy.eval()

    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=cfg,
        pretrained_path=cfg.pretrained_path,
        dataset_stats=rename_stats(dataset.meta.stats, rename_map),
        preprocessor_overrides={
            "device_processor": {"device": device},
            "rename_observations_processor": {"rename_map": rename_map},
        },
    )

    n = len(dataset)
    idxs = np.linspace(0, n - 1, num_frames).astype(int)
    if verbose:
        scope = f"episodes {episodes}" if episodes else f"all {dataset.meta.total_episodes} episodes"
        print(f"\nDataset has {n} frames across {scope}.")
        print(f"Running inference on frames: {idxs.tolist()}\n")

    errors = []
    latencies = []
    for i in idxs:
        # Policies queue a whole action chunk and pop from it; reset before each
        # frame so every prediction is a fresh open-loop inference from that obs
        # (and so the timing below measures a full forward pass, not a queue pop).
        policy.reset()
        frame = dataset[int(i)]
        obs = frame_to_robot_observation(frame)
        frame_task = frame.get("task", task)
        if isinstance(frame_task, (list, tuple)):
            frame_task = frame_task[0]
        if frame_task in (None, ""):
            raise SystemExit(
                "Dataset frames carry no task string; pass --task '<instruction>'."
            )

        t0 = time.perf_counter()
        action = predict_action(
            observation=obs,
            policy=policy,
            device=torch_device,
            preprocessor=preprocessor,
            postprocessor=postprocessor,
            use_amp=False,
            task=frame_task,
            robot_type=dataset.meta.robot_type,
        )
        latencies.append(time.perf_counter() - t0)
        pred = np.asarray(action.numpy(), dtype=np.float32).reshape(-1)
        gt = np.asarray(frame["action"].numpy(), dtype=np.float32).reshape(-1)
        err = np.abs(pred - gt)
        errors.append(err)

        if verbose:
            print(f"frame {int(i):5d} | task='{frame_task}'")
            print("  joint        pred      gt       |err|")
            for j, name in enumerate(JOINT_NAMES):
                print(f"  {name:12s} {float(pred[j]):8.2f} {float(gt[j]):8.2f} {float(err[j]):8.2f}")
            print()

    errors = np.stack(errors)
    correct = errors <= tolerance
    joint_mae = {name: float(errors[:, j].mean()) for j, name in enumerate(JOINT_NAMES)}
    joint_acc = {name: float(correct[:, j].mean()) for j, name in enumerate(JOINT_NAMES)}
    steady = latencies[1:] or latencies
    metrics = {
        "num_frames": int(len(errors)),
        "tolerance": float(tolerance),
        "overall_mae": float(errors.mean()),
        "joint_accuracy": float(correct.mean()),
        "frame_accuracy": float(correct.all(axis=1).mean()),
        "latency_first_ms": float(latencies[0] * 1000),
        "latency_mean_ms": float(np.mean(steady) * 1000),
        "latency_max_ms": float(max(steady) * 1000),
        "fps": int(fps),
        "joint_mae": joint_mae,
        "joint_accuracy_per_joint": joint_acc,
    }

    if verbose:
        budget_ms = 1000.0 / fps
        print("=" * 50)
        print(f"Action accuracy over {len(errors)} frames "
              f"(correct = within {tolerance:g} of ground truth):")
        print(f"  {'joint':12s} {'MAE':>8} {'accuracy':>9}")
        for name in JOINT_NAMES:
            print(f"  {name:12s} {joint_mae[name]:8.3f} {joint_acc[name]:8.1%}")
        print(f"\nOverall MAE       : {metrics['overall_mae']:.3f}")
        print(f"Joint accuracy    : {metrics['joint_accuracy']:.1%}  "
              f"(per-joint targets within tolerance)")
        print(f"Frame accuracy    : {metrics['frame_accuracy']:.1%}  "
              f"(all 6 joints within tolerance)")
        print(f"\nPolicy query latency: first {metrics['latency_first_ms']:.0f} ms, "
              f"then mean {metrics['latency_mean_ms']:.0f} ms / "
              f"max {metrics['latency_max_ms']:.0f} ms "
              f"(budget {budget_ms:.0f} ms @ {fps} fps)")
        if metrics["latency_max_ms"] > budget_ms:
            print("  WARNING: a query exceeds one control period. Closed-loop recording will "
                  "either stall or drop frames; consider --compile_policy or fewer action steps.")
        print("\nInference pipeline OK: policy produced valid 6-D actions for every frame.")

    return metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--checkpoint",
        default="outputs/train/so101_act_screwdriver/checkpoints/010000/pretrained_model",
    )
    parser.add_argument("--repo-id", default="local/so101-screwdriver-above")
    parser.add_argument("--root", default="./hf_data/so101-pick-place-dataset/view_above")
    parser.add_argument("--device", default="cpu", help="cpu or cuda")
    parser.add_argument("--num-frames", type=int, default=5)
    parser.add_argument("--task", default=None,
                        help="Language instruction, if the dataset frames don't carry one. "
                             "Required by language-conditioned policies such as SmolVLA.")
    parser.add_argument("--fps", type=int, default=30,
                        help="Control-loop rate the latency report is judged against.")
    parser.add_argument("--tolerance", type=float, default=5.0,
                        help="A predicted joint target counts as correct when it lands within "
                             "this much of the demonstrated one, in the action's own units "
                             "(degrees for the 5 arm joints, 0-100 for the gripper).")
    parser.add_argument("--rename-map", default=None,
                        help='JSON map of dataset observation keys to the keys the policy '
                             'expects, e.g. \'{"observation.images.top_cam": '
                             '"observation.images.camera1"}\'. Defaults to the map saved '
                             "in the checkpoint.")
    parser.add_argument("--episodes", type=int, nargs="*", default=None,
                        help="Restrict sampling to these episode indices. Pass the episodes held "
                             "out of training (the trainer's --dataset.val_episodes) to turn this "
                             "into a generalization measure instead of a fitting check.")
    args = parser.parse_args()

    rename_map = (json.loads(args.rename_map) if args.rename_map is not None
                  else None)
    compute_offline_metrics(
        checkpoint=args.checkpoint,
        repo_id=args.repo_id,
        root=args.root,
        device=args.device,
        num_frames=args.num_frames,
        task=args.task,
        fps=args.fps,
        tolerance=args.tolerance,
        rename_map=rename_map,
        episodes=args.episodes,
        verbose=True,
    )


if __name__ == "__main__":
    main()
