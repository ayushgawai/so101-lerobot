"""Backfill offline training-set accuracy into an existing W&B training run.

Runs the same MAE / joint-accuracy / frame-accuracy check as
`test_inference_offline.py` on a checkpoint, then writes the numbers into
`run.summary["offline/..."]` so they sit next to the loss curve. This is a
fitting metric on training episodes unless --repo-id/--root point at held-out
data.

Usage (defaults match the SmolVLA run):
    python scripts/log_offline_acc_to_wandb.py
    python scripts/log_offline_acc_to_wandb.py --dry-run
    python scripts/log_offline_acc_to_wandb.py --num-frames 50 --device cuda

    # Generalization number for the holdout run: only the episodes it never trained on
    python scripts/log_offline_acc_to_wandb.py --run-id i4t0f3la --episodes 4 14 24 34 44 \
        --checkpoint outputs/train/smolvla_so101_pick_cube_holdout/checkpoints/last/pretrained_model
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

# Repo root so we can import test_inference_offline without installing it.
REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from test_inference_offline import JOINT_NAMES, compute_offline_metrics  # noqa: E402


def _default_root(repo_id: str) -> str:
    return str(Path.home() / ".cache" / "huggingface" / "lerobot" / repo_id.replace("/", os.sep))


def metrics_to_summary(metrics: dict, checkpoint: str, episodes: list[int] | None = None) -> dict:
    """Flat summary keys under offline/ for the W&B run dashboard.

    `offline/episodes` records which episodes the frames came from, because the same
    keys mean very different things for trained and held-out episodes.
    """
    out = {
        "offline/checkpoint": checkpoint,
        "offline/episodes": "all" if not episodes else ",".join(str(e) for e in episodes),
        "offline/num_frames": metrics["num_frames"],
        "offline/tolerance": metrics["tolerance"],
        "offline/overall_mae": metrics["overall_mae"],
        "offline/joint_accuracy": metrics["joint_accuracy"],
        "offline/frame_accuracy": metrics["frame_accuracy"],
        "offline/latency_mean_ms": metrics["latency_mean_ms"],
        "offline/latency_max_ms": metrics["latency_max_ms"],
    }
    for name in JOINT_NAMES:
        out[f"offline/mae/{name}"] = metrics["joint_mae"][name]
        out[f"offline/accuracy/{name}"] = metrics["joint_accuracy_per_joint"][name]
    return out


def main():
    default_repo = "aakashv100/so101-pick-cube-v2"
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--checkpoint",
        default="outputs/train/smolvla_so101_pick_cube/checkpoints/last/pretrained_model",
    )
    ap.add_argument("--repo-id", default=default_repo)
    ap.add_argument("--root", default=None,
                    help="Dataset root (default: LeRobot cache for --repo-id).")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--num-frames", type=int, default=200)
    ap.add_argument("--tolerance", type=float, default=5.0)
    ap.add_argument("--task", default=None)
    ap.add_argument("--project", default="so101-smolvla")
    ap.add_argument("--run-id", default="o1ngsm6o",
                    help="Training run to resume and attach offline metrics to.")
    ap.add_argument("--entity", default=None)
    ap.add_argument("--rename-map", default=None,
                    help="Optional JSON rename map; default is the checkpoint's.")
    ap.add_argument("--episodes", type=int, nargs="*", default=None,
                    help="Restrict sampling to these episodes. Pass the trainer's "
                         "--dataset.val_episodes to log a generalization number.")
    ap.add_argument("--dry-run", action="store_true",
                    help="Compute and print metrics without touching W&B.")
    args = ap.parse_args()

    root = args.root if args.root is not None else _default_root(args.repo_id)
    rename_map = json.loads(args.rename_map) if args.rename_map else None

    metrics = compute_offline_metrics(
        checkpoint=args.checkpoint,
        repo_id=args.repo_id,
        root=root,
        device=args.device,
        num_frames=args.num_frames,
        task=args.task,
        tolerance=args.tolerance,
        rename_map=rename_map,
        episodes=args.episodes,
        verbose=True,
    )
    summary = metrics_to_summary(metrics, args.checkpoint, args.episodes)

    print("\nW&B summary keys:")
    for k, v in summary.items():
        if isinstance(v, float):
            print(f"  {k}: {v:.4f}" if "accuracy" in k or "mae" in k else f"  {k}: {v:.2f}")
        else:
            print(f"  {k}: {v}")

    if args.dry_run:
        print("\nDry run — nothing sent to W&B.")
        return

    import wandb

    init_kwargs = {"project": args.project, "id": args.run_id, "resume": "must"}
    if args.entity:
        init_kwargs["entity"] = args.entity
    run = wandb.init(**init_kwargs)
    for k, v in summary.items():
        run.summary[k] = v
    run.finish()
    print(f"\nLogged offline accuracy to W&B run {args.run_id} in project {args.project}.")


if __name__ == "__main__":
    main()
