"""Probe why SmolVLA sits still at the start of an eval episode.

The 3-trial sanity run took 288, 300 and 1280 frames to commit to motion from an
identical fixed start pose, where ACT took 65 (median over 100 episodes, max 96).
That spread points at the flow-matching sampler: SmolVLA draws noise per query, so
the same observation can yield a "hold still" chunk or a "go" chunk.

This script tests that directly. For each probe observation it re-queries the
policy --num-samples times and reports how many of the sampled action chunks
contain real motion. A deterministic policy (ACT) gives the same answer every
time; a spread of answers for SmolVLA confirms the sampler is the mechanism.

Motion is measured as the largest deviation of any commanded target in the chunk
from the observed joint state, summed over the 6 joints, in the action's own units.

Usage:
    uv run python scripts/probe_start_hesitation.py \
        --checkpoint outputs/train/smolvla_so101_pick_cube/checkpoints/020000/pretrained_model \
        --device cuda
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lerobot.common.control_utils import prepare_observation_for_inference
from lerobot.configs import PreTrainedConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies import make_policy, make_pre_post_processors
from lerobot.processor import rename_stats

from test_inference_offline import checkpoint_rename_map, frame_to_robot_observation


def episode_frame_index(dataset: LeRobotDataset, episode: int, offset: int) -> int:
    """Global dataset index of frame `offset` within `episode`."""
    row = dataset.meta.episodes[episode]
    start, end = row["dataset_from_index"], row["dataset_to_index"]
    index = start + offset
    if index >= end:
        raise SystemExit(
            f"episode {episode} has only {end - start} frames, cannot take offset {offset}"
        )
    return index


def sample_chunks(policy, preprocessor, postprocessor, obs, task, robot_type, device, n):
    """Return (n, chunk_len, action_dim) commanded targets from n independent queries."""
    chunks = []
    for _ in range(n):
        # Fresh queue per sample so every call is a full denoise from this observation
        # rather than a pop from the chunk the previous call left behind.
        policy.reset()
        batch = prepare_observation_for_inference(dict(obs), device, task, robot_type)
        batch = preprocessor(batch)
        with torch.inference_mode():
            chunk = policy.predict_action_chunk(batch)
        # Postprocess per timestep: the pipeline is written for a single (B, action_dim)
        # action, so feeding it the whole (B, T, action_dim) chunk is not guaranteed safe.
        steps = [postprocessor(chunk[:, t]).cpu().numpy().reshape(-1) for t in range(chunk.shape[1])]
        chunks.append(np.stack(steps))
    return np.stack(chunks)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--checkpoint",
        default="outputs/train/smolvla_so101_pick_cube/checkpoints/020000/pretrained_model",
    )
    parser.add_argument("--train-repo-id", default="aakashv100/so101-pick-cube-v2",
                        help="Dataset the checkpoint was trained on; supplies meta and stats.")
    parser.add_argument("--probe-repo-id", default="aakashv100/eval_sanity-smolvla",
                        help="Dataset to pull the parked observations from.")
    parser.add_argument("--probe-episode", type=int, default=2,
                        help="Episode to probe; 2 is the sanity episode that sat still for 51 s.")
    parser.add_argument("--probe-offsets", type=int, nargs="+", default=[0, 300, 600, 900, 1200],
                        help="Frame offsets within the probe episode. All are frames where the "
                             "arm was parked, so a working policy should command motion.")
    parser.add_argument("--control-offsets", type=int, nargs="+", default=[0, 150],
                        help="Frame offsets in the TRAINING data used as controls: an "
                             "in-distribution 'still waiting' frame and one just after motion "
                             "starts.")
    parser.add_argument("--num-samples", type=int, default=20)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--task", default="Pick up the cube and place it in the bowl")
    parser.add_argument("--motion-threshold", type=float, default=10.0,
                        help="A chunk counts as commanding motion when its peak summed "
                             "deviation from the current joint state exceeds this.")
    args = parser.parse_args()

    device = torch.device(args.device)
    rename_map = checkpoint_rename_map(args.checkpoint)

    train_ds = LeRobotDataset(args.train_repo_id)
    probe_ds = LeRobotDataset(args.probe_repo_id)

    cfg = PreTrainedConfig.from_pretrained(args.checkpoint)
    cfg.pretrained_path = args.checkpoint
    cfg.device = args.device

    policy = make_policy(cfg, ds_meta=train_ds.meta, rename_map=rename_map)
    policy.eval()
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=cfg,
        pretrained_path=cfg.pretrained_path,
        dataset_stats=rename_stats(train_ds.meta.stats, rename_map),
        preprocessor_overrides={
            "device_processor": {"device": args.device},
            "rename_observations_processor": {"rename_map": rename_map},
        },
    )

    n_action_steps = getattr(policy.config, "n_action_steps", None)
    print(f"policy      : {cfg.type}, n_action_steps={n_action_steps}, "
          f"num_steps={getattr(policy.config, 'num_steps', 'n/a')}")
    print(f"samples     : {args.num_samples} independent queries per observation")
    print(f"motion      : peak summed |command - state| over the chunk > {args.motion_threshold:g}\n")

    probes = [("probe", probe_ds, args.probe_episode, o) for o in args.probe_offsets]
    probes += [("control", train_ds, 0, o) for o in args.control_offsets]

    print(f"{'source':8s} {'ep':>3s} {'frame':>6s} {'moves':>9s} {'peak dev: min':>14s} "
          f"{'median':>8s} {'max':>8s}")
    for kind, dataset, episode, offset in probes:
        index = episode_frame_index(dataset, episode, offset)
        frame = dataset[index]
        obs = frame_to_robot_observation(frame)
        state = frame["observation.state"].numpy().astype(np.float32).reshape(-1)

        chunks = sample_chunks(
            policy, preprocessor, postprocessor, obs, args.task,
            dataset.meta.robot_type, device, args.num_samples,
        )
        # Peak commanded deviation from the joint state the policy was shown.
        peak = np.abs(chunks - state).sum(axis=2).max(axis=1)
        moves = peak > args.motion_threshold
        print(f"{kind:8s} {episode:3d} {offset:6d} {moves.sum():4d}/{len(peak):<4d} "
              f"{peak.min():14.1f} {np.median(peak):8.1f} {peak.max():8.1f}")

    print("\nReading this: a 'probe' row is an observation where the arm sat parked and "
          "should have been moving.\nAll samples moving means the sampler is not the "
          "cause and the closed-loop observation path\nis the next suspect. A mixed count "
          "confirms sampling variance: each query is a coin flip,\nand n_action_steps "
          f"= {n_action_steps} makes every unlucky flip cost {n_action_steps} frames.")


if __name__ == "__main__":
    main()
