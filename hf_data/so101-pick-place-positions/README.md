---
license: apache-2.0
task_categories:
- robotics
tags:
- LeRobot
- so101
- so-arm
- pick-and-place
- teleoperation
- imitation-learning
- act
configs:
- config_name: default
  data_files: data/*/*.parquet
---

# SO-101 Pick-and-Place (Multi-Position)

Teleoperated pick-and-place demonstrations on the **SO-ARM 101** (follower + leader), recorded with [LeRobot](https://github.com/huggingface/lerobot) v3.0.

**Task:** pick a cube from a numbered sheet position and place it in a **fixed box**.

<a href="https://huggingface.co/spaces/lerobot/visualize_dataset?path=aakashv100/so101-pick-place-positions">
<img src="https://huggingface.co/datasets/huggingface/badges/resolve/main/visualize-this-dataset-xl.svg"/>
</a>

## Summary

| Field | Value |
|-------|--------|
| Robot | SO-101 follower (`so_follower`) |
| Control | Human teleoperation (SO-101 leader) |
| Episodes | 50 |
| Frames | 60,186 |
| FPS | 30 |
| Cameras | `gripper_cam`, `top_cam` (640×480, H.264) |
| Action / state | 6-DoF joint positions |
| Pickup coverage | Sheet positions **1–10**, **5 demos each** |
| Drop target | Fixed box (same every episode) |
| License | Apache-2.0 |

## Task design

- **Pickup:** cube starts on a marked sheet location (positions `1`–`50` supported by the recording protocol; this release covers `1`–`10`).
- **Place:** cube is dropped into a fixed box (no position id).
- Each episode’s LeRobot `task` string includes the pickup position, e.g.  
  `Pick up the cube from sheet position 6 and place it in the box`.
- Sidecar files map `episode_index → pickup_position`:
  - [`pickup_positions.jsonl`](pickup_positions.jsonl)
  - [`pickup_positions_summary.json`](pickup_positions_summary.json)

## Dataset structure (LeRobot v3.0)

```
meta/info.json          # schema, totals, fps
meta/episodes/          # per-episode metadata
meta/tasks.parquet      # task strings
data/                   # parquet: action, observation.state, indices
videos/                 # per-camera H.264
pickup_positions.jsonl  # episode → sheet position log
```

### Features

| Feature | Shape / notes |
|---------|----------------|
| `action` | float32 `[6]` — shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper |
| `observation.state` | float32 `[6]` — same joint order |
| `observation.images.gripper_cam` | video 480×640×3 |
| `observation.images.top_cam` | video 480×640×3 |
| `timestamp`, `frame_index`, `episode_index`, `index`, `task_index` | standard LeRobot fields |

See [`meta/info.json`](meta/info.json) for the full schema.

## Collection protocol

Recorded on Windows with an interactive LeRobot wrapper:

- Operator enters **sheet position** and **number of iterations**.
- Teleop each demo; **Right arrow** saves; environment reset between demos.
- Sessions **append** to the same dataset (no overwrite).
- Scripts (in the project repo): `scripts/record_pickplace.ps1`, `scripts/replay_pickplace.ps1`.

## Intended use

Imitation learning / behavior cloning (e.g. **ACT**, VLA finetuning) for multi-start-position pick-and-place on SO-101. Position metadata supports stratified splits and position-conditioned analysis.

## Load with LeRobot

```python
from lerobot.datasets import LeRobotDataset

ds = LeRobotDataset("aakashv100/so101-pick-place-positions")
print(ds.num_episodes, ds.num_frames, ds.fps)
```

Local root (if you already have files):

```python
ds = LeRobotDataset(
    "aakashv100/so101-pick-place-positions",
    root="hf_data/so101-pick-place-positions",
)
```

## Citation

```bibtex
@misc{so101_pick_place_positions,
  title        = {SO-101 Pick-and-Place Multi-Position Demonstrations},
  author       = {Madabhushi, Aakash},
  year         = {2026},
  howpublished = {Hugging Face dataset},
  url          = {https://huggingface.co/datasets/aakashv100/so101-pick-place-positions}
}
```

## License

Apache-2.0
