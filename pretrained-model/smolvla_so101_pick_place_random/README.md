---
library_name: lerobot
license: apache-2.0
base_model: lerobot/smolvla_base
datasets:
- aakashv100/so101-pick-place-random-trimmed
pipeline_tag: robotics
tags:
- robotics
- smolvla
- so101
---

# SmolVLA SO-101 pick-and-place, random cube positions

Finetuned from [lerobot/smolvla_base](https://huggingface.co/lerobot/smolvla_base) on [aakashv100/so101-pick-place-random-trimmed](https://huggingface.co/datasets/aakashv100/so101-pick-place-random-trimmed) (idle-trimmed copy of [aakashv100/so101-pick-place-random](https://huggingface.co/datasets/aakashv100/so101-pick-place-random)).

- Task string: "Pick up the cube and place it in the box" (pass it exactly at inference)
- Job: smolvla_pick_place_random_places
- Steps: 20,000 | batch 16 | seed 1000 | checkpoint 020000
- Train episodes: 45 | val episodes: 47, 45, 37, 42, 39 | val loss 0.097 at 20k (min 0.090 at 6k-10k)
- Cameras: top_cam -> camera1, gripper_cam -> camera2 (pass the same rename map at inference)
- W&B: [jjn8scoc](https://wandb.ai/aakashvardhan-madabhushi-san-jose-state-university/so101-smolvla/runs/jjn8scoc)
- Sanity eval dataset: [aakashv100/eval_sanity-smolvla-random-places](https://huggingface.co/datasets/aakashv100/eval_sanity-smolvla-random-places)

Load with LeRobot via --policy.path=aakashv100/smolvla_so101_pick_place_random.
