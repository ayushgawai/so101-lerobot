# SmolVLA Finetuning on SO-ARM 101 (50 demonstrations)

**Project:** SO-101 pick-cube imitation learning
**Date:** July 2026
**Framework:** [LeRobot](https://github.com/huggingface/lerobot) v3.0
**Policy:** SmolVLA (vision-language-action, flow matching), finetuned from [`lerobot/smolvla_base`](https://huggingface.co/lerobot/smolvla_base)
**W&B run:** [smolvla_so101_pick_cube](https://wandb.ai/aakashvardhan-madabhushi-san-jose-state-university/so101-smolvla/runs/o1ngsm6o)

---

## Executive summary

A SmolVLA policy was **finetuned** from the pretrained `lerobot/smolvla_base` checkpoint on the same **50 teleoperated demonstrations** used for ACT v2 (*pick up the cube and place it in the bowl*). Only the action expert and the state/action projections are trained (99.9M of 450M parameters); the SmolVLM2-500M backbone and its vision encoder stay frozen. The run is the SmolVLA arm of a head-to-head comparison against `act_so101_pick_cube_v2` under one shared protocol: the same dataset, the same 100-trial scoresheet layout, and the same 50 random cube positions.

| Metric | SmolVLA | ACT v2 (reference) |
|--------|---------|--------------------|
| Initialization | Finetune of `smolvla_base` | From scratch (ImageNet ResNet18) |
| Demonstrations | 50 episodes / 59,998 frames | same |
| Training steps | 20,000 | 60,000 |
| Batch size | 16 | 8 |
| Effective epochs | 5.3 | 8.0 |
| Total parameters | 450.0M | 51.7M |
| Trainable parameters | 99.9M | 51.7M |
| Action chunk | 50 (~1.7 s @ 30 fps) | 100 (~3.3 s) |
| Peak VRAM | 5.1 GB | ~7.7 GB |
| Throughput | 1.03 steps/s | ~1.3 steps/s |
| Wall-clock | 5.7 h | ~13 h (est. from v1) |
| Final train loss | 0.039 | *see W&B `55h5xmgg`* |
| Training-set MAE (200 frames) | **1.06** | 1.49 |
| Training-set joint accuracy (±5) | **97.8%** | 94.8% |
| Training-set frame accuracy (±5) | **89.0%** | 72.0% |
| Inference latency / query | 251 ms | 18 ms |
| Fixed-position success | *pending* | 92% (46/50) |
| Random-position success | *pending* | *unscored* |

**Offline result:** SmolVLA fits the demonstrations better than ACT v2 despite one third the gradient steps and two thirds the epochs — MAE 1.06 vs 1.49, and 89.0% vs 72.0% of frames with all six joint targets inside ±5. It pays for that with a 14x slower query (251 ms vs 18 ms), which is the main risk to closed-loop behaviour.

**Status:** training and offline evaluation are automated and verified. Two real-robot attempts on 2026-07-28 were **voided**. A 3-trial sanity run showed the policy stalling 11-51 s before initiating motion, traced to objects in the top-camera view that never appear in the demonstrations (see [sanity-run findings](#sanity-run-findings-2026-07-28)). A subsequent 50-trial Fixed run was launched before the scene was cleared and died at episode 17 when `gripper_cam` dropped out (see [first full-run attempt](#first-full-run-attempt-2026-07-28-voided)). The 100 scored trials need a human at the arm, a cleared scene and the USB fix verified; they are handed over in [section 8](#8-real-robot-evaluation-handover).

---

## 1. Task and robot setup

Identical to the ACT v2 run, so the comparison isolates the policy.

- **Instruction:** `Pick up the cube and place it in the bowl` (SmolVLA is language-conditioned, so this string is a real input, not just metadata)
- **Robot:** SO-101 follower (`so_follower`) on `COM3`, calibration `calibration/robots/so_follower/my_so_arm.json`
- **Cameras:** `gripper_cam` (OpenCV index 0), `top_cam` (wide-angle USB, index 1), both 640x480 @ 30 fps, MJPG
- **Action/state space:** 6-D joint position targets (`shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`)

---

## 2. Dataset

| Property | Value |
|----------|-------|
| Hub | [aakashv100/so101-pick-cube-v2](https://huggingface.co/datasets/aakashv100/so101-pick-cube-v2) |
| Local root | `%USERPROFILE%\.cache\huggingface\lerobot\aakashv100\so101-pick-cube-v2` |
| Episodes | 50 |
| Frames | 59,998 (~1,200 frames/episode) |
| FPS | 30 |
| Train split | All 50 episodes — **no held-out validation split** |

Every demonstration starts the cube on the "A" paper marker (spread < 1 cm across all 50 first frames), so the training distribution contains **zero cube-position randomization**. This caps what any policy trained on it can do on the Random tab, and it applies equally to both policies.

---

## 3. Model architecture

**Policy type:** SmolVLA (VLM prefix + flow-matching action expert)

| Hyperparameter | Value | Notes |
|----------------|-------|-------|
| `vlm_model_name` | `HuggingFaceTB/SmolVLM2-500M-Video-Instruct` | frozen |
| `num_vlm_layers` | 16 | truncated from the full backbone |
| `attention_mode` | `cross_attn` | expert attends to the VLM prefix |
| `expert_width_multiplier` | 0.75 | action-expert width vs VLM |
| `self_attn_every_n_layers` | 2 | |
| `chunk_size` | 50 | predicts 50 future actions (~1.7 s @ 30 fps) |
| `n_action_steps` | 50 | executes the full chunk before re-querying |
| `num_steps` | 10 | flow-matching denoising steps per query (inference only) |
| `n_obs_steps` | 1 | single-frame observation |
| `resize_imgs_with_padding` | 512 x 512 | letterboxed, then scaled to [-1, 1] |
| `tokenizer_max_length` | 48 | |
| `freeze_vision_encoder` | true | |
| `train_expert_only` | true | |
| `train_state_proj` | true | |
| `max_state_dim` / `max_action_dim` | 32 / 32 | our 6-D vectors are zero-padded |

**Normalization:** IDENTITY for VISUAL, MEAN_STD for STATE and ACTION.
**Parameters:** 450,046,176 total, **99,880,992 trainable**.

### Camera key mapping

`smolvla_base` declares its inputs as `observation.images.camera{1,2,3}`, while this robot records `top_cam` and `gripper_cam`. The wrapper therefore trains with

```
--rename_map={observation.images.top_cam: observation.images.camera1,
              observation.images.gripper_cam: observation.images.camera2}
```

`camera3` simply stays absent (with `empty_cameras=0` a declared-but-missing camera is dropped). The map is baked into the checkpoint's saved preprocessor, but `lerobot_record` *overrides* that step from `--dataset.rename_map`, so the same map must be passed at eval time — `run_eval.ps1 -Policy smolvla` does this automatically.

---

## 4. Training configuration

### Command

```powershell
.\scripts\train_smolvla.ps1 -Wandb -WandbProject so101-smolvla
```

### Optimizer and schedule (SmolVLA training preset)

| Setting | Value |
|---------|-------|
| Optimizer | AdamW |
| Learning rate | 1x10^-4 (peak) |
| Betas / eps | (0.9, 0.95) / 1x10^-8 |
| Weight decay | 1x10^-10 |
| Gradient clip | 10.0 |
| LR scheduler | cosine decay with warmup |
| Warmup steps | 1,000 |
| Decay steps | 20,000 (matched to run length by the wrapper) |
| Final LR | 2.5x10^-6 |
| Batch size | 16 |
| Steps | 20,000 |
| Seed | 1000 |

### Infrastructure

| Setting | Value |
|---------|-------|
| GPU | NVIDIA RTX 5080 Laptop, 16 GB |
| Device | CUDA (PyTorch 2.10+cu130) |
| VRAM used | 5.1 GB of 16.3 GB |
| GPU utilization | ~75% |
| `num_workers` | 0 (required on Windows) |
| Video backend | pyav (`torchcodec` unavailable on Windows) |
| Data augmentation | Off |
| `return_uint8` | Off (unverified against SmolVLA's IDENTITY visual normalization) |

Checkpoints every 2,000 steps into `outputs/train/smolvla_so101_pick_cube/checkpoints`, ~1.2 GB each.

---

## 5. Training results

### Loss curve

| Step | Train loss | Grad norm | LR |
|------|-----------|-----------|-----|
| 200 | 0.304 | 4.14 | 1.0x10^-5 (warmup) |
| 1,000 | 0.166 | 3.07 | 9.0x10^-5 |
| 2,000 | 0.126 | 2.26 | 9.8x10^-5 |
| 5,000 | 0.093 | 1.70 | 8.6x10^-5 |
| 10,000 | 0.057 | 1.17 | 5.2x10^-5 |
| 15,000 | 0.041 | 0.90 | 1.7x10^-5 |
| **20,000** | **0.039** | **0.81** | 2.5x10^-6 |

**Observations:**

- Smooth monotone decrease with no instability; gradient norm fell from 4.1 to 0.8, well under the clip threshold of 10 throughout.
- The loss flattened over the last ~3,000 steps as the cosine schedule drove the LR to 2.5x10^-6, so the run ended converged rather than truncated. Extra steps would need the decay horizon extended too (`-Steps` sets `-DecaySteps` automatically).
- 320K samples seen = 5.33 epochs over the 50 demonstrations.

### Duration and throughput

- **Wall-clock:** 5.7 h (14:06 to 19:48)
- **Throughput:** 1.03 steps/s at batch 16 (`updt_s` ~0.51, `data_s` ~0.49), so the run is about half data-bound on CPU-side video decode — the same bottleneck as ACT, and the reason `num_workers=0` on Windows hurts.
- **VRAM:** 5.1 GB of 16.3 GB, GPU utilization ~75%. Batch could still be raised.

### Simulation evaluation during training

**Not performed.** LeRobot's `eval_freq` rollouts need a simulation environment; this is offline imitation only (`env: null`). Task success is measured on the physical robot.

---

## 6. Offline evaluation

### Method

```powershell
.\.venv\Scripts\python.exe test_inference_offline.py `
  --checkpoint outputs/train/smolvla_so101_pick_cube/checkpoints/last/pretrained_model `
  --repo-id aakashv100/so101-pick-cube-v2 `
  --root "$env:USERPROFILE\.cache\huggingface\lerobot\aakashv100\so101-pick-cube-v2" `
  --device cuda --num-frames 200
```

The script runs the full inference pipeline (rename -> tokenize -> normalize -> SmolVLA -> unnormalize) on frames sampled evenly across the dataset and reports three things:

- **MAE** per joint, in the action's own units.
- **Action accuracy** — the share of predicted joint targets within `--tolerance` (default 5.0) of the demonstrated one, reported both per-joint-target (*joint accuracy*) and as the stricter share of frames where all 6 joints land inside tolerance (*frame accuracy*).
- **Query latency** against the 33 ms control period.

Frames come from the episodes the policy trained on, so this is a **training accuracy**: it measures how well the policy fitted the demonstrations, not how it generalizes. Point `--repo-id`/`--root` at held-out episodes for the latter.

### Results

Both policies were measured with the same script, the same 200 frames and the same
tolerance, so the columns are directly comparable.

| Joint | SmolVLA MAE | SmolVLA acc. | ACT v2 MAE | ACT v2 acc. |
|-------|-------------|--------------|------------|-------------|
| shoulder_pan | 0.64 | 100.0% | 1.04 | 96.5% |
| shoulder_lift | 1.46 | 99.0% | 2.29 | 91.0% |
| elbow_flex | 1.48 | 97.0% | 2.08 | 92.0% |
| wrist_flex | 1.02 | 97.5% | 1.35 | 97.5% |
| wrist_roll | 0.53 | 99.5% | 0.77 | 99.5% |
| gripper | 1.24 | 94.0% | 1.41 | 92.5% |
| **Overall MAE** | **1.06** | | 1.49 | |
| **Joint accuracy** | | **97.8%** | | 94.8% |
| **Frame accuracy** | | **89.0%** | | 72.0% |

**Interpretation:**

- SmolVLA is better on every joint, and the gap is widest exactly where ACT v1 and v2 were weakest: `shoulder_lift` and `elbow_flex`, the reach-phase joints (MAE 1.46/1.48 vs 2.29/2.08).
- The frame-accuracy gap (89.0% vs 72.0%) is larger than the per-joint gap because ACT's errors are less correlated across joints, so more of its frames have at least one joint outside tolerance.
- `gripper` is the weakest joint for both. Gripper timing is what decides a grasp, so a 94% / 92.5% split here is likely to matter more on the robot than the arm-joint gap does.
- This is a **fitting** result on training episodes, not evidence of generalization. It says the pretrained VLM prior plus 100M trainable expert parameters reproduces these demonstrations more faithfully than ACT trained from scratch; it says nothing yet about novel cube positions. The Random tab is the test for that.

For reference, the 200-step smoke checkpoint scored MAE 3.50 / joint accuracy 85.8% / frame accuracy 40.0% — a floor, not a result, since the LR was still in warmup.

### Inference latency

The final checkpoint measured **251 ms mean / 313 ms max** per query on an idle GPU at the
default 10 denoising steps. Latency is a fixed ~53 ms of VLM prefix and preprocessing plus
~20 ms per denoising step, so it scales down predictably:

| `num_steps` | Mean latency | Max |
|-------------|--------------|-----|
| 10 (default) | 252 ms | 260 ms |
| 5 | 143 ms | 144 ms |
| 4 | 127 ms | 132 ms |
| 2 | 99 ms | 102 ms |
| 1 | 74 ms | 75 ms |

ACT v2 measured **18 ms** per query on the same machine — 14x faster, inside the 33 ms
budget, and it re-queries only every 100 frames.

**Implication for closed-loop control.** No SmolVLA setting fits inside one 33 ms control period, but a query only happens every `n_action_steps` = 50 frames, so the arm holds its last commanded target at each chunk boundary rather than every step. Measured on the robot (see [section 8](#sanity-run-findings-2026-07-28)): the stall is ~350 ms once per chunk, the loop sustains 25 Hz against the 30 Hz target, and that is the only overhead in the loop. If the pause breaks grasps, lower it with `run_eval.ps1 -Policy smolvla -DenoiseSteps 5`, which trades action fidelity for latency and needs no retraining.

---

## 7. Evaluation protocol

Identical to ACT's, scored in `SmolVLA_eval_scoresheet.xlsx` (generated by `scripts/make_eval_scoresheet.py`, same layout as `ACT_eval_scoresheet.xlsx`):

- **Fixed tab:** 50 trials, cube on the A marker every time.
- **Random tab:** 50 trials, cube on dot N of the printed `eval_position_grid.html` — **the same 50 positions as ACT**, copied from `ACT_eval_scoresheet.xlsx` `Random!B15:B64`.
- One checkpoint locked for both tabs. No manual assist (touching the cube or arm = fail). Retries within an episode are allowed; success = autonomous grasp + release in the bowl.
- Failure modes: `no-grasp` / `grasped-dropped` / `wrong-placement` / `other`.
- Right arrow ends and saves an episode, left arrow discards and re-records, Esc stops the run.
- Episode index + 1 = sheet trial number.

---

## 8. Real-robot evaluation (handover)

These steps need a human at the arm. Run them after training finishes.

**1. Confirm the arm reads in-distribution** (stats are dataset-derived, shared with ACT v2):

```powershell
.\.venv\Scripts\python.exe scripts\verify_calib.py
.\.venv\Scripts\python.exe scripts\goto_start_pose.py
```

**2. Point the eval preset at the final checkpoint.** `run_eval.ps1` defaults to `checkpoints/020000`; pass `-Checkpoint` if you evaluate a different one.

**3. Clear the scene first.** Nothing on the table but the cube, the bowl and (Random only) the
grid sheet; hands out of the top camera's view for the whole rollout. This is not
housekeeping — it is the difference between the policy working and stalling, as the sanity-run
findings below show.

**4. Sanity run (3 trials, throwaway dataset)** with the cube near-A / mid / far-corner:

```powershell
.\scripts\run_eval.ps1 -Policy smolvla -Mode fixed -NumEpisodes 3 `
  -RepoId aakashv100/eval_sanity-smolvla -ClearCache
```

Watch for: whether the arm commits to motion within ~2-3 s, the ~350 ms pause at each chunk
boundary, and whether the gripper closes at the right moment. If the pause disrupts grasps, add
`-DenoiseSteps 5`. If the arm hesitates for more than a few seconds, stop and check the scene
with

```powershell
.\.venv\Scripts\python.exe scripts\probe_start_hesitation.py --device cuda `
  --probe-repo-id aakashv100/eval_sanity-smolvla --probe-episode 0 --probe-offsets 0 15 30 45 60
```

20/20 with median peak deviation above ~20 means the scene is in distribution; a mixed count
means it is not, and no amount of retrying on the robot will fix it.

**5. Full runs (50 trials each):**

```powershell
.\scripts\run_eval.ps1 -Policy smolvla -Mode fixed  -ClearCache
.\scripts\run_eval.ps1 -Policy smolvla -Mode random -ClearCache
```

If motor 5 latches: power-cycle 15 s, re-run `goto_start_pose.py`, then resume with
`-Resume -NumEpisodes <remaining>`.

**6. Measure the placement/gripper columns** from the recorded `top_cam` frames:

```powershell
.\.venv\Scripts\python.exe scripts\measure_placement_error.py `
  --repo-id aakashv100/eval_so101-pick-cube-v2-smolvla-fixed --calib-cm 11.5
.\.venv\Scripts\python.exe scripts\measure_placement_error.py `
  --repo-id aakashv100/eval_so101-pick-cube-v2-smolvla-random --measure gripper `
  --ref 1:1.4,-4.7 --ref 3:2.4,1.8
```

**7. Score the sheet, then log to W&B:**

```powershell
.\.venv\Scripts\python.exe scripts\log_eval_to_wandb.py --xlsx SmolVLA_eval_scoresheet.xlsx `
  --project so101-smolvla --run-id o1ngsm6o --checkpoint checkpoints/020000 --dry-run
```

Drop `--dry-run` once the printed numbers look right.

### Sanity-run findings (2026-07-28)

Three trials recorded to `aakashv100/eval_sanity-smolvla` (4,523 frames, `-Mode fixed`,
default `num_steps`). Nothing was scored — the run was voided — but it surfaced three
things, two of which change how the full runs should be set up.

**1. The record-loop warning is misleading; the loop ran at 25 Hz.**
`lerobot_record` logs `Record loop is running slower (2.1-3.6 Hz) than the target FPS` roughly
once every 2 s. That figure is the instantaneous rate of a *single* overrunning iteration —
`dt_s` is reset at the top of every pass (`lerobot_record.py:563`) — not an average. Episode 2
recorded 1,501 frames in exactly 60 s of wall clock, i.e. 25.0 Hz; a real 3 Hz loop would have
produced ~180. Its 31 warnings match the 1501/50 = 30 chunk boundaries, one per boundary,
spaced by the 2 s a 50-action chunk lasts at 25 Hz. The time budget closes exactly:
31 stalls x ~350 ms + 1,470 ordinary frames x 33 ms = 59.9 s, so the chunk-boundary stall is
the *only* overhead and there is no hidden per-step cost. The stall is ~350 ms rather than the
251 ms measured offline because the camera reads, preprocessing and dataset write land on the
same iteration.

Consequence: the arm executes trajectories ~17% slower than the 30 fps demonstrations, with a
~350 ms pause every 2 s. Visibly jerky, not enough to distort a grasp. Note that dataset
timestamps are synthetic (`frame_index / fps`), so a recorded episode reports a clean 33 ms
median no matter how slow the loop actually ran — wall-clock rate must be derived from frame
count over episode duration.

**2. The policy hesitated for 11-51 s before committing to motion.**
Measured as the first frame where the summed absolute joint deviation from the start pose
exceeds 15 units:

| Source | Frames to first motion | Notes |
|--------|------------------------|-------|
| Training demos (50) | median 95, max 172 | the operator paused ~3 s before starting in 47 of 50 |
| ACT eval, Fixed (50) | median 65, max 96 | never failed to move |
| ACT eval, Random (50) | median 64, max 73 | never failed to move |
| **SmolVLA sanity (3)** | **288, 300, 1280** | third episode began moving with 9 s left |

The demos do contain a learned start-up pause, but ACT — trained on the same data — starts
*faster* than the demos and always moves. This is not the loop rate: at 25 Hz a 30 fps pause
stretches by only 1.2x, and the comparison above is in frames, which removes that confound.
Ruled out by direct check: the task string is byte-identical across the training and both eval
datasets, and all three start poses sit within 1 sigma of the 50 demo start poses on every
joint (max |z| = 0.9).

**3. Cause: unfamiliar objects in the top-camera view.**
`scripts/probe_start_hesitation.py` re-queries the policy 20 times on one observation and
counts how many sampled action chunks command real motion (peak summed |command - state|
over the chunk > 10). Composing observations from the ACT-era rig and the 2026-07-28 rig,
one input at a time, isolates the cause:

| Composite observation | Chunks commanding motion | Median peak deviation |
|-----------------------|--------------------------|-----------------------|
| All ACT-era | 20/20 | 21.3 |
| All 2026-07-28 | 16/20 | 11.1 |
| 07-28 with ACT-era **top** cam | 20/20 | 23.3 |
| 07-28 with ACT-era **gripper** cam | 9/20 | 9.8 |
| ACT-era with 07-28 **gripper** cam | 20/20 | 20.4 |
| ACT-era with 07-28 **top** cam | 20/20 | 13.2 |

Substituting the ACT-era top view restores decisive behaviour; substituting the gripper view
does not. On matched frames 0-60 (arm still parked in both) the policy is 20/20 with median
deviation 20-33 on ACT's Random episodes versus 8-18/20 at median 10-13 on 07-28.

What differs in the 07-28 top view (`outputs/topcam_compare.png`): a person's arm in the
top-right corner, a white box on the table left of the robot, a white ring at the right edge,
and a card at the bottom-right. None appear in any of the 50 demonstrations.

**The grid sheet is not the cause and should be kept** for Random trials — ACT's Random
episodes were recorded with it in place, and SmolVLA is fully decisive on those frames. Note
that global image statistics do *not* detect this: by mean absolute difference against the
training top-cam stream, the 07-28 frames score 0.107 versus 0.120 for ACT's Random session,
i.e. nominally *closer*. The sensitivity is to specific scene content, not brightness or
colour balance.

**Before the full runs:** clear the table of everything except the cube, bowl and (for Random)
the grid sheet, keep hands out of the top camera's field of view for the whole rollout, place
and read the cube position before starting the episode, and re-run
`probe_start_hesitation.py` against a fresh recording — 20/20 with median deviation above ~20
indicates the scene is back in distribution. That is a far cheaper feedback loop than spending
robot trials.

### First full-run attempt (2026-07-28, voided)

`run_eval.ps1 -Policy smolvla -Mode fixed -ClearCache` was launched before the scene was
cleared. It ended after 17 of 50 episodes when `gripper_cam` dropped out. Both failures are
documented here because each one alone would have voided the run.

**The scene was unchanged, so the trials reproduced the hesitation.** Mean absolute difference
of the top view against the training stream was 0.135, versus 0.138 for the voided sanity run —
the grid sheet, the white box, the white ring, the keyboard and the operator's arm were all
still in frame (`outputs/topcam_newrun.png`). Of the 17 recorded episodes, 7 never produced
meaningful motion (peak joint deviation < 25 units, i.e. a twitch: episodes 4, 6, 7, 8, 10, 13,
15) and the 10 that did move started at a median of 507 frames, about 20 s, against ACT's
65-frame median. Consistent with the probe: an uncleared scene costs trials, not just polish.

**The camera dropped out 44 min in.** OpenCV/MSMF raised `-1072873822`
(`MF_E_VIDEO_RECORDING_DEVICE_INVALIDATED`) on `OpenCVCamera(0)` at 13:38:57, i.e. 2661 s into
the session, and `lerobot_record` exited. The same two cameras had streamed fine for the
preceding 17 episodes, so this is a Windows/USB fault rather than a code defect — though the
code makes it unrecoverable: `camera_opencv.py:508` tolerates 10 consecutive read failures with
no backoff and no attempt to reopen the device, and all 11 retries landed inside 6 ms
(timestamps 2661.928 to 2661.934). A transient invalidation has no chance to clear.

Note the timing arithmetic: 50 trials at 60 s episode + 60 s reset is ~100 min of continuous
streaming, more than twice the observed 44 min time-to-failure. Long runs need to be split.

**Remediation applied (2026-07-28):**

| Action | Detail |
|--------|--------|
| USB selective suspend disabled | Was `Enabled` on both AC and DC in the Balanced scheme; set to `Disabled` via `powercfg /setacvalueindex` + `/setdcvalueindex` on `48e6b7a6-50f5-4782-a5d4-53bb8f07e226`. Effective without reboot. |
| Void dataset cleared locally | `~/.cache/huggingface/lerobot/aakashv100/eval_so101-pick-cube-v2-smolvla-fixed` (483.7 MB, 17 episodes) deleted so the next run starts clean |
| Run split recommended | `-ResetTime 20` brings a 25-trial half to ~22 min of streaming, inside the observed failure window; second half with `-Resume` |

**Still outstanding:** the void run reached the Hub at 13:39, so
`aakashv100/eval_so101-pick-cube-v2-smolvla-fixed` holds the 17 contaminated episodes remotely.
`-ClearCache` clears only the local cache, so the repo should be deleted before re-running or
the new push will land alongside stale chunk files. Physical steps not yet done: clearing the
table, removing the grid sheet for the Fixed tab (it is only needed to read Random
coordinates), and moving the two cameras onto separate USB controllers.

---

## 9. Known issues and mitigations

| Issue | Impact | Mitigation applied |
|-------|--------|-------------------|
| `--policy.path` with a Hub repo id fails on Windows | `Path("lerobot/smolvla_base")` becomes `lerobot\smolvla_base`, which `hf_hub_download` rejects | `train_smolvla.ps1` snapshot-downloads the base model to `pretrained-model/smolvla_base` and passes a local directory |
| Base checkpoint uses `camera1/2/3` keys | Feature-mismatch error, or a policy fed camera keys it never saw | `--rename_map` at train time; `run_eval.ps1 -Policy smolvla` re-supplies it because `lerobot_record` overrides the checkpoint's own rename step |
| 251 ms inference vs 33 ms control period | ~250 ms hold at each chunk boundary | Accepted at the default; `-DenoiseSteps` / `-CompilePolicy` / `-InterpolationMultiplier` available |
| Scoresheet Random tab column shift | `log_eval_to_wandb.py` read `Gripper Pos` as `Grasped?` for both policies | Per-tab column map plus `tests/test_log_eval_to_wandb.py` |
| openpyxl writes inline strings | Hand-rolled xlsx reader silently saw an empty sheet | Reader now handles `t="inlineStr"` |
| Windows dataloader stall (`num_workers > 0`) | Training appears frozen at step 0 | `num_workers=0` |
| No validation split | Cannot measure generalization offline | Offline metrics are explicitly training accuracy; the real-robot Random tab is the generalization test |
| Fewer epochs than ACT | 5.3 vs 8.0 epochs at 20k steps | Batch raised from 4 to 16 after the smoke run showed 2.6 GB VRAM use; further headroom remains |
| Unfamiliar objects in the top-camera view stall the policy | 11-51 s of hesitation before the arm commits; one sanity episode never completed a rollout | Clear the scene and keep hands out of frame; verify with `scripts/probe_start_hesitation.py` before spending trials. ACT tolerated the same shift, so this is a SmolVLA robustness limit, not a rig defect |
| `Record loop is running slower (N Hz)` warning reads as a sustained rate | Prompted a wrong diagnosis of ~3 Hz throughput; actual rate was 25 Hz | The figure is one iteration's instantaneous rate; derive the real rate from frame count over episode wall-clock duration |
| Eval-scene drift between policy runs | ACT's 100 trials were recorded on an earlier rig state, so cross-policy comparison is only valid if the scene matches | Restore the training scene before scoring; if it cannot be restored, ACT's trials must be re-run on the final setup |
| `gripper_cam` dies mid-run with MSMF `-1072873822` (`MF_E_VIDEO_RECORDING_DEVICE_INVALIDATED`) | Killed the first full Fixed run 44 min in, at episode 17 of 50 | USB selective suspend disabled via `powercfg` (was Enabled on AC and DC); put the two cameras on separate USB controllers. The read loop gives up after 10 consecutive failures with no backoff or reopen (`camera_opencv.py:508`), so all 11 retries burn in ~6 ms and a transient dropout is fatal |
| A 50-trial run needs ~100 min of continuous camera streaming | Longer than the observed 44 min time-to-failure | `-ResetTime 20` cuts a 50-trial run to ~45 min; split into halves with `-Resume` so a dropout costs at most half the run |

---

## 10. Artifacts

| Artifact | Path |
|----------|------|
| Final model | `outputs/train/smolvla_so101_pick_cube/checkpoints/last/pretrained_model/` |
| Base model (local copy) | `pretrained-model/smolvla_base/` |
| Training log | `logs/train_smolvla_20k.log` |
| W&B dashboard | https://wandb.ai/aakashvardhan-madabhushi-san-jose-state-university/so101-smolvla/runs/o1ngsm6o |
| Training script | `scripts/train_smolvla.ps1` |
| Eval runner | `scripts/run_eval.ps1 -Policy smolvla` |
| Scoresheet | `SmolVLA_eval_scoresheet.xlsx` (from `scripts/make_eval_scoresheet.py`) |
| Offline eval script | `test_inference_offline.py` |
| Scene / hesitation probe | `scripts/probe_start_hesitation.py` |
| Top-camera scene comparison | `outputs/topcam_compare.png`, `outputs/topcam_newrun.png` |
| Voided sanity dataset | [aakashv100/eval_sanity-smolvla](https://huggingface.co/datasets/aakashv100/eval_sanity-smolvla) (3 episodes, 2026-07-28) |
| Voided Fixed run (Hub only; local copy deleted) | [aakashv100/eval_so101-pick-cube-v2-smolvla-fixed](https://huggingface.co/datasets/aakashv100/eval_so101-pick-cube-v2-smolvla-fixed) (17 of 50 episodes, 2026-07-28) |
| Dataset (local) | `%USERPROFILE%\.cache\huggingface\lerobot\aakashv100\so101-pick-cube-v2` |

---

## 11. Conclusions

1. **Finetuning worked and converged.** 20,000 steps at batch 16 took 5.7 h on one 16 GB laptop GPU, ending at loss 0.039 with the LR fully decayed and gradient norms falling throughout.
2. **SmolVLA fits the demonstrations better than ACT v2** on identical frames: MAE 1.06 vs 1.49, frame accuracy 89.0% vs 72.0%, with one third the gradient steps. The pretrained VLM prior is doing real work, most visibly on the reach-phase joints ACT struggled with.
3. **Its cost is latency, and the cost is bounded.** 251 ms per query against an 18 ms ACT baseline and a 33 ms control period, but because it re-queries only every 50 frames the measured loop rate is 25 Hz with a ~350 ms stall every 2 s — a 17% slowdown, not a breakdown. `-DenoiseSteps 5` halves the stall if it ever matters.
4. **The real closed-loop risk turned out to be visual robustness, not latency.** In the sanity run the policy stalled for 11-51 s when the top camera showed a few objects absent from training — a person's arm and some desk clutter. ACT, trained on the same 50 demonstrations, tolerated the same scene shift. The offline metrics in section 6 rank SmolVLA higher on every joint and are structurally blind to this, because they are teacher-forced on demonstration frames and never ask the policy to initiate motion from a standstill in a changed scene.
5. **Task success is still unmeasured.** Offline accuracy on training episodes does not predict closed-loop success. The 100 trials in section 8 remain the definitive test, and they are only meaningful with the scene restored.

---

## 12. Open items

1. **Real-robot trials pending** for SmolVLA (section 8). Both 2026-07-28 attempts are void. In order: delete the Hub repo `aakashv100/eval_so101-pick-cube-v2-smolvla-fixed`, clear the table and move the cameras onto separate USB controllers, re-run the 3-trial sanity check and confirm it with `scripts/probe_start_hesitation.py`, then run 50 Fixed + 50 Random in 25-trial halves.
2. **Quantify the visual-robustness gap.** The sanity run showed it qualitatively; a clean measurement would probe both policies over the same set of perturbed scenes and report chunks-commanding-motion and frames-to-first-motion side by side. `frames to first motion` is cheap to compute from any eval recording and is the metric that separated the two policies closed-loop.
3. **ACT's Random tab is unscored** even though `eval_so101-pick-cube-v2-random` holds all 50 recorded episodes. The Random half of the comparison is blocked until those trials are scored from the recordings.
4. **Optional:** push the finetuned checkpoint to the Hub (`-PushToHub`) for reproducibility.
5. **Optional:** hold out episodes 40-49 and retrain, to turn the offline accuracy into a generalization number rather than a fitting one. On this dataset that is the only offline way to separate "fitted the demos" from "learned the task".
6. **Optional:** raise the batch further (5.1 GB of 16.3 GB used) or extend the run; the loss had flattened, so more steps need a longer decay horizon to help.
