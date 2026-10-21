# SmolVLA Holdout Finetuning on SO-ARM 101 (45 / 5 split)

**Project:** SO-101 pick-cube imitation learning  
**Date:** 2026-07-31  
**Framework:** [LeRobot](https://github.com/huggingface/lerobot) v3.0  
**Policy:** SmolVLA (vision-language-action, flow matching), finetuned from [`lerobot/smolvla_base`](https://huggingface.co/lerobot/smolvla_base)  
**Job:** `smolvla_so101_pick_cube_holdout`  
**W&B run:** [i4t0f3la](https://wandb.ai/aakashvardhan-madabhushi-san-jose-state-university/so101-smolvla/runs/i4t0f3la)  
**Companion report:** [`SmolVLA_training_report.md`](SmolVLA_training_report.md) (full 50-episode fit, no holdout)

---

## Executive summary

Same recipe as the original SmolVLA finetune, but with **5 episodes held out** of training so validation loss and offline accuracy measure generalization to unseen demonstrations rather than fitting. Held-out episodes are every tenth index — `4, 14, 24, 34, 44` — to avoid confounding generalization with late-session operator/lighting drift.

| Metric | Holdout (this run) | Full-data SmolVLA | ACT v2 (reference) |
|--------|--------------------|-------------------|--------------------|
| Train / val episodes | 45 / 5 | 50 / 0 | 50 / 0 |
| Train frames | 53,998 | 59,998 | 59,998 |
| Training steps | 20,000 | 20,000 | 60,000 |
| Batch size | 16 | 16 | 8 |
| Effective epochs | 5.93 | 5.33 | 8.0 |
| Final train loss | 0.037 | 0.039 | *see W&B `55h5xmgg`* |
| Final val loss | **0.153** | n/a | n/a |
| Best val loss | **0.122** @ 5k & 7.5k | n/a | n/a |
| Train-set MAE (200 frames) | **0.98** | 1.06 | 1.49 |
| Train-set joint / frame acc. (±5) | **98.8% / 94.5%** | 97.8% / 89.0% | 94.8% / 72.0% |
| Holdout MAE (200 frames) | **1.20** | n/a | n/a |
| Holdout joint / frame acc. (±5) | **97.1% / 84.5%** | n/a | n/a |
| Inference latency / query | 238–243 ms | 251 ms | 18 ms |
| Wall-clock | 6.3 h | 5.7 h | ~13 h |
| Peak step time | ~1.00 s (`updt`≈0.50 / `data`≈0.50) | 1.03 steps/s | ~1.3 steps/s |

**Offline result:** the final checkpoint generalizes to the five held-out episodes at MAE 1.20 / frame accuracy 84.5% — worse than its own train-set fit (0.98 / 94.5%) but still better than ACT v2's *training-set* fit (1.49 / 72.0%). Validation loss bottomed at steps 5,000–7,500 (`val/loss` 0.122) and rose to 0.153 by step 20,000 while train loss kept falling, so the run overfit the train episodes on the flow-matching loss. The last checkpoint still beats the best-`val/loss` checkpoint (6,000) on holdout action accuracy (MAE 1.20 vs 2.01), so loss and teacher-forced accuracy do not pick the same checkpoint.

**Status:** training finished; offline train + holdout metrics measured on `checkpoints/last`. Real-robot Fixed/Random scoring is still pending (same handover as the full-data report).

---

## 1. Task and robot setup

Identical to the full-data SmolVLA and ACT v2 runs.

- **Instruction:** `Pick up the cube and place it in the bowl`
- **Robot:** SO-101 follower (`so_follower`) on `COM3`, calibration `calibration/robots/so_follower/my_so_arm.json`
- **Cameras:** `gripper_cam` (OpenCV index 0), `top_cam` (wide-angle USB, index 1), both 640×480 @ 30 fps, MJPG
- **Action/state space:** 6-D joint position targets (`shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`)

---

## 2. Dataset and split

| Property | Value |
|----------|-------|
| Hub | [aakashv100/so101-pick-cube-v2](https://huggingface.co/datasets/aakashv100/so101-pick-cube-v2) |
| Local root | `%USERPROFILE%\.cache\huggingface\lerobot\aakashv100\so101-pick-cube-v2` |
| Total episodes / frames | 50 / 59,998 |
| FPS | 30 |
| Train episodes | 45 (all except the holdout) → **53,998 frames** |
| Val episodes | **`[4, 14, 24, 34, 44]`** → **6,000 frames** |
| Val frequency | every 2,500 steps (+ final step) |
| Normalizer stats | still computed on the **whole** dataset (LeRobot default) |

Every demonstration starts the cube on the "A" paper marker (spread < 1 cm), so the training distribution still contains **zero cube-position randomization**. The holdout tests generalization to unseen *demonstrations of the same scene*, not to novel cube positions — the Random tab on the robot remains that test.

---

## 3. Model architecture

Unchanged from the full-data run / `smolvla_base` finetune recipe.

| Hyperparameter | Value |
|----------------|-------|
| `vlm_model_name` | `HuggingFaceTB/SmolVLM2-500M-Video-Instruct` (frozen) |
| `num_vlm_layers` | 16 |
| `attention_mode` | `cross_attn` |
| `expert_width_multiplier` | 0.75 |
| `chunk_size` / `n_action_steps` | 50 / 50 |
| `num_steps` (denoising) | 10 |
| `freeze_vision_encoder` / `train_expert_only` | true / true |
| `train_state_proj` | true |
| Normalization | IDENTITY (VISUAL), MEAN_STD (STATE, ACTION) |
| Parameters | 450,046,176 total, **99,880,992 trainable** |

### Camera key mapping

```
--rename_map={observation.images.top_cam: observation.images.camera1,
              observation.images.gripper_cam: observation.images.camera2}
```

Baked into the checkpoint preprocessor; `run_eval.ps1 -Policy smolvla` re-supplies it at eval time because `lerobot_record` overrides that step from `--dataset.rename_map`.

---

## 4. Training configuration

### Command

```powershell
.\scripts\train_smolvla.ps1 `
  -ValEpisodes 4,14,24,34,44 `
  -ValFreq 2500 `
  -Wandb -WandbProject so101-smolvla `
  -JobName smolvla_so101_pick_cube_holdout
```

### Optimizer and schedule

| Setting | Value |
|---------|-------|
| Optimizer | AdamW |
| Peak LR | 1×10⁻⁴ |
| Betas / eps | (0.9, 0.95) / 1×10⁻⁸ |
| Weight decay | 1×10⁻¹⁰ |
| Gradient clip | 10.0 |
| Scheduler | cosine decay with warmup |
| Warmup / decay steps | 1,000 / 20,000 |
| Final LR | 2.5×10⁻⁶ |
| Batch size | 16 |
| Steps | 20,000 |
| Seed | 1000 |
| `num_workers` | 0 (Windows) |
| Video backend | pyav |
| Augmentation / bf16 | off |

### Infrastructure

| Setting | Value |
|---------|-------|
| Host | `IDS-TWIN-LAB-08` |
| GPU | NVIDIA RTX 5080 Laptop, 16 GB (Blackwell) |
| CUDA / Python | 13.1 / 3.12.10 |
| Device | CUDA, fp32 |
| Started / ended | 2026-07-31 16:09:32 → 22:28:50 (local) |
| W&B `_runtime` | 22,754 s (**6.32 h**) |

Checkpoints every 2,000 steps into `outputs/train/smolvla_so101_pick_cube_holdout/checkpoints/` (`002000` … `020000` + `last`).

---

## 5. Training results

### Train loss curve (console `log_freq=200` windows)

| Step | Train loss | Grad norm | LR | Notes |
|------|-----------|-----------|-----|-------|
| 200 | 0.301 | 4.241 | 1.0×10⁻⁵ | warmup |
| 1,000 | 0.159 | 2.816 | 9.0×10⁻⁵ | end of warmup |
| 2,000 | 0.127 | 2.149 | 9.8×10⁻⁵ | first ckpt |
| 5,000 | 0.090 | 1.613 | 8.6×10⁻⁵ | |
| 10,000 | 0.055 | 1.141 | 5.2×10⁻⁵ | |
| 15,000 | 0.038 | 0.838 | 1.7×10⁻⁵ | |
| **20,000** | **0.037** | **0.765** | 2.5×10⁻⁶ | final |

Smooth monotone decrease; grad norms stayed well under the clip of 10. Loss flattened over the last ~5k steps as the cosine schedule drove LR to 2.5×10⁻⁶ — same convergence shape as the full-data run (final 0.037 vs 0.039).

320,000 samples seen ÷ 53,998 train frames = **5.93 epochs**.

### Validation loss curve (every 2,500 steps, all 6,000 holdout frames)

| Step | `val/loss` | Val wall time |
|------|------------|---------------|
| 2,500 | 0.130 | 344 s |
| **5,000** | **0.122** | 355 s |
| **7,500** | **0.122** | 333 s |
| 10,000 | 0.134 | 335 s |
| 12,500 | 0.141 | 338 s |
| 15,000 | 0.147 | 342 s |
| 17,500 | 0.151 | 331 s |
| **20,000** | **0.153** | 343 s |

**Observations:**

- Best `val/loss` is **0.122** at steps 5,000 and 7,500. After that, val rises steadily while train loss continues to fall — classic overfit on the 45 train episodes.
- Each val pass costs ~5.5–6 min (~19 frames/s, video-decode bound with `num_workers=0`). Eight passes added ~46 min; total wall-clock 6.3 h vs 5.7 h for the no-val full-data run.
- Nearest saved checkpoints to the val minimum are `006000` and `008000`. See §6 — teacher-forced holdout accuracy prefers `last` over `006000`.

### Duration and throughput

- **Wall-clock:** 6.32 h (W&B `_runtime`); training-step tqdm alone was 6 h 13 m before the final val pass.
- **Throughput:** ~1.00 s/step at batch 16 (`updt_s` ≈ 0.50, `data_s` ≈ 0.50) — same half data-bound profile as the full-data run.
- Holdout split did not change the per-step data path cost (confirmed at step 200: `updt_s` 0.482 / `data_s` 0.537).

### Simulation evaluation during training

**Not performed.** `env: null`; task success is measured on the physical robot.

---

## 6. Offline evaluation

### Method

```powershell
# Holdout generalization (episodes never trained on)
.\.venv\Scripts\python.exe test_inference_offline.py `
  --checkpoint outputs/train/smolvla_so101_pick_cube_holdout/checkpoints/last/pretrained_model `
  --repo-id aakashv100/so101-pick-cube-v2 `
  --root "$env:USERPROFILE\.cache\huggingface\lerobot\aakashv100\so101-pick-cube-v2" `
  --device cuda --num-frames 200 --episodes 4 14 24 34 44

# Train-set fit (the 45 episodes used for training)
.\.venv\Scripts\python.exe test_inference_offline.py `
  --checkpoint outputs/train/smolvla_so101_pick_cube_holdout/checkpoints/last/pretrained_model `
  --repo-id aakashv100/so101-pick-cube-v2 `
  --root "$env:USERPROFILE\.cache\huggingface\lerobot\aakashv100\so101-pick-cube-v2" `
  --device cuda --num-frames 200 --episodes 0 1 2 3 5 6 7 8 9 10 11 12 13 15 16 17 18 19 20 21 22 23 25 26 27 28 29 30 31 32 33 35 36 37 38 39 40 41 42 43 45 46 47 48 49
```

Tolerance ±5 on each joint target. Metrics are teacher-forced action accuracy, not closed-loop success.

### Final checkpoint (`020000` / `last`)

| Joint | Train MAE | Train acc. | Holdout MAE | Holdout acc. |
|-------|-----------|------------|-------------|--------------|
| shoulder_pan | 0.76 | 99.0% | 0.80 | 99.0% |
| shoulder_lift | 1.31 | 99.0% | 1.51 | 95.5% |
| elbow_flex | 1.46 | 98.5% | 1.52 | 98.0% |
| wrist_flex | 0.82 | 100.0% | 0.91 | 99.5% |
| wrist_roll | 0.53 | 98.5% | 0.65 | 100.0% |
| gripper | 1.01 | 97.5% | 1.82 | 90.5% |
| **Overall MAE** | **0.98** | | **1.20** | |
| **Joint accuracy** | | **98.8%** | | **97.1%** |
| **Frame accuracy** | | **94.5%** | | **84.5%** |

**Interpretation:**

- Train-set fit is slightly tighter than the full-data SmolVLA run (MAE 0.98 vs 1.06, frame accuracy 94.5% vs 89.0%) despite 5 fewer train episodes — consistent with 5.93 vs 5.33 epochs on a smaller set.
- Holdout gap is real but modest on most arm joints; **`gripper` is the weak link** (train MAE 1.01 → holdout 1.82; accuracy 97.5% → 90.5%). Grasp timing is what decides success on the robot.
- Frame accuracy drops more than joint accuracy (94.5% → 84.5%) because holdout errors are less correlated across joints than train errors — same pattern seen when comparing SmolVLA to ACT.
- This is still **same-scene, same-cube-position** generalization. It does not predict Random-tab success.

### Best-`val/loss` checkpoint vs last (holdout episodes)

| Checkpoint | Step | `val/loss` | Holdout MAE | Holdout joint / frame acc. |
|------------|------|------------|-------------|----------------------------|
| `006000` | 6,000 | ~0.122 (nearest to min) | **2.01** | 91.4% / **61.0%** |
| `last` (`020000`) | 20,000 | 0.153 | **1.20** | 97.1% / **84.5%** |

Flow-matching `val/loss` and teacher-forced action accuracy disagree on which checkpoint to keep. Prefer `last` for offline action fidelity unless robot trials show otherwise; if scoring by `val/loss` alone, also robot-test `006000` / `008000`.

### Cross-run comparison (same script, 200 frames, ±5)

| Run | Scope | MAE | Joint acc. | Frame acc. | Latency mean |
|-----|-------|-----|------------|------------|--------------|
| Holdout SmolVLA `last` | train eps | **0.98** | **98.8%** | **94.5%** | 243 ms |
| Holdout SmolVLA `last` | holdout eps | **1.20** | **97.1%** | **84.5%** | 238 ms |
| Full-data SmolVLA `last` | all 50 (train) | 1.06 | 97.8% | 89.0% | 251 ms |
| ACT v2 | all 50 (train) | 1.49 | 94.8% | 72.0% | 18 ms |

### Inference latency

Holdout `last`: **238 ms mean / 340 ms max** on holdout frames (first query 817 ms cold); **243 ms mean / 266 ms max** on train frames. Same ballpark as the full-data run (~251 ms) and still ~14× slower than ACT v2. Closed-loop implications unchanged: re-query every 50 actions → ~350 ms stall per chunk, ~25 Hz loop (see full-data report §8).

---

## 7. Evaluation protocol (robot)

Unchanged. Scoresheet: `SmolVLA_eval_scoresheet.xlsx`.

- **Fixed tab:** 50 trials, cube on A marker.
- **Random tab:** 50 trials, same 50 grid positions as ACT.
- One checkpoint locked for both tabs. No manual assist.
- Failure modes: `no-grasp` / `grasped-dropped` / `wrong-placement` / `other`.

Recommended checkpoint for the first scored runs: `outputs/train/smolvla_so101_pick_cube_holdout/checkpoints/last` (best holdout action accuracy). Optionally A/B a short sanity against `006000` if you want a `val/loss`-selected control.

---

## 8. Real-robot evaluation

Still pending. Use the same cleared-scene procedure and USB mitigations documented in [`SmolVLA_training_report.md`](SmolVLA_training_report.md) §8 (voided 2026-07-28 attempts, top-cam clutter hesitation, `gripper_cam` MSMF dropout).

Point the eval preset at this run's checkpoint:

```powershell
.\scripts\run_eval.ps1 -Policy smolvla -Mode fixed `
  -Checkpoint outputs/train/smolvla_so101_pick_cube_holdout/checkpoints/last/pretrained_model `
  -ClearCache
```

Log scored results to this W&B run:

```powershell
.\.venv\Scripts\python.exe scripts\log_eval_to_wandb.py --xlsx SmolVLA_eval_scoresheet.xlsx `
  --project so101-smolvla --run-id i4t0f3la --checkpoint checkpoints/020000 --dry-run
```

Backfill offline metrics to W&B summary:

```powershell
.\.venv\Scripts\python.exe scripts\log_offline_acc_to_wandb.py --run-id i4t0f3la `
  --episodes 4 14 24 34 44 `
  --checkpoint outputs/train/smolvla_so101_pick_cube_holdout/checkpoints/last/pretrained_model
```

---

## 9. Known issues specific to this run

| Issue | Impact | Notes |
|-------|--------|-------|
| `val/loss` rises after 7.5k while train loss falls | Overfitting signal on flow-matching loss | Last ckpt still wins on holdout action accuracy; do not auto-pick by `val/loss` alone |
| Normalizer stats include val episodes | Mild train→val leakage in MEAN_STD stats | LeRobot default; affects both train and val equally; not a split of the data distribution |
| Holdout is same-scene / same cube pose | Offline “generalization” ≠ Random-tab generalization | Random tab remains the position-generalization test |
| No offline metrics in W&B summary yet | Dashboard missing MAE / accuracy | Run `log_offline_acc_to_wandb.py` as above |
| Windows `num_workers>0` deadlock | Forces `num_workers=0`, ~half step time in decode | Unchanged; see `train_smolvla.ps1` notes |
| Visual clutter stalls closed-loop start | Not re-tested on this checkpoint | Clear scene; verify with `scripts/probe_start_hesitation.py` |

---

## 10. Artifacts

| Artifact | Path |
|----------|------|
| Final model | `outputs/train/smolvla_so101_pick_cube_holdout/checkpoints/last/pretrained_model/` |
| Best-`val/loss` region | `.../checkpoints/006000/`, `.../008000/` |
| All checkpoints | `outputs/train/smolvla_so101_pick_cube_holdout/checkpoints/{002000…020000,last}/` |
| Train config | `.../checkpoints/020000/pretrained_model/train_config.json` |
| W&B local run | `outputs/train/smolvla_so101_pick_cube_holdout/wandb/run-20260731_160932-i4t0f3la/` |
| W&B dashboard | https://wandb.ai/aakashvardhan-madabhushi-san-jose-state-university/so101-smolvla/runs/i4t0f3la |
| Training script | `scripts/train_smolvla.ps1` |
| Offline eval | `test_inference_offline.py` |
| Offline → W&B | `scripts/log_offline_acc_to_wandb.py` |
| Full-data companion report | `SmolVLA_training_report.md` |
| Dataset (local) | `%USERPROFILE%\.cache\huggingface\lerobot\aakashv100\so101-pick-cube-v2` |

---

## 11. Conclusions

1. **Holdout training completed and converged.** 20k steps, batch 16, 5.93 epochs over 45 demos, 6.3 h wall-clock including eight full-holdout val passes. Final train loss 0.037 matches the full-data run.
2. **We now have a real generalization offline number.** Holdout MAE 1.20 / frame accuracy 84.5% vs train-set 0.98 / 94.5%. Gripper timing accounts for most of the gap.
3. **`val/loss` and action accuracy disagree on the checkpoint.** Val bottoms at 5k–7.5k (0.122) then rises to 0.153; `006000` is worse on holdout MAE/frame accuracy than `last`. Prefer action-accuracy (or robot trials) over flow-matching val loss for checkpoint selection here.
4. **Latency and closed-loop risks are unchanged** from the full-data report (~240 ms query, scene-clutter hesitation, USB camera dropout on long runs).
5. **Task success is still unmeasured.** Offline holdout accuracy is necessary but not sufficient; Fixed + Random robot trials remain the definitive comparison against ACT v2.

---

## 12. Open items

1. **Push offline metrics to W&B** for run `i4t0f3la` (`log_offline_acc_to_wandb.py` with `--episodes 4 14 24 34 44`).
2. **Optional:** offline-eval `008000` and `010000` on holdout episodes to map the accuracy curve around the val minimum.
3. **Real-robot trials** with `checkpoints/last` (and a short A/B vs `006000` if desired), cleared scene, USB fix verified — see full-data report §8.
4. **Score ACT Random tab** so the Random half of the policy comparison is not blocked.
5. **Optional:** push the holdout checkpoint to the Hub (`-PushToHub`).
