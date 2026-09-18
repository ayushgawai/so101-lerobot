# SO-101 Pick-and-Place Dataset Recording — Progress Report

**Project:** SO-ARM 101 imitation learning (LeRobot)  
**Date:** 2026-09-18  
**Hardware:** SO-101 follower + leader, 2× USB cameras (gripper + top)  
**Framework:** [LeRobot](https://github.com/huggingface/lerobot) (local fork `so101-lerobot`)  
**Intended next step:** ACT policy training on the new multi-position dataset  

---

## 1. Executive summary

We set up a **repeatable, interactive teleoperation recording workflow** for a pick-and-place task where the cube is picked from numbered sheet positions (1–50) and dropped into a **fixed box**.

**Today’s progress**
- Built recording scripts with interactive prompts (position + iteration count).
- Recorded **25 successful demonstrations**: sheet positions **6–10**, **5 demos each**.
- Dataset is stored locally and is ready to **append** more positions in later sessions without creating a new dataset.
- Side-by-side live camera preview is available during recording.
- Next planned work: continue filling positions, then train **ACT** on the combined dataset.

| Item | Status |
|------|--------|
| Recording tooling | Done |
| Positions 6–10 (5× each) | Done (25 episodes) |
| Remaining positions (1–5, 11–50) | Pending |
| ACT training on this dataset | Pending |

---

## 2. Task definition

| Field | Value |
|-------|--------|
| Task | Pick up the cube from a marked sheet position and place it in the box |
| Pickup | Sheet positions **1–50** (unique id per pickup location) |
| Drop | **Fixed box** (same target every episode; no position id) |
| Control | Human teleoperation (leader arm → follower arm) |
| Sensors | Joint state (6-DoF) + `gripper_cam` + `top_cam` |
| Rate | 30 fps |

Each saved demonstration (episode) includes:
- Joint **observations** and **actions** (6 motors)
- Synchronized **video** from both cameras
- A **task string** that embeds the pickup position (e.g. *“Pick up the cube from sheet position 6 and place it in the box”*)
- A sidecar log mapping `episode_index → pickup_position`

---

## 3. Dataset status (as of 2026-09-18)

| Property | Value |
|----------|--------|
| Local path | `hf_data/so101-pick-place-positions/` |
| Dataset id | `aakashv100/so101-pick-place-positions` |
| Format | LeRobot v3.0 |
| Episodes | **25** |
| Frames | **18,890** |
| FPS | 30 |
| Robot type | `so_follower` (SO-101) |
| Features | `action`, `observation.state`, both camera videos, timestamps, task index |

### Coverage so far

| Pickup position | Episodes |
|-----------------|----------|
| 6 | 5 |
| 7 | 5 |
| 8 | 5 |
| 9 | 5 |
| 10 | 5 |
| **Total** | **25** |

### Key files to inspect

| File | Purpose |
|------|---------|
| `hf_data/so101-pick-place-positions/meta/info.json` | Dataset totals and feature schema |
| `hf_data/so101-pick-place-positions/pickup_positions.jsonl` | One line per episode: position + episode index |
| `hf_data/so101-pick-place-positions/pickup_positions_summary.json` | Human-readable counts by position |
| `hf_data/so101-pick-place-positions/data/` | Parquet tables (state/action) |
| `hf_data/so101-pick-place-positions/videos/` | H.264 videos per camera |

Episode lengths vary (~16–60 s). That is expected for human demos and is fine for ACT.

---

## 4. Recording software we created

### Scripts

| Script | Role |
|--------|------|
| `scripts/record_pickplace.py` | Main recorder: connects robot/teleop/cameras, prompts for position & iterations, saves LeRobot episodes + sidecar logs |
| `scripts/record_pickplace.ps1` | One-command Windows wrapper with SO-101 defaults (COM ports, calibration paths, cameras) |

### Design goals

1. **Simple command** — no hardcoded episode totals in the launch command.
2. **Interactive blocks** — before each position block, ask:
   - pickup position (1–50)
   - number of iterations for that position
3. **Append-friendly** — re-running the same command continues the **same** dataset (new positions or more iterations of an existing position).
4. **Operator feedback** — side-by-side camera popup (gripper | top) while teleoperating.
5. **Traceability** — position stored in dataset task strings **and** in `pickup_positions.jsonl`.

### What gets recorded every episode

- 6-DoF follower joint state and teleop actions  
- `gripper_cam` and `top_cam` @ 640×480, 30 fps (MJPG capture, H.264 storage)  
- Task label including sheet position  
- Episode metadata suitable for later ACT / VLA training  

---

## 5. How to record (step-by-step for another operator)

### Prerequisites

1. SO-101 **follower** and **leader** powered and connected (typical ports: follower `COM3`, leader `COM4`).
2. Both arms **calibrated** (see repo `README.md`).
3. Both USB cameras plugged in (gripper index `0`, top index `1` by default).
4. Repo environment activated:

```powershell
cd <path-to>\so101-lerobot
.\.venv\Scripts\Activate.ps1
```

Optional sanity check before recording:

```powershell
lerobot-teleoperate `
  --robot.type=so101_follower --robot.port=COM3 --robot.id=my_so_arm `
  --robot.calibration_dir=./calibration/robots/so_follower `
  --teleop.type=so101_leader --teleop.port=COM4 --teleop.id=my_so_arm `
  --teleop.calibration_dir=./calibration/teleoperators/so_leader `
  --display_data=true
```

### Start recording (same command every session)

```powershell
.\scripts\record_pickplace.ps1
```

- If the dataset folder **does not exist** → creates a new local dataset.  
- If it **already exists** with real episodes → **appends** automatically.

### Interactive prompts

1. A **camera window** opens (gripper | top). Press **Enter** in the terminal when ready.
2. Prompt: **Pickup sheet position** → e.g. `6`
3. Prompt: **Iterations for this position** → e.g. `5`
4. For each iteration:
   1. Teleop: pick cube → place in box  
   2. Press **Right arrow** to **save** that demo  
   3. Reset cube onto the same sheet mark  
   4. Press **Right arrow** again to end reset and start the next iteration  
5. After the block finishes, enter the **next position** (or `q` to quit).

### Keyboard controls (during teleop / reset)

| Key | Action |
|-----|--------|
| **Right arrow** | End current phase (save episode, or finish reset) |
| **Left arrow** | Discard current episode and re-record it |
| **Esc** | Stop the whole session |
| **`q`** at a text prompt | Quit cleanly without starting a new block |

### Later sessions (more positions)

Use the **same** command again:

```powershell
.\scripts\record_pickplace.ps1
```

Examples:
- New positions `15`–`25` with 5 demos each → enter each position and `5`.
- Extra demos for position `6` → enter `6` and how many more to add.

Everything appends into `hf_data/so101-pick-place-positions/`.

### Wipe and start over (only if needed)

```powershell
Remove-Item -Recurse -Force hf_data/so101-pick-place-positions
.\scripts\record_pickplace.ps1
```

---

## 6. Command cheat sheet

```powershell
# Activate environment
.\.venv\Scripts\Activate.ps1

# Record / append pick-and-place demos (interactive)
.\scripts\record_pickplace.ps1

# Optional: refuse to start if a dataset already exists
.\scripts\record_pickplace.ps1 -Fresh

# Optional: upload to Hugging Face Hub when finished
.\scripts\record_pickplace.ps1 -PushToHub

# Optional: also open Rerun visualization
.\scripts\record_pickplace.ps1 -DisplayData
```

Default hardware assumptions in the wrapper:
- Robot `COM3`, teleop `COM4`, id `my_so_arm`
- Calibration under `./calibration/robots/so_follower` and `./calibration/teleoperators/so_leader`
- Cameras: gripper `0`, top `1`

Override example:

```powershell
.\scripts\record_pickplace.ps1 -RobotPort COM5 -TeleopPort COM6 -GripperCamIndex 1 -TopCamIndex 2
```

---

## 7. Why this workflow (for training later)

- **One dataset** across days avoids splitting/merging headaches for ACT.  
- **Position metadata** supports analysis (e.g. success by location) and optional language-conditioned policies later.  
- **ACT** primarily needs consistent state/action/video; the interactive logger does not change that core format.  
- Existing training entrypoint in this repo: `scripts/train_act.ps1` (will be pointed at this new dataset when recording coverage is sufficient).

---

## 8. Next steps

1. Continue recording remaining sheet positions (same script; append mode).  
2. Spot-check a few episodes visually (videos under `hf_data/.../videos/`).  
3. Optionally push dataset to the Hub (`-PushToHub` or manual upload).  
4. Train ACT with local root:

```powershell
.\scripts\train_act.ps1 `
  -RepoId aakashv100/so101-pick-place-positions `
  -DatasetRoot hf_data/so101-pick-place-positions `
  -JobName act_so101_pick_place_positions `
  -Wandb
```

(Exact hyperparameters can match prior ACT runs documented in `ACT_training_report.md`.)

---

## 9. Related repo docs

| Document | Content |
|----------|---------|
| `README.md` | Windows SO-101 setup, calibrate, teleop, motor ID fix |
| `ACT_training_report.md` | Prior ACT training on the older single-position pick-cube set |
| `SmolVLA_training_report.md` / `SmolVLA_holdout_training_report.md` | Prior SmolVLA runs |

---

## 10. Short status for email / Slack

> We implemented an interactive pick-and-place recording pipeline for the SO-101 (leader/follower + dual cameras). Operators enter sheet pickup position and demo count; data appends to one LeRobot dataset with position metadata. On 2026-09-18 we recorded positions 6–10 (5 demos each = 25 episodes, 18,890 frames). Remaining positions will be appended in later sessions, then we will train ACT on the combined dataset.
