# Evaluation protocol (SO-101 pick-and-place)

**Protocol version:** 1.0  
**Frozen:** Week 1 (Fall 2026)  
**Source:** Research Plan Fall 2026 §6  
**Owners:** Aakash (author), Ayush (review — flag anything unclear)

Every number reported in the paper, weekly reports, and gate reviews must come from trials run under this protocol. Do not change scoring rules after new demonstration data for a condition has been collected unless you bump the protocol version and note the change in `reports/`.

---

## 1. Task

**One sentence:** Pick up the cube from a marked start position and place it into the bowl.

| Region | Definition |
|--------|------------|
| **Start** | Cube starts on one of **10** table positions marked with tape and numbered **1–10**. Overhead photo: `docs/img/init_positions.jpg`. These positions are the reproducible operationalization of varied / “random” starts. |
| **Goal** | Fixed **bowl** on the table. |

Training and evaluation for this protocol use the **bowl** as the goal (not a box). Any earlier recording notes that use a box are superseded for paper numbers.

---

## 2. Training conditions

| Rule | Value |
|------|--------|
| Demonstrations per training condition | **50** |
| Coverage | **5** demos at each of positions **1–10** |
| Cameras / mount | Training cameras rigidly mounted; mount bases outlined with tape (Pose A — see §6) |

Each Hugging Face dataset card lists episode counts, cameras, camera pose, and collector per episode range.

---

## 3. Evaluation design

| Rule | Value |
|------|--------|
| Initial positions | Same **1–10** as training |
| Trials per condition | **20** = each position tested **twice** |
| Policies (real, Week 2+) | ACT and SmolVLA under identical trial rules |
| Domains | `real` and `sim` use the **same** trial counts and success definition |

**RQ2 note:** Compare the **trend** (how success falls as the camera moves) between sim and real, not absolute success rates. A policy trained on real images is not expected to run well on rendered images.

A **condition** is a unique tuple of: policy + checkpoint + train data + camera pose + domain + robot.

---

## 4. Success and failure

### Success (score `success = 1`)

The cube is **released by the policy** and comes to rest **fully inside the bowl** (not balanced on the rim, not outside).

### No time limit

There is **no trial time limit**. A trial ends when the operator scores success or failure (clear terminal failure or safety stop). This overrides the research plan’s generic “within the time limit” wording for this project.

### Automatic failure (score `success = 0`)

- Operator **safety stop** / e-stop / forced stop
- Any **human touch** or teleop assist during the trial
- Cube never grasped, dropped in transit, or released outside / on the rim of the bowl

### Failure modes (`failure_mode` column)

Score the event that ended the trial’s chance of success:

| Code | Meaning |
|------|---------|
| `no-grasp` | Cube never lifted |
| `grasped-dropped` | Grasped then lost in transit |
| `wrong-placement` | Released outside the bowl or on the rim |
| `safety-stop` | Operator stopped the trial for safety |
| `other` | Anything else (explain in `notes`) |

On success, leave `failure_mode` empty or use `-`.

---

## 5. Checkpoint rule

1. Choose the checkpoint **before** any evaluation trials for that condition (fixed training step **or** `last`).
2. Record the chosen step in this file (table below) and in each `results.csv` row.
3. **Never** choose or swap a checkpoint after looking at test success rates.

### Locked checkpoints (fill before first eval of that policy)

| Policy | Checkpoint step / tag | Train data (HF id) | Notes |
|--------|------------------------|--------------------|-------|
| ACT | _TBD before first ACT eval_ | _TBD_ | |
| SmolVLA | _TBD before first SmolVLA eval_ | _TBD_ | |

---

## 6. Camera protocol

| Rule | Detail |
|------|--------|
| Mount | Rigid; base outlined with tape |
| Pose A | Training / reference pose |
| Recording a pose | Position from arm base: **x, y, z in cm**; angles: **yaw, pitch in degrees**; plus a photo |
| Registry | Measurements and photos live in `docs/camera_poses.md` and `docs/img/poseA_*.jpg` (later: pose B, C, …) |

### Start of every collection or eval session

1. Capture one frame from the evaluation camera.
2. Compare it to the Pose A reference photo.
3. If they do not match, **fix the mount** before collecting or testing anything.

---

## 7. Logging

### `results/results.csv`

One row per trial. Columns (exact order):

```text
date,domain,robot,policy,checkpoint,train_data,camera_pose,init_pos_id,trial,success,failure_mode,operator,video_file,notes
```

| Column | Values |
|--------|--------|
| `date` | ISO date `YYYY-MM-DD` |
| `domain` | `real` or `sim` |
| `robot` | `so101` or `bigarm` |
| `policy` | e.g. `act`, `smolvla` |
| `checkpoint` | Step or tag chosen under §5 |
| `train_data` | HF dataset id |
| `camera_pose` | e.g. `A`, `B`, `C`, `D` |
| `init_pos_id` | `1` … `10` |
| `trial` | `1` or `2` for that position |
| `success` | `0` or `1` |
| `failure_mode` | See §4 |
| `operator` | Who ran the trial |
| `video_file` | Relative path under `videos/` |
| `notes` | Free text |

### Video naming

```text
videos/WXX/{policy}_pose{POSE}_pos{NN}_t{N}.mp4
```

Example: `videos/W02/act_poseA_pos03_t1.mp4` (week, policy, pose, initial position, trial).

Every `results.csv` row must have a matching video file.

---

## 8. Gate 1 (ACT threshold)

**ACT success threshold at Pose A for Gate 1:**  
`TBD — set by Dr. Liu before the first evaluation and written here.`

Week 1 deliverable is complete with this line still TBD. Do not run Gate 1 scoring until the number is filled in.

---

## 9. Out of scope for this file

| Topic | Where it lives |
|-------|----------------|
| Pose B/C/D measurements and photos | `docs/camera_poses.md` |
| Commands from raw data → eval | `docs/runbook_act.md`, `docs/runbook_smolvla.md` |
| Large-arm procedure | `docs/runbook_bigarm.md` |

---

## 10. Alignment notes

- Paper eval uses **10** taped positions and **20** trials per condition, not the exploratory 1–50 sheet or the July 2025 Fixed/Random 50-trial tabs.
- If a printed sheet’s labels 1–10 coincide with the taped marks, they may map 1:1; otherwise re-mark positions 1–10 to match the demonstrations used for training.
- Sim evaluations (Week 5+) follow §§3–7 with `domain=sim` and the same 10 initial positions in the digital twin.

---

## Review checklist (Ayush — Task 1.5)

Flag anything unclear before the protocol is treated as frozen:

- [ ] Task / bowl success definition is unambiguous in the lab
- [ ] Positions 1–10 are marked and photographed
- [ ] No-time-limit ending rule is agreed (when to stop a stuck trial → `safety-stop` or `other`)
- [ ] CSV columns and video naming match how we will actually record
- [ ] Gate 1 threshold still TBD (expected until Dr. Liu sets it)
