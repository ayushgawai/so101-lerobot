# Environment Setup & Reproduction Guide v0.1

**Project:** SO-101 Robot Arm — LeRobot Fork  
**Repo:** https://github.com/ayushgawai/so101-lerobot  
**Platform:** Windows 10/11 (PowerShell) · Python 3.12+  
**Version:** 0.1 · June 2026

---

## 1. Purpose

This guide reproduces the full SO-101 setup from a clean machine through working teleoperation with cameras. It covers:

- Environment install
- Motor ID assignment and EEPROM fix
- Per-arm calibration
- Camera setup
- Teleoperation and dataset recording
- Planned next steps (training / pretrained inference)

---

## 2. What This Repo Adds

| Item | Description |
|------|-------------|
| `src/lerobot/motors/motors_bus.py` | Patched: writes `Lock=1` after ID/baud changes so EEPROM commits |
| `fix_motor_ids.py` | Interactive fix for wrist_roll (ID 5) and gripper (ID 6) |
| `scan_motor_ids.py` | Non-destructive bus scan — lists responding motor IDs |
| `calibration/` | Committed calibration JSON for `my_so_arm` (follower + leader) |
| `README.md` | Windows step-by-step reference |
| `so-arm-guide.md` | Extended reference (originally macOS paths) |

---

## 3. Hardware Requirements

| Component | Details |
|-----------|---------|
| Follower arm | SO-101 robot being controlled |
| Leader arm | SO-101 teleoperator (moved by hand) |
| Servos | 6× Feetech STS3215 per arm |
| USB adapters | 1 per arm (Waveshare or similar) |
| Power | 5–7.4 V per arm |
| Cameras (optional) | 2× USB — gripper-mounted + top/overview |
| PC | Windows 10/11 with USB ports |

### Motor ID Map (both arms)

| Joint | ID |
|-------|-----|
| shoulder_pan | 1 |
| shoulder_lift | 2 |
| elbow_flex | 3 |
| wrist_flex | 4 |
| wrist_roll | 5 |
| gripper | 6 |

On the **leader**, joint 6 is the **trigger handle** (still named `gripper` in code).

---

## 4. Software Prerequisites

1. **Python 3.12+** — https://www.python.org/downloads/ (check "Add python.exe to PATH")
2. **uv** package manager:

   ```powershell
   powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
   ```

3. **Git** — https://git-scm.com/download/win
4. **USB serial driver** — CH340 or CP210x if ports don't appear in Device Manager
5. **Hugging Face account** (for recording/training) — https://huggingface.co

---

## 5. Step 0 — Clone and Install

```powershell
cd $env:USERPROFILE\Desktop
git clone https://github.com/ayushgawai/so101-lerobot.git
cd so101-lerobot
uv sync --extra feetech --extra viz --extra dataset
```

**Extras installed:**

| Extra | Purpose |
|-------|---------|
| `feetech` | Feetech servo SDK + pyserial |
| `viz` | Rerun viewer for live camera display |
| `dataset` | HF datasets, video encoding for recording |

**CUDA note:** `pyproject.toml` pins CUDA PyTorch. For CPU-only, comment out the `pytorch-cu130` index blocks before `uv sync`.

**Verify install:**

```powershell
.\.venv\Scripts\Activate.ps1
lerobot-find-port --help
```

---

## 6. Step 1 — Activate Environment (every new terminal)

```powershell
cd $env:USERPROFILE\Desktop\so101-lerobot
.\.venv\Scripts\Activate.ps1
```

If blocked:

```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```

---

## 7. Step 2 — Find Serial Ports

Ports change after reboot or USB socket swap. **Label each cable.**

### Method A — `lerobot-find-port` (recommended)

Plug in **one** arm (USB + motor power):

```powershell
lerobot-find-port
```

1. Note listed ports
2. Unplug **only that arm's USB**, press Enter
3. Script prints the port for that arm
4. Replug and repeat for the other arm

### Method B — Device Manager

Win + X → Device Manager → Ports (COM & LPT) → unplug/replug to see which COM appears.

### Known ports (verify on your machine)

| Arm | Windows | macOS (original setup) |
|-----|---------|------------------------|
| Follower | `COM3` | `/dev/cu.usbmodem5B3E1225231` |
| Leader | `COM4` | `/dev/cu.usbmodem5B3E1218771` |

---

## 8. Step 3 — Scan Motor IDs (sanity check)

With all six motors powered and only **one arm's USB** connected:

```powershell
python scan_motor_ids.py --port COM3   # follower
python scan_motor_ids.py --port COM4   # leader
```

**Expected:** IDs `1` through `6`, ending with `OK: all expected IDs 1..6 present.`

**If IDs 5 or 6 are missing** → see `troubleshooting-gripper-motor-id.md`.

---

## 9. Step 4 — Set Up Motor IDs (first time only)

Connect **one motor at a time** when prompted. Order: gripper (6) → wrist_roll (5) → … → shoulder_pan (1).

```powershell
# Follower
lerobot-setup-motors --robot.type=so101_follower --robot.port=COM3 --robot.id=my_so_arm

# Leader
lerobot-setup-motors --teleop.type=so101_leader --teleop.port=COM4 --teleop.id=my_so_arm
```

The patched `motors_bus.py` writes `Lock=1` after each ID assignment so IDs survive reboot.

**Alternative fix script** (if IDs 5/6 still revert):

```powershell
python fix_motor_ids.py --port COM3
python fix_motor_ids.py --port COM4
```

---

## 10. Step 5 — Calibrate Each Arm Separately

**Only plug in the arm being calibrated** (one USB cable).

Calibration files:

- `./calibration/robots/so_follower/my_so_arm.json`
- `./calibration/teleoperators/so_leader/my_so_arm.json`

```powershell
# Follower
lerobot-calibrate --robot.type=so101_follower --robot.port=COM3 --robot.id=my_so_arm

# Leader
lerobot-calibrate --teleop.type=so101_leader --teleop.port=COM4 --teleop.id=my_so_arm
```

**During calibration:**

1. Move arm to **middle** of each joint's range → press Enter
2. Move **all six joints** through full range slowly (including wrist_roll)
3. Open/close gripper (follower) or trigger (leader) fully
4. Press Enter to save

**Verify:** Re-run calibrate; it should connect to all 6 motors without `Missing motor IDs` errors.

---

## 11. Step 6 — Camera Setup

### Find indices

```powershell
lerobot-find-cameras
```

Or quick OpenCV scan:

```powershell
python -c "import cv2
for i in range(5):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        ret, _ = cap.read()
        print(f'Camera {i}:', 'OK' if ret else 'no frame')
        cap.release()"
```

### Verified mapping

| Name | Index | Role |
|------|-------|------|
| gripper_cam | 0 | Wrist-mounted, faces workspace |
| top_cam | 1 | Fixed above table, full workspace |

### Top camera positioning

- Height: 40–60 cm above table
- Angle: ~30–45° down
- Faces the **follower arm + workspace** (spectator view, not same direction as gripper cam)
- Must be **rigidly mounted** — no wobble during recording
- Avoid seeing leader arm, hands, or cables

### Live preview (press Q to quit)

```powershell
python -c "import cv2
cap0, cap1 = cv2.VideoCapture(0), cv2.VideoCapture(1)
while True:
    r0, f0 = cap0.read(); r1, f1 = cap1.read()
    if r0: cv2.imshow('Gripper Cam', f0)
    if r1: cv2.imshow('Top Cam', f1)
    if cv2.waitKey(1) & 0xFF == ord('q'): break
cap0.release(); cap1.release(); cv2.destroyAllWindows()"
```

---

## 12. Step 7 — Teleoperate (reproduction checkpoint)

Plug in **both** arms. Leader mirrors to follower.

### Without cameras

```powershell
lerobot-teleoperate `
  --robot.type=so101_follower `
  --robot.port=COM3 `
  --robot.id=my_so_arm `
  --robot.calibration_dir=./calibration/robots/so_follower `
  --teleop.type=so101_leader `
  --teleop.port=COM4 `
  --teleop.id=my_so_arm `
  --teleop.calibration_dir=./calibration/teleoperators/so_leader
```

### With cameras + Rerun viewer

```powershell
lerobot-teleoperate `
  --robot.type=so101_follower `
  --robot.port=COM3 `
  --robot.id=my_so_arm `
  --robot.calibration_dir=./calibration/robots/so_follower `
  --robot.cameras="{gripper_cam: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, top_cam: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" `
  --teleop.type=so101_leader `
  --teleop.port=COM4 `
  --teleop.id=my_so_arm `
  --teleop.calibration_dir=./calibration/teleoperators/so_leader `
  --fps=30 `
  --display_data=true `
  --display_compressed_images=true
```

`--fps=30` matches the cameras (no duplicated frames); compressed images stop Rerun hitting its memory limit.

**Verify:** Move leader → follower mirrors all 6 joints including gripper/trigger. Ctrl+C to stop.

**Camera preview workaround:** Rerun may not auto-open. Use the OpenCV preview script in a second terminal while teleoperate runs.

---

## 13. Step 8 — Record Dataset (optional)

One-time HF login:

```powershell
huggingface-cli login
```

```powershell
lerobot-record `
  --robot.type=so101_follower `
  --robot.port=COM3 `
  --robot.id=my_so_arm `
  --robot.calibration_dir=./calibration/robots/so_follower `
  --robot.cameras="{gripper_cam: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, top_cam: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" `
  --teleop.type=so101_leader `
  --teleop.port=COM4 `
  --teleop.id=my_so_arm `
  --teleop.calibration_dir=./calibration/teleoperators/so_leader `
  --dataset.repo_id=YOUR_USERNAME/so101-pick-cube `
  --dataset.num_episodes=30 `
  --dataset.single_task="Pick up the cube and place it in the bowl"
```

- **Space** — start/stop each episode
- **Ctrl+C** — finish early

**Windows note:** Video encoding may fail without `torchcodec`. Test recording without cameras first if it errors.

---

## 14. Step 9 — Training & Inference (planned)

Not yet run end-to-end in this project. Planned workflow:

### Training (M1 Mac ~1.5 hr; NVIDIA GPU on Windows)

```powershell
uv sync --extra training

lerobot-train `
  --dataset.repo_id=YOUR_USERNAME/so101-pick-cube `
  --policy.type=act `
  --output_dir=outputs/train/my_so101_act `
  --job_name=so101_training `
  --policy.device=cuda `
  --policy.repo_id=YOUR_USERNAME/my_so101_act
```

### Inference (policy drives follower)

```powershell
lerobot-record `
  --robot.type=so101_follower `
  --robot.port=COM3 `
  --robot.id=my_so_arm `
  --robot.calibration_dir=./calibration/robots/so_follower `
  --robot.cameras="{gripper_cam: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, top_cam: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" `
  --dataset.repo_id=YOUR_USERNAME/eval_so101 `
  --policy.path=YOUR_USERNAME/my_so101_act `
  --dataset.num_episodes=10
```

---

## 15. Reproduction Checklist

| # | Step | Pass criteria |
|---|------|---------------|
| 1 | `uv sync` + activate venv | `lerobot-find-port` runs |
| 2 | Find COM ports | Follower + leader ports identified |
| 3 | `scan_motor_ids.py` | All IDs 1–6 on both arms |
| 4 | Calibrate both arms | No `Missing motor IDs` error |
| 5 | Teleoperate (no cameras) | Follower mirrors leader, 6 DOF |
| 6 | Camera preview | Both feeds visible at indices 0 and 1 |
| 7 | Teleoperate + cameras | Connects at ~60 Hz, no crash |
| 8 | Record 1 test episode | Episode saves to HF (optional) |

---

## 16. Quick Reference

```powershell
.\.venv\Scripts\Activate.ps1
python scan_motor_ids.py --port COM3
lerobot-calibrate --robot.type=so101_follower --robot.port=COM3 --robot.id=my_so_arm
lerobot-teleoperate --robot.type=so101_follower --robot.port=COM3 --robot.id=my_so_arm --robot.calibration_dir=./calibration/robots/so_follower --teleop.type=so101_leader --teleop.port=COM4 --teleop.id=my_so_arm --teleop.calibration_dir=./calibration/teleoperators/so_leader --display_data=true
```

---

## 17. References

- Repo: https://github.com/ayushgawai/so101-lerobot
- Upstream LeRobot: https://github.com/huggingface/lerobot
- LeRobot IL docs: https://huggingface.co/docs/lerobot/il_robots
- SO-101 assembly: `src/lerobot/robots/so_follower/so101.md`
- Gripper troubleshooting: `troubleshooting-gripper-motor-id.md`
