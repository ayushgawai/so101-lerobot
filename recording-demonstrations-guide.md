# Recording Demonstrations Guide (SO-101 + LeRobot, Windows)

How to record teleoperation demonstrations with `lerobot-record`, plus the issues we hit on Windows and their fixes.

---

## Prerequisites

- Robot (follower) and leader arm connected and calibrated.
- Calibration files present:
  - `./calibration/robots/so_follower`
  - `./calibration/teleoperators/so_leader`
- Cameras connected (verify indices with `lerobot-find-cameras`).
- Correct COM ports (verify with `lerobot-find-port`). In this setup: follower = `COM3`, leader = `COM4`.
- Logged into Hugging Face (see below).

### Hugging Face login

`huggingface-cli` is deprecated and not on PATH. Use the venv `hf` tool:

```powershell
.\.venv\Scripts\hf.exe auth login
```

- Paste a token with **Write** role from https://huggingface.co/settings/tokens
- The token input is **invisible** (no characters shown) — paste and press Enter.
- When asked `Add token as git credential?` → `n`.

Verify:

```powershell
.\.venv\Scripts\hf.exe auth whoami
```

---

## Recommended wrappers

Both pin the settings every session must share with eval: loop, dataset and cameras at 30 Hz, the
same camera names/indices/640x480/MJPG as `run_eval.ps1`, h264, and compressed Rerun images.

- **`scripts/run_record.ps1`**: cube placed anywhere (no position ids). Per episode: Enter to
  start (the follower first slides to the leader pose, unrecorded), Right = save, Left = discard &
  redo, Esc = stop. No time limit, no reset phase, one task string per dataset.
- **`scripts/record_pickplace.ps1`**: fixed sheet positions 1-50 (position stored in the task text).

```powershell
.\scripts\run_record.ps1 -NumEpisodes 50 -TeleopTest     # aakashv100/so101-pick-place-random, appends
.\scripts\record_pickplace.ps1 -TeleopTest               # position blocks
```

Keep the two in separate datasets: their task strings differ, which matters for SmolVLA.

The raw commands below still work but don't set MJPG or compressed display.

## Sanity test (2 episodes)

Always run a 2-episode test before a full 30-episode session to confirm cameras, encoding, and Hub upload all work.

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
  --dataset.repo_id=aakashv100/so101-pick-cube-test `
  --dataset.num_episodes=2 `
  --dataset.episode_time_s=45 `
  --dataset.reset_time_s=15 `
  --dataset.rgb_encoder.vcodec=h264 `
  --dataset.no_stamp=true `
  --display_data=true `
  --dataset.single_task="Pick up the cube and place it in the bowl"
```

## Full run (30 episodes)

After the sanity test passes, switch the repo id and episode count:

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
  --dataset.repo_id=aakashv100/so101-pick-cube `
  --dataset.num_episodes=30 `
  --dataset.episode_time_s=45 `
  --dataset.reset_time_s=15 `
  --dataset.rgb_encoder.vcodec=h264 `
  --dataset.no_stamp=true `
  --display_data=true `
  --dataset.single_task="Pick up the cube and place it in the bowl"
```

### Key flags

| Flag | Purpose |
|------|---------|
| `--dataset.num_episodes` | Number of demonstrations to record |
| `--dataset.episode_time_s=45` | Max seconds per episode (press → to end early) |
| `--dataset.reset_time_s=15` | Idle seconds between episodes to reset the scene |
| `--dataset.rgb_encoder.vcodec=h264` | **Required on Windows** — the default `libsvtav1` encoder crashes (LeRobot ≥0.6.1 name; was `--dataset.vcodec`) |
| `--dataset.no_stamp=true` | Keep `repo_id` as typed — LeRobot ≥0.6.1 otherwise appends a timestamp |
| `--display_data=true` | **Required to see the camera feed** (Rerun viewer); off by default |
| `--dataset.single_task` | Plain-text task description (should match what you demonstrate) |

---

## Keyboard controls during recording

| Key | Action |
|-----|--------|
| **Right Arrow (→)** | End current episode / reset early, move to next |
| **Left Arrow (←)** | Discard current episode and re-record it |
| **Escape (Esc)** | Stop recording, save, and exit cleanly |
| **Ctrl+C** | Hard abort (last resort) |

> Note: the README mentions Space, but this build uses arrow keys (`src/lerobot/common/control_utils.py`).

## Recording workflow

1. **Connect phase** — robot + leader arm connect; cameras warm up; Rerun viewer opens.
2. **Episode records** — teleoperate the leader to perform the task. Press **→** when done.
3. **Encoding** — the episode video encodes immediately (in-process).
4. **Reset phase** — `Reset the environment` is logged/spoken; move the cube back; press **→** to skip ahead. (Skipped after the final episode.)
5. **Repeat** until all episodes are done.
6. **Upload** — dataset pushes to the Hub repo.

## Task design

- Keep **one episode = one direction** (pick cube → place in bowl). Do **not** record round trips.
- Use the reset phase to manually return the cube to start.
- The `single_task` string should match the demonstration (policies condition on it).

---

## Issues & troubleshooting

### 1. `FileExistsError: Cannot create a file when that file already exists`

A previous (often failed) run left the local dataset folder behind; `create()` refuses to overwrite.

**Fix** — delete the local folder before re-running:

```powershell
Remove-Item -Recurse -Force "$env:USERPROFILE\.cache\huggingface\lerobot\aakashv100\so101-pick-cube-test" -ErrorAction SilentlyContinue
```

(Adjust the repo id in the path. Optionally delete the Hub repo too at its `/settings` page.)

### 2. `401 Unauthorized` on `push_to_hub`

Not logged into Hugging Face.

**Fix** — run `.\.venv\Scripts\hf.exe auth login` with a **Write** token (see [Hugging Face login](#hugging-face-login)).
To skip uploading entirely (local-only), add `--dataset.push_to_hub=false`.

### 3. Camera feed not visible

`display_data` defaults to `False`.

**Fix** — add `--display_data=true` (opens the Rerun viewer).
If the viewer opens but feeds are black/missing, it's a camera index problem — run `lerobot-find-cameras` and fix the indices.

### 4. `BrokenProcessPool` / `Video encoding failed` (default `libsvtav1`)

The default AV1 encoder (`libsvtav1`) crashes the encoding worker process on this Windows build.

**Fix** — use a different codec: `--dataset.rgb_encoder.vcodec=h264` (or try `hevc`).

### 5. `TypeError: 'NoneType' object is not subscriptable` in `_batch_save_episode_video`

Setting `--dataset.video_encoding_batch_size > 1` routes through a **buggy batch-encoding path** (`self._meta.episodes` is `None`, `dataset_writer.py:335`).

**Fix** — do **not** set `video_encoding_batch_size` (leave default 1), and force serial in-process encoding. We patched `src/lerobot/scripts/lerobot_record.py` so episodes encode one-at-a-time without the process pool:

```646:646:src/lerobot/scripts/lerobot_record.py
                dataset.save_episode(parallel_encoding=False)
```

This avoids both the process-pool crash (issue 4's pool path) and the batch bug.

### 6. `torchcodec not available, falling back to pyav`

Harmless warning on Windows — `pyav` is used for decoding instead. No action needed.

### 7. `ConnectionError` on exit (Ctrl+C)

The leader arm disconnecting mid-read. Harmless — ignore.

---

## Verifying a recorded dataset

- **Hub:** `https://huggingface.co/datasets/aakashv100/<dataset_name>`
- **Local:** `%USERPROFILE%\.cache\huggingface\lerobot\aakashv100\<dataset_name>`
- **Replay an episode** to confirm data quality:

```powershell
lerobot-replay `
  --robot.type=so101_follower `
  --robot.port=COM3 `
  --robot.id=my_so_arm `
  --robot.calibration_dir=./calibration/robots/so_follower `
  --dataset.repo_id=aakashv100/so101-pick-cube-test `
  --dataset.episode=0
```
