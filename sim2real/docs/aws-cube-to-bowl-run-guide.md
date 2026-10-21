# How to Run: AWSBuilderCube Teleop (Isaac Sim 6.0.1, no Isaac Lab)

Quick step-by-step for running the two raw-Isaac-Sim teleop scripts against
`source/sim_to_real_so101/demo/real-to-sim.usd`. For the full technical
background (why these exist, physics fixes, verification history), see
[aws-cube-to-bowl-teleop-plan.md](aws-cube-to-bowl-teleop-plan.md).

Both scripts run on a plain Isaac Sim 6.0.1 Python (`C:\Isaac-Sim\python.bat`)
with **no Isaac Lab installed** — always launch by direct file path from the
repo root, never `-m`.

---

## Option A — Keyboard jog (no hardware needed)

```powershell
C:\Isaac-Sim\python.bat source\sim_to_real_so101\scripts\keyboard_agent_raw_isaacsim.py
```

Run from the repo root. Boots in ~30-45s. Click into the viewport once it
opens — it needs focus to receive key events. If the robot isn't visible,
select `SO_ARM101_USD` in the Stage outliner and press `F` to frame it.

**Controls** (hold to move, release to stop):

| Joint | Increase (+) | Decrease (−) |
|---|---|---|
| Rotation (base yaw) | `D` | `A` |
| Pitch (shoulder) | `W` | `X` |
| Elbow | `E` | `V` |
| Wrist_Pitch | `T` | `G` |
| Wrist_Roll | `Y` | `H` |
| Jaw (gripper) | `U` | `J` |

`R` resets all joint targets to 0° **and** snaps the cube/bowl back to their
starting positions.

**Grasp walkthrough**: press `U` to open the jaw (it starts nearly closed),
jog `D/A W/X E/V T/G` down to the cube (world pos roughly `(0, 0.03, 0.775)`,
robot base at `(0, 0.3, 0.72)`), press `J` to close, then `W`/`T` to lift.

Add `--headless` for no viewport, `--joint_step <degrees>` to change jog
speed (default `0.5°`/tick).

---

## Option B — Real leader arm

### One-time setup (already done on this machine, keep for reference)

The `lerobot` package (0.6.1, at this repo's root, one level above `sim2real/`)
needs installing into Isaac Sim's own Python,
pinned to match Isaac Sim's bundled torch/torchvision exactly (mismatched
versions crash Kit's native extensions):

```powershell
cd ..  # repo root (lerobot 0.6.1)
C:\Isaac-Sim\python.bat -m pip install -e ".[so101]"
C:\Isaac-Sim\python.bat -m pip install "torch==2.11.0+cu128" "torchvision==0.26.0+cu128" --index-url https://download.pytorch.org/whl/cu128
```

Leader-arm calibration lives in this repo at
`calibration/teleoperators/so_leader/my_so_arm.json` — the script points
`HF_LEROBOT_CALIBRATION` there itself, no manual env var needed.

### Run

```powershell
C:\Isaac-Sim\python.bat source\sim_to_real_so101\scripts\leader_arm_teleop_raw_isaacsim.py
```

Run from the repo root. Same boot time as Option A. The sim gripper tracks
the physical leader arm live once you see:

```
[INFO]: Leader arm connected: port=COM4 id=my_so_arm
[INFO]: Driving joints from the leader arm. Ctrl+C or close the window to stop.
```

Defaults to port `COM4`, id `my_so_arm` — override with `--port` / `--robot_id`
if your setup differs.

`R` (keyboard, needs viewport focus) resets the cube/bowl to their starting
positions — joint targets aren't reset here since they're driven live by the
physical arm every tick, not held state.

**Grasp walkthrough**: open the gripper, position it over the cube in the
viewport (same world position as above), close, lift, then try some
rotation/lateral motion while holding — that's the real test for slipping,
not just a straight vertical lift.

---

## Fixed-layout real-to-sim mirroring (`--layout`)

`leader_arm_teleop_raw_isaacsim.py` places the cube and bowl using a
**fixed-layout config**, not live tracking: you physically place the robot,
cube, and bowl in known, repeatable positions on the table, measure them
once, and the script converts that measurement into Isaac Sim world poses at
startup. **There is no live real-world object tracking** — if the physical
cube, bowl, table, or robot base moves relative to the fixture, re-measure
and update the layout. (This intentionally replaces the ArUco/camera-tracking
approach considered in `docs/object-pose-mirroring-plan.md` — see that file
for why it was dropped in favor of a one-time manual measurement.)

### Coordinate frame

Poses are expressed **relative to the SO-101 base mounting point** (frame
name `so101_base`), not raw Isaac world coordinates, and converted to world
poses at startup:

```
T_world_cube = T_world_base @ T_base_cube
T_world_bowl = T_world_base @ T_base_bowl
```

- **Origin**: the SO-101's mounting point on the table (`robot_world.xyz_m`
  in the layout JSON).
- **+Z**: up (matches the table-height convention, e.g.
  `table.surface_z_world_m`).
- **+X / +Y**: same directions as the world/Isaac stage axes when
  `robot_world.yaw_deg == 0`.
- **Yaw**: degrees, right-handed rotation about +Z (CCW looking down from
  above). Roll/pitch are always zero — this fixture is a flat tabletop.
- **Cube position = its center.** **Bowl position = its table-contact point**
  (the bottom of its footprint) — these are *not* the same convention, don't
  mix them up when measuring.

Full derivation and the module-level API live in
`source/sim_to_real_so101/utils/fixed_workspace.py` (pure Python, no
isaacsim/omni import — unit-tested independently of Isaac Sim, see
`tests/test_fixed_workspace.py`).

### Physically measuring your fixture

1. Screw down / clamp the SO-101 base, the table, the cube's resting spot,
   and the bowl's resting spot so they don't shift between sessions.
2. Measure the robot base's world position (`robot_world.xyz_m`) and the
   table surface height (`table.surface_z_world_m`) — same as the existing
   `ROBOT_POS`/table-height constants this file used to hardcode.
3. With the cube resting at its normal spot, measure its **center** position
   relative to the robot base origin (subtract `robot_world.xyz_m` from a
   world-frame tape measurement, or measure directly base-relative) ->
   `cube.xyz_base_m`. Alternatively, set `cube.rest_on_table: true` and leave
   `xyz_base_m[2]` at `0.0` — Z is then derived as
   `table.surface_z_world_m + cube.size_m[2] / 2` instead of measured
   directly (don't set both an explicit nonzero Z *and* `rest_on_table:
   true` — the script rejects that as ambiguous).
4. Same for the bowl's **table-contact point** -> `bowl.xyz_base_m`.
5. Measure the real cube's actual side length (expected ~0.053m / 5.3cm, not the
   0.05m originally authored in `real-to-sim.usd`) -> `cube.size_m`.
6. Fill in `calibration/workspaces/aws_cube_bowl_fixed.json` (or a copy of
   it) with these numbers. The checked-in file's values are a **pure
   coordinate-transform of this repo's previously-measured world-frame
   constants** (not a new/independent measurement) — treat it as a starting
   template whose numbers you should re-verify against your own physical
   setup, not as ground truth for a different setup.

### Schema

```json
{
  "version": 1,
  "frame": "so101_base",
  "robot_world": { "xyz_m": [0.0, 0.3, 0.72], "yaw_deg": 0.0 },
  "table": { "surface_z_world_m": 0.7504 },
  "cube": {
    "xyz_base_m": [0.0, -0.253, 0.0554],
    "yaw_deg": 0.0,
    "size_m": [0.053, 0.053, 0.053],
    "mass_kg": 0.05,
    "rest_on_table": false
  },
  "bowl": { "xyz_base_m": [0.18, -0.253, 0.03], "yaw_deg": 0.0 }
}
```

`version`/`frame` are validated (must be `1`/`"so101_base"`); every `xyz_m`/
`xyz_base_m` must be exactly 3 finite numbers; `size_m` components and
`mass_kg` must be `> 0`; a bad layout fails **before** Kit finishes booting,
with a clear `[ERROR]` message, not a silent fallback.

`robot_world.yaw_deg` is validated and factored into the cube/bowl base-frame
math, but is **not yet applied to the simulated robot's physical mount**
(known limitation — see the receipt doc). Leave it at `0.0` unless you're
prepared to verify the mismatch yourself.

### CLI

```powershell
C:\Isaac-Sim\python.bat source\sim_to_real_so101\scripts\leader_arm_teleop_raw_isaacsim.py `
    --layout calibration\workspaces\aws_cube_bowl_fixed.json
```

- `--layout <path>`: use a layout JSON. Omit it to fall back to an in-code
  legacy layout that reproduces this file's old hardcoded world-frame
  constants exactly (today's default behavior, unchanged).
- `--cube_yaw_deg` / `--bowl_yaw_deg`: override just the cube/bowl yaw from
  whichever layout is in effect.
- `--cube_pos x,y,z` / `--bowl_pos x,y,z`: **unchanged from before** —
  absolute world-frame overrides that bypass the base-frame math entirely.

Precedence, most to least specific: **CLI override > `--layout` JSON >
in-code legacy default.**

### What `R` does now

Pressing `R` restores the cube's and bowl's **configured initial pose**
(translate *and* orientation, both now included in the snapshot/restore) and
zeroes the cube's linear/angular velocity. The cube stays a dynamic rigid
body throughout — its pose is only ever set at startup and on `R`, never
written every tick.

### Manual verification sequence (no automated substitute for this)

1. Place the robot, cube, and bowl in the fixture per your measurements.
2. Launch sim-only (no `--follower_port`):
   ```powershell
   C:\Isaac-Sim\python.bat source\sim_to_real_so101\scripts\leader_arm_teleop_raw_isaacsim.py --layout calibration\workspaces\aws_cube_bowl_fixed.json
   ```
3. Visually compare the sim cube's position/orientation against the real cube.
4. Visually compare the sim bowl's position against the real bowl.
5. Jog the simulated robot through the three checkpoints below.
6. Press `R`.
7. Confirm the cube/bowl snap back to exactly their startup pose (position
   *and* orientation), and the cube isn't carrying leftover velocity/spin.
8. **Only then**, if you also want the real follower mirrored, add
   `--follower_port COM3` (see Option B above) — never as part of an
   automated check.

### Alignment checkpoints

- **Checkpoint A**: gripper centered directly above the cube.
- **Checkpoint B**: gripper closed around the cube, before lifting.
- **Checkpoint C**: gripper centered directly above the bowl.

If real and sim disagree noticeably at any checkpoint, investigate **in this
order** (cheapest/most-likely first):

1. Coordinate frame (origin, axis directions, yaw sign — re-read the
   convention above).
2. Robot base transform (`robot_world.xyz_m`/`yaw_deg`).
3. Object measurement (`cube.xyz_base_m` / `bowl.xyz_base_m`).
4. Object scale (`cube.size_m` — watch for a stale 0.05m assumption).
5. Joint calibration (leader-arm calibration file, not this layout).
6. Only then, friction/contact tuning (`AWS_CUBE_STATIC_FRICTION` etc.).

### Limitations

- No live tracking — a moved fixture needs a re-measurement, not a
  re-launch.
- `robot_world.yaw_deg` isn't wired into the robot's physical mount yet
  (translation is; see the receipt doc).
- This is a demo-oriented fixed-pose system, not general object-pose
  perception — it deliberately does not use ArUco/AprilTags/camera tracking.

---

## Troubleshooting

- **Nothing happens on key press**: click into the viewport window first.
- **Boot looks stuck / high GPU use, no window**: if launched from an
  automated/agent terminal rather than an interactive one, the window may
  never actually attach to the desktop. Run from a real interactive terminal.
- **Leader arm won't connect**: check the physical arm is on the port you
  passed (`--port`, default `COM4`) and that `calibration/teleoperators/so_leader/`
  has a file matching `--robot_id` (default `my_so_arm`).
- **Cube launches away instead of being held**: already fixed (Jaw
  `effort_limit` was 30, way more torque than a 50g cube needs against an
  unreachable "fully closed" target — reduced to 3). If it's still happening,
  say so — there's more room to tune.
