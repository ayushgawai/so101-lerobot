# Fixed-Layout Real-to-Sim Mirroring — Implementation Receipt

## Goal

Replace the unexplained, scattered Isaac-world-frame constants
(`REAL_CUBE_POS`/`REAL_BOWL_POS`/`ROBOT_POS`) in
`leader_arm_teleop_raw_isaacsim.py` with a fixed-layout config system: cube
and bowl poses measured once, expressed relative to the SO-101 base mounting
point, converted to Isaac world poses at startup — repeatable across
sessions without re-measuring or code edits, and without any camera/ArUco
tracking.

## Files changed

**Created:**
- `source/sim_to_real_so101/utils/fixed_workspace.py` — pure config/math
  (JSON loading, schema validation, base→world frame conversion, yaw math,
  `rest_on_table` Z derivation, CLI-override precedence). No
  isaacsim/omni/pxr import — unit-testable standalone.
- `calibration/workspaces/aws_cube_bowl_fixed.json` — the layout file,
  seeded with values derived from this repo's previously-measured
  world-frame constants (see "Final configured values" below — not a new
  physical measurement).
- `tests/test_fixed_workspace.py` — 40 tests covering base→world transform,
  yaw transform, layout parsing, invalid-schema rejection, CLI-override
  precedence, `rest_on_table` Z calculation, and legacy-default behavior.
  **Note:** this repo's `.gitignore` excludes `tests/` entirely (line 245 —
  a pre-existing convention; `tests/mdp/` and `tests/usd/` are equally
  untracked), so this file exists on disk and runs locally but won't show in
  `git status`/`git diff`.
- `docs/hardcoded-real2sim-implementation-receipt.md` — this file.

**Modified:**
- `source/sim_to_real_so101/scripts/leader_arm_teleop_raw_isaacsim.py` — the
  primary target file. Added `--layout`/`--cube_yaw_deg`/`--bowl_yaw_deg`
  CLI flags; replaced the hardcoded `REAL_CUBE_POS`/`REAL_BOWL_POS`/
  `ROBOT_POS`/`AWS_CUBE_MASS_KG` constants with layout resolution (validated
  and resolved before Kit boots); added cube geometry correction (5cm→5.6cm,
  applied as a runtime scale on the cube's existing mesh, not a USD edit);
  added cube/bowl yaw support via a safe translate+orient xformOp helper;
  added a `[LAYOUT]` startup summary and a table-contact consistency
  warning; added an opt-in follower-startup interpolation ramp.
- `source/sim_to_real_so101/utils/scene_reset.py` — added
  `ensure_translate_orient_ops()`, a small focused helper for safely
  reading/adding a prim's translate+orient xformOps without duplicating or
  reordering existing ones (used for both cube and bowl, which start with
  different authored ops — see "USD transform op findings" below).
- `docs/aws-cube-to-bowl-run-guide.md` — added a "Fixed-layout real-to-sim
  mirroring" section: coordinate-frame convention, measurement procedure,
  schema, CLI, manual verification sequence, alignment checkpoints,
  limitations.

**Not modified** (inspected only, per scope): `keyboard_agent_raw_isaacsim.py`,
`aws_cube_to_bowl_env_cfg.py`, `real-to-sim.usd` (no binary edits — the cube
geometry fix is applied at runtime, matching this script's existing
"patch on load, don't hand-edit the checked-in asset" convention for
RigidBodyAPI/friction/root-joint compensation).

## Architecture

```
calibration/workspaces/aws_cube_bowl_fixed.json  (or --layout <path>, or
                                                    neither -> legacy default)
                    |
                    v
   fixed_workspace.load_layout() / default_layout()   <- pure, validated,
                    |                                     runs before Kit boots
                    v
   fixed_workspace.resolve_cube_pose() / resolve_bowl_pose()
   (applies --cube_pos/--bowl_pos/--cube_yaw_deg/--bowl_yaw_deg CLI
    overrides on top of the layout, per precedence below)
                    |
                    v
   main(): open real-to-sim.usd, validate prims, apply mass/friction/
   geometry-scale/pose via ensure_translate_orient_ops(), snapshot pose,
   play timeline
                    |
                    v
   runtime loop: leader -> sim joints (+ optional leader -> real follower);
   PhysX owns the cube's pose every tick; 'R' restores the snapshot
```

**Precedence** (most to least specific): `--cube_pos`/`--bowl_pos`/
`--cube_yaw_deg`/`--bowl_yaw_deg` CLI overrides > `--layout` JSON >
in-code legacy default (`fixed_workspace.default_layout()`).

## Coordinate convention

`T_world_obj = T_world_base @ T_base_obj` (translation + yaw only, roll/pitch
always zero).

- **Frame**: `so101_base`. Origin = the SO-101's mounting point
  (`robot_world.xyz_m`).
- **+Z**: up. **+X/+Y**: match world axes when `robot_world.yaw_deg == 0`.
- **Yaw**: degrees, right-handed about +Z.
- **Cube position = center.** **Bowl position = table-contact point**
  (bottom of its footprint) — confirmed by direct inspection of
  `real-to-sim.usd`: the cube's mesh points are symmetric about its own
  origin (a true center), while the bowl's mesh extent starts at local z=0
  (its origin already sits at its resting/contact height).

Full docstring with derivation lives at the top of `fixed_workspace.py`.

## Configuration schema

See `calibration/workspaces/aws_cube_bowl_fixed.json` and the run guide's
"Schema" section. Validated fields: `version` (must be `1`), `frame` (must
be `"so101_base"`), `robot_world.xyz_m`/`yaw_deg`, `table.surface_z_world_m`,
`cube.xyz_base_m`/`yaw_deg`/`size_m`/`mass_kg`/`rest_on_table`,
`bowl.xyz_base_m`/`yaw_deg`. `cube.rest_on_table=true` together with a
nonzero `cube.xyz_base_m[2]` is rejected as ambiguous.

## Cube geometry findings

Direct USD inspection (`Usd.Stage.Open` against `real-to-sim.usd`, via
`pxr` loaded from Isaac Sim's `omni.usd.libs` extension directory without
booting Kit):

- `/World/AWSBuilderCube/Geometry/AWSBuilderCube_Geo` is a single `Mesh`
  prim serving as **both** the visual and the collision shape
  (`PhysicsCollisionAPI` + `PhysicsMeshCollisionAPI` applied directly to it,
  `physics:approximation = convexHull`).
- Its 8 mesh points sit at exactly ±0.025 on every axis — a **5.0cm cube**,
  confirmed authored, not just extent-cached.
- Real cube measures ~5.6cm (measured directly by the user 2026-08-20,
  superseding an earlier ~5.7cm estimate in
  `docs/object-pose-mirroring-plan.md`'s 2026-08-18 finding).
- **Fix applied**: at runtime, set this mesh's existing `xformOp:scale` to
  `cube.size_m[i] / 0.05` per axis. Since visual and collision are the same
  prim, this corrects both together — never just the visual mesh, never a
  mismatched collision size. Verified directly against the real asset (see
  Verification below): scaled half-width matches the configured size exactly.
- `/World/AWSBuilderCube` (the rigid-body root) originally authored **only**
  `xformOp:translate` — no orient op. `ensure_translate_orient_ops()` adds
  one (`AddOrientOp`, float precision) only when missing, appended after
  translate — verified this produces `xformOpOrder = [translate, orient]`
  (T·R composition: rotate about the cube's own origin, then translate to
  world position).
- `/World/PaperBowl` already authored `[translate, orient, scale]` —
  `ensure_translate_orient_ops()` reuses the existing orient op unchanged
  rather than adding a duplicate.

## Verification commands

```powershell
# Pure unit tests (no Isaac Sim needed)
usdenv\Scripts\python -m pytest tests\test_fixed_workspace.py -q

# Syntax check
usdenv\Scripts\python -m py_compile `
  source\sim_to_real_so101\scripts\leader_arm_teleop_raw_isaacsim.py `
  source\sim_to_real_so101\utils\fixed_workspace.py `
  source\sim_to_real_so101\utils\scene_reset.py

# --help / fail-fast CLI validation (exits before needing Isaac Sim at all,
# since argparse + fixed_workspace's validation both run before `from
# isaacsim import SimulationApp`)
usdenv\Scripts\python source\sim_to_real_so101\scripts\leader_arm_teleop_raw_isaacsim.py --help
usdenv\Scripts\python source\sim_to_real_so101\scripts\leader_arm_teleop_raw_isaacsim.py --layout does_not_exist.json
```

**Additional integration-level verification performed** (not a permanent
test file — exercised once during this implementation, using `pxr` loaded
from `C:\Isaac-Sim\extscache\omni.usd.libs-*` without booting Kit or
touching hardware): opened the real `real-to-sim.usd`, and replicated
`main()`'s exact cube/bowl mutation sequence (mass, geometry scale,
`ensure_translate_orient_ops`, pose set, snapshot/restore, velocity zeroing)
across four scenarios — legacy default, the checked-in layout JSON, a
layout+CLI-override combination, and a `rest_on_table=true` layout. All
assertions passed, including exact position+orientation restoration on
reset, correct geometry scale (`0.056 / 0.05 = 1.12`, scaled half-width
`0.028`), and correct `rest_on_table` Z derivation (that scenario used
`0.057` inline to demonstrate the formula generically; against the
checked-in `0.056` cube it would be `0.7504 + 0.056/2 = 0.7784`), gap
`0.00mm` in both cases.

## Verification results

- `pytest tests/test_fixed_workspace.py -q` → **40 passed**.
- `py_compile` on all three modified/new Python files → **clean**.
- `--help` → prints usage, exits 0, without needing Isaac Sim.
- Fail-fast checks tried and confirmed to exit 1 with a clear `[ERROR]`
  before Kit would boot: missing `--layout` file, `frame: "world"` (wrong
  frame), `--cube_yaw_deg nan`, malformed `--cube_pos "1,2"`.
- Integration-level `pxr`-only scenario script (see above): **all
  assertions passed** across 4 scenarios, including cube xformOpOrder
  becoming exactly `[translate, orient]`, bowl's pre-existing
  `[translate, orient, scale]` left untouched, cube mass/scale/pose set
  correctly, `R`-equivalent restore recovering exact position *and*
  orientation plus zeroed velocity, cube keeping `RigidBodyAPI` (dynamic)
  and bowl never getting one (static).

## Manual verification still required

The following need the physical leader arm (and optionally follower) and
**were not, and cannot safely be, automated**:

1. Launch `--layout calibration\workspaces\aws_cube_bowl_fixed.json`
   sim-only and visually confirm the sim cube/bowl match the real fixture.
2. Jog the simulated robot through Checkpoints A/B/C (gripper over cube,
   gripper closed on cube, gripper over bowl) and compare against the real
   arm's equivalent poses.
3. Press `R` mid-session and visually confirm the cube/bowl snap back
   exactly, with no residual drift/spin.
4. Only after 1–3 pass, add `--follower_port COM3` and confirm the follower
   startup ramp behaves reasonably (this build's ramp logic was verified by
   code inspection against the real `SO101Follower.get_observation()`/
   `send_action()` signatures, but not run against physical follower
   hardware in this session, per the "don't move real hardware during
   automated verification" constraint).

## Known limitations

- **No live object tracking.** If the physical cube, bowl, table, or robot
  base moves relative to the fixture, the layout must be re-measured —
  this is a fixed-layout system, not perception.
- **`robot_world.yaw_deg` is not yet wired into the robot's physical
  mount.** It's validated and factored into the cube/bowl base-frame
  math, but the `root_joint` compensation that positions the simulated
  robot itself still only applies `robot_world.xyz_m` (translation).
  Rotating the physical mount's `physics:localPos0`/`localRot0`
  compensation was deliberately deferred: this file's own history shows
  that root-joint frame mistakes produce a hard, highly visible failure
  ("found a joint with disjointed body transforms", robot snapping away
  from the table) — a much worse failure mode than a cube/bowl position
  being off, and not required by any acceptance criterion (only cube/bowl
  yaw was required). Left at `0.0` by default; set it and you'll get a
  `[WARN]` at startup.
- **Follower startup ramp is a mitigation, not a guarantee.** It reduces
  but does not eliminate the first-connect snap risk — still position the
  physical leader near the follower's actual pose before connecting.
- **`cube.rest_on_table`** only derives Z for the cube (center = table + half
  height); no equivalent was added for the bowl, since the bowl's own
  authored convention (position = contact point) already makes its Z
  trivially equal to the table height with no half-height math needed.

## Final configured values

(As currently set in `calibration/workspaces/aws_cube_bowl_fixed.json`. This
file has now diverged from `fixed_workspace.default_layout()` — the bowl
position was updated from a fresh 2026-08-20 measurement, while the legacy
Python fallback intentionally stays frozen at the old value for backward
compatibility. Cube pose/robot pose/table height are unchanged from the
legacy default so far.)

| Field | Value |
|---|---|
| robot world pose | `(0.0, 0.3, 0.72)` m, yaw `0.0°` |
| cube base-relative pose | `(0.0, -0.253, 0.0554)` m, yaw `0.0°` |
| cube world pose | `(0.0, 0.047, 0.7754)` m |
| cube dimensions | `0.056 × 0.056 × 0.056` m |
| cube mass | `0.05` kg |
| bowl base-relative pose | `(0.18, -0.253, 0.03)` m, yaw `0.0°` |
| bowl world pose | `(0.18, 0.047, 0.75)` m |
| table surface height | `0.7504` m (world Z) |

**Provenance, not a fabrication:** the robot/cube/table values above are
exactly this repo's pre-existing `REAL_CUBE_POS`/`ROBOT_POS` constants
(measured 2026-08-19 per the script's prior comments), transformed into
base-relative coordinates (`base = world - robot_world`, since yaw is 0),
not a new/independent measurement. **Cube dimensions and the bowl's X
offset are the exceptions** — `0.056m` and the bowl's `0.18m` cube-to-bowl
center distance are fresh, direct measurements the user provided in this
session (2026-08-20), superseding the ~0.057m cube-size estimate in
`docs/object-pose-mirroring-plan.md` and the old `0.153m` offset
respectively. **The bowl's X sign (which side of the robot it's on) is
still the user's original `+0.18` guess, unconfirmed** — see
`MEASURE_PHYSICALLY` below.

**`MEASURE_PHYSICALLY`** — re-verify these before trusting them for a new
physical setup, or if this fixture has moved since 2026-08-19:

- **All of the above except cube dimensions and the bowl's cube-to-bowl
  distance**, if your physical fixture
  differs from the one this repo's original constants were measured
  against.
- **Bowl X sign** specifically: the removed `REAL_BOWL_POS` comment flagged
  this as "a provisional guess pending a live-viewport check" that was never
  confirmed as far as this session's history shows — carried forward
  unchanged into the layout, but still unverified.
- **Cube Z / cube-size interaction**: `REAL_CUBE_POS.z = 0.7754` was
  measured 2026-08-19, using whatever cube-size assumption was current at
  that time — it does not exactly satisfy the `rest_on_table` formula
  against `table_surface_z_world_m` and the now-confirmed `0.056m` size
  (`0.7504 + 0.056/2 = 0.7784` vs the measured `0.7754`, a 3.0mm gap —
  under this build's 5mm warning tolerance, but worth re-measuring directly
  if you have the physical setup available, rather than trusting a value
  computed one field-remove from the original tape measurement).

## Exact launch commands

Sim-only, fixed layout:
```powershell
C:\Isaac-Sim\python.bat source\sim_to_real_so101\scripts\leader_arm_teleop_raw_isaacsim.py --layout calibration\workspaces\aws_cube_bowl_fixed.json
```

Sim-only, today's legacy default (no layout file, unchanged from before this
change):
```powershell
C:\Isaac-Sim\python.bat source\sim_to_real_so101\scripts\leader_arm_teleop_raw_isaacsim.py
```

With the real follower mirrored (only after the sim-only manual verification
sequence above passes):
```powershell
C:\Isaac-Sim\python.bat source\sim_to_real_so101\scripts\leader_arm_teleop_raw_isaacsim.py --layout calibration\workspaces\aws_cube_bowl_fixed.json --follower_port COM3
```

## Git diff summary

```
 M docs/aws-cube-to-bowl-run-guide.md
 M source/sim_to_real_so101/scripts/leader_arm_teleop_raw_isaacsim.py
 M source/sim_to_real_so101/utils/scene_reset.py
?? calibration/workspaces/aws_cube_bowl_fixed.json
?? source/sim_to_real_so101/utils/fixed_workspace.py
?? docs/hardcoded-real2sim-implementation-receipt.md
   (tests/test_fixed_workspace.py exists on disk but is excluded by
   .gitignore's repo-wide `tests/` rule, same as pre-existing tests/mdp
   and tests/usd content — not shown by `git status`)
```

No unrelated files were modified. `keyboard_agent_raw_isaacsim.py`,
`aws_cube_to_bowl_env_cfg.py`, and `real-to-sim.usd` were inspected but not
changed.
