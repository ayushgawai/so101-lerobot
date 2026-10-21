# Wrist-Roll Real-to-Sim Alignment — Implementation Receipt

## Original report

Real leader-arm teleop against the raw (no Isaac Lab) sim
(`leader_arm_teleop_raw_isaacsim.py`): the physical wrist's upright/neutral
orientation did not visually match the simulated wrist's orientation,
apparently offset by roughly 90°. Near-zero leader values produced
near-zero simulator targets, so the mismatch looked like a missing constant
offset rather than a sign or scale error. Log samples at the time:

```
wrist_roll -- leader raw= -15.96 deg, sim target= -25.53 deg
wrist_roll -- leader raw= -67.91 deg, sim target=-108.66 deg
wrist_roll -- leader raw=  +1.89 deg, sim target=  +3.02 deg
wrist_roll -- leader raw= -56.40 deg, sim target= -90.23 deg
wrist_roll -- leader raw= +20.00 deg, sim target= +32.00 deg
```

These match `sim_target = 1.6 * leader_raw` exactly (no offset term).

## Root cause

`LeRobotSO101Interface.get_mapped_actions_vectorized()`
(`source/sim_to_real_so101/utils/lerobot_interface.py`) maps every joint
through a pure scale-through-zero: `raw=0` always maps to `mapped_deg=0`.
For `wrist_roll`, `SO101_USD_MAPPING`'s USD range is `-160..160°`, giving
the observed `1.6x` scale — **intentional and correct**, not the bug (it's
what stretches the leader's `-100..100` reading to cover the joint's full
authored travel).

The actual defect: this design silently assumes the leader's calibration
zero (`raw=0`) coincides with the simulated joint's own authored 0° pose.
That assumption is more fragile for `wrist_roll` than the other five
joints, for two independent reasons:

1. `assets/so101.py`'s Isaac-Lab `ArticulationCfg.init_state` parks
   `Wrist_Roll` at **-1.6034 rad (~-91.9°)**, not 0° — nothing else in this
   codebase treats this joint's USD-zero as the robot's visual "neutral"
   either.
2. `wrist_roll` is lerobot's sole `full_turn_motor`: its calibration range
   is hardcoded `0..4095` (a full revolution) instead of motion-capture-derived
   like the other five joints, so its `raw=0` is defined purely by wherever
   the arm was held during calibration's homing step, with no mechanical
   range-of-motion anchor.

Separately, this repo's own memory/investigation from the same day
(`lerobot 0.6.1 upgrade + recalibration`) found the *actual* proximate cause
of the originally-reported 90° mismatch: `wrist_roll`'s stored calibration
had drifted (`range_min`/`range_max` wrapped to `4095/8190`, above a 12-bit
STS3215's `0..4095`), and was fixed by a plain recalibration earlier that
day — see "Live hardware verification" below for how this was confirmed.

## Files changed

**Created:**
- `source/sim_to_real_so101/utils/wrist_roll_alignment.py` — pure math (no
  isaacsim/omni/lerobot/torch import), same "isaac-free, unit-testable"
  convention as `utils/fixed_workspace.py`. Defines `WristRollAlignment
  (direction_sign, zero_offset_deg)` implementing
  `unclamped_deg = direction_sign * scaled_deg + zero_offset_deg`, clamped
  to the joint's limits, returning every intermediate value for debugging.
  Full root-cause writeup lives in its module docstring.
- `source/sim_to_real_so101/utils/test_wrist_roll_alignment.py` — 6 pytest
  tests: leader-neutral → sim-neutral (default and offset cases),
  positive/negative direction, direction-sign flip, and clamping (including
  an offset pushing a target past a limit). Runs without Isaac Sim:
  ```
  PYTHONPATH=source python -m pytest source/sim_to_real_so101/utils/test_wrist_roll_alignment.py -v
  ```
  All 6 passed as of this change.

**Modified:**
- `source/sim_to_real_so101/scripts/leader_arm_teleop_raw_isaacsim.py`:
  - Imports `WristRollAlignment` in the pre-Kit pure-import block.
  - Added `WRIST_ROLL_ALIGNMENT = WristRollAlignment(...)` near
    `RAD_TO_DEG` — the single knob for this correction.
  - The per-tick joint-target loop now special-cases `Wrist_Roll` only:
    applies the alignment after `SO101_USD_MAPPING`'s scale, before
    clamping. Every other joint's line is unchanged.
  - The periodic debug line now reports every stage: raw leader reading,
    scaled degrees, applied direction, applied offset, unclamped target,
    and clamped sim target — e.g.:
    ```
    [INFO]: wrist_roll -- leader raw= -55.87  scaled= -89.39 deg  direction=+1.0  offset=  +0.00 deg  unclamped= -89.39 deg  sim target(clamped)= -89.39 deg
    ```

**Deliberately not modified:** `get_mapped_actions_vectorized()` itself —
it's shared by `replay_act_dataset_to_sim.py`, `test_grasp_dynamics.py`,
`compare_real_vs_sim_joints.py`, the GR00T client, etc. Keeping the
correction scoped to `leader_arm_teleop_raw_isaacsim.py` means only
`wrist_roll` behavior in that one script changes; nothing else in the repo
is affected.

## Live hardware verification (2026-08-24)

With `WRIST_ROLL_ALIGNMENT` at its no-op default (`direction_sign=1.0,
zero_offset_deg=0.0`), ran the script live against the leader arm (COM4)
and watched the sim viewport:

| Pose | Real wrist | Sim wrist | Debug line |
|---|---|---|---|
| Neutral | jaws vertical | jaws ~vertical (slight roll) | `raw≈-0.84, target≈-1.34°` |
| Quarter-turn | jaws horizontal | jaws horizontal | `raw=-55.87, target=-89.39°` |

Direction, magnitude, and visual landmark all agreed at both checkpoints —
confirming the offset=0 default is correct for the current (recalibrated)
arm, and that the originally-reported 90° mismatch does not reproduce.
This is consistent with the "corrupted calibration, not a USD/code bug"
root cause above.

A separate, unrelated issue surfaced when testing with `--follower_port
COM3`: the process was silently killed by Kit during `follower.connect()`,
before the USD stage even loaded. This is the repo's pre-existing,
previously-documented "follower connect kills Kit with no traceback" issue
(see `isolate_follower_connect.py` / `isolate_dual_arm_connect.py` /
`isolate_kit_dual_arm_connect.py` / `isolate_busy_scene_dual_arm.py`, and
the comment in `main()`) — **not fixed as part of this change**, and
orthogonal to wrist_roll alignment.

## Current state

Despite the live verification above showing `zero_offset_deg=0.0` as the
matching value, `WRIST_ROLL_ALIGNMENT` was subsequently changed at the
user's explicit request to:

```python
WRIST_ROLL_ALIGNMENT = WristRollAlignment(direction_sign=1.0, zero_offset_deg=-90.0)
```

(`+90.0` was set first, then changed to `-90.0`; `direction_sign` stayed
`1.0` throughout, per the requirement that real and sim keep rotating the
same side.) **This value was not re-verified live on hardware** — the
request was made without a fresh reported mismatch, after the offset=0
default had already been confirmed correct. If the simulated wrist looks
~90° off at neutral on the next run, that confirms `-90.0` is wrong for the
current calibration; revert to `0.0` (known-good as of the verification
above), or try `+90.0`.

## How to re-verify / adjust

```
C:\Isaac-Sim\python.bat source\sim_to_real_so101\scripts\leader_arm_teleop_raw_isaacsim.py
```

1. Hold the physical wrist upright/neutral, watch the `wrist_roll --` debug
   line for `leader raw` near 0, and compare the simulated wrist's
   orientation.
2. If off by ~90° one way, set `zero_offset_deg` to `+90.0`; the other way,
   `-90.0`. If the *direction* of rotation is mirrored (not just the
   resting pose), flip `direction_sign` to `-1.0` instead.
3. Edit the constant at `leader_arm_teleop_raw_isaacsim.py`'s
   `WRIST_ROLL_ALIGNMENT` line and re-run.
