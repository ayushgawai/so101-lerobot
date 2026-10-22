# Codex scene: leader drives simulation and real follower

Run from the repository root in PowerShell:

```powershell
.\sim2real\source\sim_to_real_so101\scripts\start_codex_dual_teleop.ps1 -Mode Preflight
.\sim2real\source\sim_to_real_so101\scripts\start_codex_dual_teleop.ps1 -Mode Dual -LeaderPort COM4 -FollowerPort COM3
```

The launcher uses `C:\Isaac-Sim\python.bat`, opens
`source/sim_to_real_so101/demo/codex-pick-place-table.usd`, and loads these
files from the **repository-root** `calibration/` directory:

- Leader: `teleoperators/so_leader/my_so_arm.json`
- Follower: `robots/so_follower/my_so_arm.json`

Backup and `.drifted_8190` files are never selected. Change paths/IDs with
`-CalibrationRoot`, `-LeaderId`, `-FollowerId`, and `-IsaacPython`.
The launcher works from other working directories too. It uses the existing
raw Isaac Sim controller; Isaac Lab and policy inference are not needed.
The Isaac Python environment needs this repository's LeRobot motor dependencies.
For dependency errors, use the repository's environment setup instructions in
[the existing run guide](aws-cube-to-bowl-run-guide.md).

## Starting and stopping

If the arm poses differ too much for the default `Match` mode, explicitly select
`Ramp`:

```powershell
.\sim2real\source\sim_to_real_so101\scripts\start_codex_dual_teleop.ps1 -Mode Dual -StartupMode Ramp
```

Press F8 and hold the leader still. All six commands start from the measured
follower position and change by at most 10 normalized units/second until the
five arm commands are within 1 unit of the leader, then normal 60-unit/s limiting
resumes. A 46.2-unit shoulder difference takes about 4.6 seconds with a stationary
leader. F9 pauses; F8 re-reads the follower and restarts convergence. This is a
joint-space command ramp, not collision-aware path planning or confirmation that
the physical follower has reached the commanded pose. Keep its travel path clear.
The gripper retains the follower driver's contact/stall relief. Invalid readings,
communication failures and control-gap handling remain enforced. Direct Python
invocation uses `--startup_mode ramp`. Default `Match` behavior is described below.

1. Place the physical follower in a supported resting pose before startup.
   Startup checks motor EEPROM against the selected JSON; it never prompts to
   recalibrate or writes new calibration. It preloads the follower's current
   position as its hold target before configuration enables torque.
2. The window starts paused. Move the unpowered leader to approximately the
   follower's pose, focus the Isaac Sim window, and press **F8**. The five arm
   joint differences must be at most 15 normalized units. The gripper may have
   a different opening: its target starts at the observed follower reading
   and ramps toward the leader at at most 60 normalized units/second, on both
   startup and resume. For example, follower=39.1 and leader=2.1 starts near
   39.1 and closes gradually instead of rejecting F8 or jumping to 2.1.
   A rejection for an arm joint prints the
   mismatched joint names, both readings, and the signed change needed in the
   leader reading, largest mismatch first. Values are normalized units, not
   degrees. Adjust the listed leader joints toward the
   follower readings and retry F8. Leave the powered follower in place.
3. **F9** pauses commands and simulation. The real follower retains its last
   target with torque enabled; it may finish moving to that target. **F8**
   checks the poses again before resuming. Pausing/stopping the Kit timeline
   also pauses the command gate.
4. **Esc**, **Ctrl+C**, or closing the window ends the session and attempts
   to disable torque and close both serial ports. Support the arm when stopping,
   since torque release can let it settle under gravity.
5. **R** pauses and resets the simulated cube/bowl. Reset the physical objects
   manually before pressing F8 again.

## Commands and scene layout

The loop reads the leader once per tick and limits the target's rate of change
to 60 normalized units/second at a nominal 30 Hz (`-ControlHz` overrides rate).
It sends that target to the real follower, then maps the follower's returned
command (including gripper stall relief) to the six simulated joint drives
before stepping simulation. Both arm configs explicitly use `use_degrees=False`:
body joints use -100..100 and the gripper uses 0..100, each normalized with that
arm's own calibration. The USD uses the project's existing joint-range mapping.
Serial and physics operations are sequential in one loop; this does not imply
identical physical trajectories or hardware-clock synchronization.

The default workspace JSON is
[`codex_pick_place_photo.json`](../calibration/workspaces/codex_pick_place_photo.json).
It preserves the ruler-derived scene: cube center (0, 0.03362524, 0.7775), bowl
bottom (-0.21, 0.03362524, 0.75), robot root (0, 0.3, 0.72), in world meters.
The bowl is on the robot's left (-X), 21 cm center-to-center from the cube,
with the original 11 cm edge-to-rim gap.
Runtime changes use the USD session layer and do not save over the source scene.
`-Layout path.json` selects another workspace. Camera registration is optional;
only pass `-RequireCameraAlignment` with a successful live registered layout.

The existing simulated wrist correction is **-90 degrees**, exposed as
`-WristRollOffsetDeg`. Calibration JSON establishes encoder normalization, not
the USD wrist zero. Compare the real and simulated neutral wrist in `SimOnly`
mode; pass `-WristRollOffsetDeg 0` if that matches the current setup. The offset
affects only simulation.

## Checks and error handling

```powershell
# File/config check only: no Kit startup, serial connection or motion.
.\sim2real\source\sim_to_real_so101\scripts\start_codex_dual_teleop.ps1 -Mode Preflight

# Finite headless scene/physics check: no arms or cameras opened.
.\sim2real\source\sim_to_real_so101\scripts\start_codex_dual_teleop.ps1 -Mode PhysicsCheck

# Leader and simulation only; F8 starts.
.\sim2real\source\sim_to_real_so101\scripts\start_codex_dual_teleop.ps1 -Mode SimOnly

# Optional real camera previews (not required for control).
.\sim2real\source\sim_to_real_so101\scripts\start_codex_dual_teleop.ps1 -Mode Dual -ShowCameras -FixedCamera 1 -WristCamera 0
```

Preflight validates both six-joint calibration schemas, IDs/ranges, distinct
ports, scene-file presence, and workspace parameters. It does not verify USB
availability, EEPROM or installed runtime dependencies. Connection errors name
the failing arm/port; check motor power, USB connections and other processes
using that port. An EEPROM mismatch requires explicit calibration verification.

Invalid/missing/nonfinite leader samples and serial errors terminate dual
operation with a nonzero exit status and best-effort cleanup of both arms.
No automatic reconnect or continuing simulation after follower failure occurs
in Dual mode. Short rendering/loop delays (including the observed 0.727-second
delay) continue using a fresh leader sample. Movement credit from the delay is
discarded: each target changes by at most one nominal command step, capped at
`min(1 / ControlHz, 1 / 30)` seconds. At the default settings this is at most
2 normalized units per joint, or 0.333 during the slower startup ramp. A delay
cannot trigger a large catch-up jump. `[DELAY]` logs report the gap and measured
render/serial durations; periodic statistics include maximum tick/render times.

A control gap **over 2 seconds** pauses until F8; the gate checks this when the
process regains control. Override this threshold with `-ControlTimeoutSec`
(direct Python: `--control_timeout_s`), e.g. `-ControlTimeoutSec 0.5` restores
the earlier stricter cutoff. The threshold must exceed the nominal control
period. F9, manual timeline pauses, scene resets, invalid samples and serial
errors continue to stop/pause operation as described above. This is not an independent hardware watchdog
for a hung process or disconnected cable. A failed write can have reached some
motors before reporting an error, so a serial write is not an atomic two-arm step.

Automated checks cover calibration parsing, explicit unit selection, EEPROM
mismatch, invalid reads, partial connection/configuration cleanup, mapping,
start-pose rejection, slew limits and timeouts. Hardware movement must be
verified on the physical setup.
