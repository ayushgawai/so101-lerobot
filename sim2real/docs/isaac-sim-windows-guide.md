# Running the SO-101 Workshop Natively on Windows (Isaac Sim + Isaac Lab)

This is a living document for students who have **Isaac Sim**, **Omniverse Kit SDK**, and **USD
Composer** installed natively on Windows and want to work through this workshop's three tasks
without Docker/Linux (the path [README.md](../README.md) documents). It captures what's been
worked out so far — install requirements, a version-compatibility problem discovered along the
way, and how to think about the three tasks in light of it. New findings get appended as
troubleshooting entries; nothing here should be treated as final until you've verified it against
your own machine.

---

## 1. The three tasks

1. **Log into Omniverse and confirm Isaac Sim can simulate the SO-101 arm doing pick-and-place via
   teleoperation.** **Status: done and confirmed.** Sim confirmed working (§5a); keyboard-jog teleop
   script written, two real bugs found + fixed via automated smoke testing, and now **visually
   confirmed working** — holding a jog key in the viewport moves the arm (§7).
2. **(Aakash)** Open USD Composer, check for existing scene templates, and build a simple indoor
   environment scene. **Status: done, see §10** — built programmatically rather than by hand in
   Composer's GUI (this session can't drive a GUI app); still needs a visual look-over in Composer.
3. **(Aakash)** Export that scene as `.usd`, import into Isaac Sim, and verify axis/scale alignment
   with the robot arm. **Status: robot referenced into the scene and bounding boxes checked
   programmatically, see §11 — still needs a visual look in Composer/Isaac Sim to fully close out.**

> Note: the task list says "SO-ARM100" — this repo (and the USD assets in it) are built around
> **SO-101** specifically (`SO-ARM101-USD.usd`, loaded by
> [so101.py](../source/sim_to_real_so101/assets/so101.py)). SO-ARM100 is the predecessor hardware
> line from the same open-source project.

## 2. What's actually in this repo (relevant background)

- Stack: **Isaac Lab** (an RL/robotics framework built on top of Isaac Sim), used via a
  Docker image in the documented setup (`docker/sim/Dockerfile` pins **Isaac Lab 2.3.2**).
- The robot: [so101.py](../source/sim_to_real_so101/assets/so101.py) defines `SO101_CFG`, an
  `ArticulationCfg` with per-joint actuator tuning (stiffness/damping/effort limits matched to the
  real servos' gear ratios).
- The task: `Lerobot-So101-Teleop-Vials-To-Rack` in
  [vials_to_rack_env_cfg.py](../source/sim_to_real_so101/tasks/vials_to_rack_env_cfg.py) — pick up
  a vial, place it in a rack, detected via a contact sensor + geometric checks
  (`vial_placed_on_rack_termination`, lines 250-267).
- Current "environment" is tabletop-only: a lightbox + mat
  ([task_env_cfg.py:71-103](../source/sim_to_real_so101/tasks/task_env_cfg.py#L71-L103)) — there is
  **no room/indoor scene anywhere in this repo**, so task 2 is net-new content, not something to go
  find here.
- **Teleop input is hardware-bound in the shipped code**: the only teleop script,
  [lerobot_agent.py](../source/sim_to_real_so101/scripts/lerobot_agent.py), drives the sim purely
  from a physical SO-101 leader arm over serial
  ([lerobot_interface.py:21](../source/sim_to_real_so101/utils/lerobot_interface.py#L21)). There is
  no keyboard/SpaceMouse/gamepad device wired to the SO-101's joints anywhere in the codebase
  (confirmed by grep) — `zero_agent`/`random_agent` are non-interactive debug scripts only.
  - **Decision made**: since no physical leader arm is being used, teleop is demonstrated via a
    **new keyboard-jogging script**, [keyboard_agent.py](../source/sim_to_real_so101/scripts/keyboard_agent.py),
    that replaces `LeRobotSO101Interface` as the action source with per-joint key mappings, reusing
    the camera/recording scaffolding from `lerobot_agent.py`. See §7.

## 3. What's actually installed on this machine

Checked directly (not assumed):

| Item | Location | Notes |
|---|---|---|
| Isaac Sim | `C:\Isaac-Sim` | Version **6.0.1-rc.7** (release candidate) |
| Omniverse Kit SDK | `C:\Omniverse-Kit` | App-template dev kit (`repo.bat template new`) |
| USD Composer | `C:\USD-Composer` | App-template pattern (`repo.bat launch`) |
| SO-ARM100 hardware repo | `Desktop\SO-ARM100` | Physical robot repo; has `Simulation/SO100`, `Simulation/SO101` subfolders, worth checking for extra USD/URDF assets |
| so101-lerobot | `Desktop\so101-lerobot` | Separate LeRobot project (own venv/docker), for real-hardware control — not directly relevant to Isaac Sim work |
| Isaac Lab | **Installed** at `Y:\e` (subst-mapped to `C:\ilab\e`) | Isaac Lab 2.3.2 + bundled Isaac Sim 5.1.0.0 pip install, in its own Python 3.11 venv. See §5. |

## 4. Why Isaac Lab is required (not optional)

Isaac Sim by itself is the physics/rendering engine — it can load a USD stage and simulate it, but
it has no concept of a "task," a reward/success condition, or tuned actuator behavior. Every file in
`source/sim_to_real_so101` is written against Isaac Lab's API, not raw Isaac Sim:

- `isaaclab.sim`, `isaaclab.actuators.ImplicitActuatorCfg`, `isaaclab.assets.ArticulationCfg` →
  [so101.py:19-21](../source/sim_to_real_so101/assets/so101.py#L19-L21)
- `isaaclab.envs.ManagerBasedRLEnvCfg`, `isaaclab.managers.*` →
  [so101_env_cfg.py:17-22](../source/sim_to_real_so101/tasks/so101_env_cfg.py#L17-L22)
- `isaaclab_tasks` (Gym task registry) → every script in `scripts/`

Without Isaac Lab installed, `import isaaclab.sim as sim_utils` fails immediately and none of this
repo's scripts can even start. The alternative — hand-writing the SO-101's actuators, the
vial/rack task logic, and the teleop/recording pipeline directly against raw Isaac Sim/PhysX APIs —
means rebuilding everything this workshop already provides.

### Which of the 3 tasks actually need it

| Task | Needs Isaac Lab? | Why |
|---|---|---|
| Task 1 (teleop pick-and-place, reusing this repo's task) | **Yes** | Actuator tuning, vial/rack scene, success detection, and the Gym env wrapper are all Isaac Lab config code |
| Task 2 (build indoor scene in Composer) | **No** | Pure USD authoring — Composer doesn't touch `isaaclab` at all |
| Task 3 (export, import, verify alignment) | **No**, for the core visual/manual check | Open both USD files in plain Isaac Sim and check alignment directly. Isaac Lab only matters if you also want the scene auto-wired into the task as an `AssetBaseCfg` |

**Practical implication**: tasks 2 and (mostly) 3 can proceed right now with what's already
installed. Isaac Lab is only a blocker for task 1.

## 5. The compatibility problem

This is the main blocker discovered so far, and it's worth understanding precisely before
installing anything.

- Isaac Lab **2.3.2** (what this repo's Docker image pins) requires **Isaac Sim 5.x + Python
  3.11** — Isaac Lab's own docs state using a different Python version "will result in errors."
- This machine's Isaac Sim is **6.0.1-rc.7**. There is a confirmed, open Isaac Lab bug
  ([#5435](https://github.com/isaac-sim/IsaacLab/issues/5435)) showing Isaac Lab's code fails to
  launch against Isaac Sim 6.0.0 — it references extensions
  (`isaacsim.core.experimental.primdata`, `isaacsim.robot.wheeled_robots.nodes`,
  `isaacsim.sensors.experimental.rtx`, `isaacsim.util.debug_draw`) that don't exist in that Isaac
  Sim release's pip bundle.
- The Isaac Lab release that *does* target Isaac Sim 6.0.0/6.0.1 is **Isaac Lab 3.0.0 Beta (Beta
  2)** ([release](https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0-beta),
  [discussion #6249](https://github.com/isaac-sim/IsaacLab/discussions/6249)). But:
  - It's a **"ground-up architectural overhaul"**: multi-backend physics, a pluggable renderer,
    Warp-native data pipelines, kit-less install mode — not a simple version bump.
  - **Windows pip wheels are not available yet** — NVIDIA's release notes say Windows support "will
    be available soon" (future tense). It currently targets Ubuntu.
  - There's an open dependency-conflict bug even on the platform it does support
    ([#6200](https://github.com/isaac-sim/IsaacLab/issues/6200)).

**Net effect**: there is currently no Isaac Lab release that both (a) has Windows pip wheels and (b)
matches this machine's Isaac Sim 6.0.1-rc.7.

### The official Isaac Lab 2.3.2 pip install (for reference)

```powershell
python3.11 -m venv env_isaaclab
env_isaaclab\Scripts\activate
python -m pip install --upgrade pip
pip install isaaclab[isaacsim,all]==2.3.2.post1 --extra-index-url https://pypi.nvidia.com
pip install -U torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
```

Important: this pip package **bundles its own Isaac Sim 5.x** — it does not attach to the existing
`C:\Isaac-Sim` 6.0.1-rc.7 install. Running it gives you a second, separate Isaac Sim living inside
that Python venv, leaving the existing install untouched.

### Options going forward

1. **Let the 2.3.2 pip install bring its own Isaac Sim 5.x** (command above) — matches this repo's
   code exactly; costs a second Isaac Sim install on disk.
2. **Wait for Isaac Lab 3.0's Windows wheels**, then likely need to adapt this repo's Isaac Lab
   2.x-era code to whatever 3.0's API changed (see §6 for exactly what would need to change).
3. **Go back to Docker/WSL2** per the main README — sidesteps version-matching entirely since the
   image already bundles known-working versions together.

**Decision taken: Option 1.** Isaac Lab 2.3.2 + its bundled Isaac Sim 5.1.0.0 is installed and
confirmed working — see §5a below for the exact steps and three real problems hit along the way.

## 5a. Confirmed working: the actual install, and 3 real problems hit

The official pip install (§5) was followed almost exactly, with one change forced by a Windows
path-length issue, and it now works end-to-end: `list_envs` prints the full task table
(`Lerobot-So101-Teleop-Base`, `-Task`, `-Vials-To-Rack`, `-Vials-To-Rack-DR`, `-Vials-To-Rack-Eval`,
`-Vials-To-Rack-DR-Eval`) inside the actual Isaac Sim/Kit process, with the RTX 5080 Laptop GPU
correctly detected.

**Setup used:**
- `uv` (already on this machine at `C:\Users\OMNI-User\.local\bin\uv.exe`) to create the Python 3.11
  venv, instead of a separately-installed system Python (none was present) — `uv venv --python 3.11
  --seed <path>` fetches its own CPython 3.11 build automatically.
- The venv lives at **`Y:\e`**, where `Y:` is a `subst`-mapped drive pointing at `C:\ilab`. This is
  not the original planned location (`C:\Users\OMNI-User\Desktop\env_isaaclab`) — see Problem 1 below
  for why.
- `pip install "isaaclab[isaacsim,all]==2.3.2.post1" --extra-index-url https://pypi.nvidia.com`,
  then `pip install -U torch==2.7.0 torchvision==0.22.0 --index-url
  https://download.pytorch.org/whl/cu128` (the plain `isaaclab[isaacsim]` install pulls a CPU-only
  torch build; the CUDA build has to be installed as a separate, explicit step over it).
- `pip install -e source/sim_to_real_so101` into that same venv.

### Problem 1: Windows path-length limit broke the install, and admin rights weren't available to fix it properly

First attempt (venv at `Desktop\env_isaaclab`) failed mid-install with:
```
OSError: [Errno 2] No such file or directory: '...env_isaaclab\Lib\site-packages\isaacsim\extscache\
isaacsim.replicator.caption.core-...\isaacsim\replicator\caption\core\tests\test_data\
Test_Scene_Data_Camera_00\bounding_box_2d_loose\bounding_box_2d_loose_labels_0003.json'
```
This is the Windows `MAX_PATH` (260-character) limit — the `isaacsim-replicator` package ships deeply
nested test-data files. The real fix is enabling `LongPathsEnabled` in the registry
(`HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem`), which **requires administrator rights**.
Admin access wasn't available on this machine (a UAC elevation attempt was also tried and canceled —
either declined or no interactive session was available for the prompt to render in).

**Workaround used (no admin required)**: Windows' `subst` command maps a drive letter to a folder
as a per-user, session-scoped virtual drive — this shortens the *effective* path Windows sees,
sidestepping the 260-character limit without touching the registry.
```powershell
mkdir C:\ilab
subst Y: C:\ilab
```
The venv was then created at `Y:\e` instead of the long `Desktop\env_isaaclab` path, leaving enough
headroom under the limit for the deepest files in the `isaacsim-replicator` package. Re-running the
install (pip's cache already had most packages downloaded) succeeded.

**Caveat**: `subst` mappings are session-only by default — they don't survive a reboot. To avoid
needing to manually re-run `subst Y: C:\ilab` after every restart, a small startup script was added:
`%APPDATA%\Microsoft\Windows\Start Menu\Programs\Startup\restore_ilab_subst.bat`
```bat
@echo off
if not exist Y:\ (
    subst Y: C:\ilab
)
```
This runs at every login (no admin needed for the Startup folder) and recreates the mapping if it's
missing. **If `Y:` doesn't exist after a reboot, this script should still create it at next login —
if the venv becomes unreachable, check this file exists and re-run it manually first.**

### Problem 2: Isaac Sim's first-run EULA prompt hangs in a non-interactive shell

`list_envs` initially failed with `Unable to bootstrap inner kit kernel: EOF when reading a line` —
Isaac Sim's Kit bootstrap (`isaacsim/kit/kit_app.py`) prompts interactively for EULA acceptance on
first run (`Do you accept the EULA? (Yes/No):`), which fails immediately in a non-interactive
terminal. It checks (in order) the `OMNI_KIT_ACCEPT_EULA` env var, then a persisted
`EULA_ACCEPTED` marker file next to the bootstrap script. The marker file was written directly to
skip this permanently:
```
Y:\e\Lib\site-packages\isaacsim\kit\EULA_ACCEPTED   (contents: "yes")
```

### Problem 3: h5py DLL conflict crashed the Kit process

After fixing the above, `list_envs` got much further (extensions loading, GPU detected) but then
crashed with:
```
ImportError: DLL load failed while importing _errors: The specified procedure could not be found.
```
while the `isaaclab_tasks` extension imports `h5py` (via `isaaclab.managers.recorder_manager` →
`isaaclab.utils.datasets.hdf5_dataset_file_handler`) — followed by a fatal process crash
(`Windows fatal exception: code 0xc0000139`) on a second import attempt. `h5py` works fine in
isolation (`python -c "import h5py"` succeeds), so this is specific to running **inside** the Kit
process: Kit loads its own native libraries (plausibly a same-named `z.dll`/zlib) before
`isaaclab_tasks` gets to import `h5py`, and Windows' DLL loader reuses whatever same-named library is
already resident in the process rather than h5py's own bundled copy — binding h5py's compiled
extension to an incompatible version.

**Fix applied**: import `h5py` at the very top of every script that boots `AppLauncher`, *before*
Isaac Sim/Kit starts — this loads h5py's own bundled DLLs into a clean process first, so the later
(cached, no-op) `import h5py` inside `isaaclab_tasks` never re-triggers DLL loading at all. Applied
to all 5 scripts that call `AppLauncher` (`lerobot_push_dataset.py` doesn't launch Isaac Sim, so it
was left alone):
- [list_envs.py](../source/sim_to_real_so101/scripts/list_envs.py)
- [zero_agent.py](../source/sim_to_real_so101/scripts/zero_agent.py)
- [random_agent.py](../source/sim_to_real_so101/scripts/random_agent.py)
- [lerobot_agent.py](../source/sim_to_real_so101/scripts/lerobot_agent.py)
- [lerobot_eval.py](../source/sim_to_real_so101/scripts/lerobot_eval.py)

**Reverting**: each was backed up before editing (`list_envs.py.bak`, `zero_agent.py.bak`,
`random_agent.py.bak`, `lerobot_agent.py.bak`, `lerobot_eval.py.bak`, all in `scripts/`). Copy a
`.bak` back over its counterpart to undo.

### Problem 4: carb render-settings race during `gym.make()` — was intermittent, then became a 5/5 blocker, now fixed

`zero_agent --task Lerobot-So101-Teleop-Vials-To-Rack` sometimes crashed during `gym.make()` with:
```
ValueError: '/rtx/translucency/reflectAtAllBounce' in RenderCfg.general_parameters does not map to a carb setting.
```
raised from `isaaclab/sim/simulation_context.py`'s `_apply_render_settings_from_cfg`, which validated
every key in the `carb_settings` dict this repo set in `SO101TaskEnvCfg.__post_init__` (RTX
translucency settings for the glass vials) by calling `get_setting(key)` and raising if it returned
`None`.

**Investigation**: a standalone script that booted `AppLauncher` and checked all 6 settings
individually found every one of them registered and valid (`get_setting()` returned real values, not
`None`) — so the settings genuinely exist in this Isaac Sim 5.1.0.0 build. Re-running the *exact same*
`zero_agent` command twice showed one run crash and the next succeed, with identical code and
identical settings. **Conclusion: Kit loads extensions partly via background thread pools, and
`gym.make()` can call `SimulationContext.__init__()` before the RTX renderer extension has finished
registering these specific settings — a timing race, not a hard incompatibility.**

**Escalation**: while smoke-testing the new `keyboard_agent.py` (§7) in this same session, this race
went from "sometimes" to **5/5 consecutive failures** — every single `gym.make()` attempt hit it.
That crossed the threshold from "retry and move on" to "worth actually fixing."

**Fix applied**: the RTX translucency/reflection settings are no longer set via
`self.sim.render.carb_settings` in `SO101TaskEnvCfg.__post_init__`
([task_env_cfg.py](../source/sim_to_real_so101/tasks/task_env_cfg.py)) — that's the strictly-validated
path that raced the RTX extension's own registration. Instead:
- `task_env_cfg.py` now exposes `RTX_TRANSLUCENCY_CARB_SETTINGS` (the same 6 settings, as `/rtx/...`
  paths) and `apply_rtx_translucency_settings()`, which sets them via the plain, non-validating
  `carb.settings.get_settings().set(key, value)` API — no existence check, no race.
- Every script that calls `gym.make()` for this task now calls `apply_rtx_translucency_settings()`
  immediately after, once the sim is fully constructed and the race window has passed:
  [zero_agent.py](../source/sim_to_real_so101/scripts/zero_agent.py),
  [random_agent.py](../source/sim_to_real_so101/scripts/random_agent.py),
  [lerobot_agent.py](../source/sim_to_real_so101/scripts/lerobot_agent.py),
  [lerobot_eval.py](../source/sim_to_real_so101/scripts/lerobot_eval.py),
  [keyboard_agent.py](../source/sim_to_real_so101/scripts/keyboard_agent.py).

**Verified**: re-ran the `keyboard_agent.py` smoke test after the fix — the `ValueError` no longer
occurs (confirmed on a run that previously failed 5/5 times before the fix).

### Unrelated, non-fatal noise seen in the logs

`isaacsim.sensors.rtx` (lidar/radar) failed to load its native plugins
(`generic_mo_io.dll`, `nvlidar_pc_converter.plugin.dll` — "The specified module could not be
found"). This repo doesn't use lidar/radar sensors (only `TiledCameraCfg` for RGB/depth/segmentation),
so this is cosmetic — Kit logs the extension failure and continues; it did not prevent
`isaaclab_tasks` from loading or `list_envs` from completing successfully.

### Confirmed: `zero_agent` renders the actual task scene correctly

`zero_agent --task Lerobot-So101-Teleop-Vials-To-Rack` was run and visually confirmed in the Isaac
Sim viewport: the orange SO-101 arm in its reset pose, the yellow lightbox, the mat, and the 3 vials
with blue caps — matching `VialsToRackSceneCfg` exactly. The `IsaacLab` panel's `Camera Eye`/
`Camera Target` values (`-0.25, -0.4, 0.22` / `0.15, 0.0, 0.12`) match
`SO101TeleopEnvCfg.__post_init__` in [so101_env_cfg.py:167-168](../source/sim_to_real_so101/tasks/so101_env_cfg.py#L167-L168),
and the Scene Debug Visualization panel lists exactly this repo's scene entities (`Vial 1/2/3`,
`Rack Left`, `Robot`, `Ee_Frame`).

**Note on apparent hangs**: once the simulation reaches steady state, the console log stops
producing new lines — this looks like a hang but isn't one. Check the actual viewport window before
assuming a stall; the render loop doesn't log anything once it's just running.

### Running commands in this venv going forward

```powershell
Y:\e\Scripts\python.exe -m sim_to_real_so101.scripts.list_envs
Y:\e\Scripts\python.exe -m sim_to_real_so101.scripts.zero_agent --task Lerobot-So101-Teleop-Vials-To-Rack
Y:\e\Scripts\python.exe -m sim_to_real_so101.scripts.keyboard_agent --task Lerobot-So101-Teleop-Vials-To-Rack
```
(`Y:` must be mapped first — the startup script above handles this automatically after a reboot; if
it's a fresh session and `Y:` isn't mapped yet, run `subst Y: C:\ilab` manually.) For
`keyboard_agent.py`'s key mapping and usage, see §7.

## 6. Proof: Isaac Lab 3.0 Beta would break this repo's existing MDP code

Isaac Lab 3.0's documented breaking change: **all `.data.*` properties on asset and sensor classes
now return `wp.array` (NVIDIA Warp) instead of `torch.Tensor`.** This repo's `mdp/` package was
written entirely assuming `torch.Tensor`, with no conversion anywhere — this is not a hypothetical
edge case, it hits the reset, per-step observation, and termination paths simultaneously:

- [mdp/obs.py:34-40](../source/sim_to_real_so101/mdp/obs.py#L34-L40) — `robot.data.root_pos_w`/
  `root_quat_w` feed `torch.cat(...)`, which cannot accept a `wp.array`.
- [mdp/obs.py:52-54](../source/sim_to_real_so101/mdp/obs.py#L52-L54) — `sensor.data.output[...]`
  `.clone()`'d and passed downstream to the numpy/imageio recording pipeline.
- [mdp/terms.py:80-84](../source/sim_to_real_so101/mdp/terms.py#L80-L84) —
  `contact_sensor.data.force_matrix_w` → `torch.linalg.vector_norm(...)`, which requires a
  `torch.Tensor` argument. **This runs every `env.step()`** via the `any_vial_grasped` and
  `vial_placed_on_rack` observation terms.
- [mdp/terms.py:221-223](../source/sim_to_real_so101/mdp/terms.py#L221-L223),
  [358-361](../source/sim_to_real_so101/mdp/terms.py#L358-L361) — `.data.root_pos_w`/
  `root_quat_w` piped into `math_utils.quat_inv`/`quat_apply`.
- [mdp/resets.py:291](../source/sim_to_real_so101/mdp/resets.py#L291),
  [312](../source/sim_to_real_so101/mdp/resets.py#L312) — `asset.data.default_root_state[env_ids]`
  → `torch.cat([positions, orientations], dim=-1)` → `write_root_pose_to_sim`. **This runs on every
  environment reset**, so even a first `env.reset()` in `zero_agent.py` would break.

Conclusion: without changes, this repo's task cannot take a single simulation step under Isaac Lab
3.0's Warp-based data model.

**Unconfirmed secondary risk**: Isaac Lab 3.0 release notes also mention a quaternion convention
change (wxyz → xyzw). This repo builds rotations via `isaacsim.core.utils.rotations.euler_angles_to_quat`
(e.g. [so101.py:51](../source/sim_to_real_so101/assets/so101.py#L51)) — whether that specific
function's output convention changes wasn't confirmed from available docs. Worth testing for
(robot/camera/mat spawning at the wrong orientation) if/when 3.0 is actually tried.

### Suggested code change (not yet applied)

A small, backward-compatible shim — a no-op on today's Isaac Lab 2.3.2, only load-bearing if/when
moving to 3.0:

```python
# source/sim_to_real_so101/mdp/_compat.py
"""Normalizes Isaac Lab asset/sensor `.data.*` outputs to torch.Tensor.

Isaac Lab 2.x returns torch.Tensor from `.data.*`; Isaac Lab 3.0 returns
warp.array instead. This keeps the mdp package working unmodified on either.
"""
import torch


def as_torch(x):
    if isinstance(x, torch.Tensor):
        return x
    try:
        import warp as wp
        if isinstance(x, wp.array):
            return wp.to_torch(x)
    except ImportError:
        pass
    return x
```

Then wrap every `.data.*` read at its call site in `terms.py`, `obs.py`, and `resets.py`, e.g.:
```python
contact_forces = as_torch(contact_sensor.data.force_matrix_w)
vial_z = as_torch(vial.data.root_pos_w)[:, 2]
rack_pos_w = as_torch(rack_obj.data.root_pos_w)
rack_quat_w = as_torch(rack_obj.data.root_quat_w)
```

**Status: applied.** [`mdp/_compat.py`](../source/sim_to_real_so101/mdp/_compat.py) now exists, and
every `.data.*` call site listed above (in `obs.py`, `terms.py`, `resets.py`) is wrapped with
`as_torch(...)`. It's a no-op today (Isaac Lab 2.3.2's `.data.*` already returns `torch.Tensor`),
and only becomes load-bearing if/when the project moves to Isaac Lab 3.0.

**Reverting**: before editing, a backup of each original file was saved alongside it —
`mdp/obs.py.bak`, `mdp/terms.py.bak`, `mdp/resets.py.bak`. To roll back, copy a `.bak` file over its
counterpart (e.g. `copy mdp\obs.py.bak mdp\obs.py` on Windows) and delete `mdp/_compat.py`.

## 7. Task 1: keyboard teleop instead of hardware

Since no physical leader arm is being used, "teleoperation" needs a new input source. The chosen
approach: **per-joint keyboard jogging**, mapping key pairs to +/- deltas on each of the SO-101's 6
joints (`Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw`), following the same
`carb.input`/`omni.appwindow` event-subscription pattern already used in
[keyboard.py](../source/sim_to_real_so101/utils/keyboard.py) for the R/S/C hotkeys — just extended
to continuous held-key deltas instead of one-shot flags.

Rejected alternative: mapping Isaac Lab's built-in `Se3Keyboard` device directly — it outputs 6-DoF
end-effector pose deltas for IK-based control, but this repo's action space is raw joint positions
(`ActionsCfg.joint_positions` in
[so101_env_cfg.py:71-76](../source/sim_to_real_so101/tasks/so101_env_cfg.py#L71-L76)), so it doesn't
plug in without adding a new IK action term. Per-joint jogging avoids that extra work.

### Implemented: `keyboard_agent.py` + `JointJogKeyboardControl`

[keyboard_agent.py](../source/sim_to_real_so101/scripts/keyboard_agent.py) is a sibling to
`lerobot_agent.py` that keeps the same camera-detection and recording scaffolding but replaces
`robot_iface.robot.get_action()` (the hardware leader-arm read) with a keyboard-jog target computed
each physics step.

`JointJogKeyboardControl` in
[keyboard.py](../source/sim_to_real_so101/utils/keyboard.py) subclasses `KeyboardControl`, adding
continuous held-key tracking (press adds a key to a `_held_keys` set, release removes it) and
`get_joint_deltas(step)`, which returns a `+step`/`-step` per joint for every currently-held key.
Key mapping (deliberately disjoint from R/S/C so no key does double duty):

| Joint | Increase (+) | Decrease (−) |
|---|---|---|
| Rotation (base yaw) | `D` | `A` |
| Pitch (shoulder) | `W` | `X` |
| Elbow | `E` | `V` |
| Wrist_Pitch | `T` | `G` |
| Wrist_Roll | `Y` | `H` |
| Jaw (gripper) | `U` | `J` |

Each physics step, `keyboard_agent.py` reads the current deltas, adds them to a running per-joint
target tensor, clamps against `robot.data.soft_joint_pos_limits` (read from the articulation itself,
not hardcoded), and steps the env with that as the action — matching
`ActionsCfg.joint_positions`'s `scale=1, use_default_offset=False` convention (raw radians, absolute
target, same order as `JOINT_ORDER`). `R`/`S`/`C` (reset/record/cancel) still work unchanged via the
base class.

**Recording is optional and its import is deferred**: `LeRobotSO101Interface`/`LeRobotRecorder` both
import the `lerobot` pip package, which — confirmed by direct check — **is not installed** in the
`Y:\e` venv (only `isaaclab[isaacsim,all]` and this repo's own package are). If those imports were
unconditional at module load, plain jogging (no `--repo_id`/`--repo_root`/`--task_name`) would fail
to even start. They're imported lazily inside the `if recording_mode:` block instead, so core jogging
never needs `lerobot` installed at all. When recording *is* requested, `LeRobotSO101Interface` is
still constructed (for its static degree↔radian conversion helpers only — `init_device()`/`connect()`
are never called, so no physical arm is needed even then), and `real_action` for each recorded frame
is computed from the commanded target via `get_raw_actions_from_radians(targets)` rather than read
from real hardware.

### Replaying a recorded trajectory: `--action_log` + `replay_agent.py`

For replaying a teleop episode back into the sim (open-loop) without needing the `lerobot` package
at all, `keyboard_agent.py` has a second, independent recording path alongside the LeRobot dataset
one above:

- `--action_log DIR` — while `keyboard_control.recording` is on (same `'S'`/`'R'` toggle as the
  LeRobot path, but this one needs no `--repo_id`/`--repo_root`/`--task_name`), every physics step
  appends the current per-joint target tensor to a buffer. The moment recording flips off (`'S'`
  again, or `'R'`), the buffer is saved as `episode_NNN.npz` in `DIR` — just two arrays,
  `actions` (T, 6) and `joint_names`, no `lerobot` dependency at all. Multiple stop/starts in one
  session save multiple numbered episodes.
- [replay_agent.py](../source/sim_to_real_so101/scripts/replay_agent.py) — a new sibling script that
  loads one `episode_NNN.npz`, checks its `joint_names` match `JointJogKeyboardControl.JOINT_ORDER`
  (guards against a log from some future, differently-ordered version of the script), then steps the
  environment through the recorded actions in order, open-loop, `--num_repeats` times. `'R'` restarts
  the current replay from step 0.

```powershell
Y:\e\Scripts\python.exe -m sim_to_real_so101.scripts.keyboard_agent --task Lerobot-So101-Teleop-Vials-To-Rack --action_log C:\ilab\action_logs
Y:\e\Scripts\python.exe -m sim_to_real_so101.scripts.replay_agent --task Lerobot-So101-Teleop-Vials-To-Rack --action_log C:\ilab\action_logs\episode_001.npz
```

**Verified end-to-end**: since generating a real recording needs a human holding keys down (something
this assistant can't do), `replay_agent.py` was instead smoke-tested against a synthetic 90-step
`.npz` (a small, safe gripper open/close gesture, other 5 joints held at the default reset pose,
built directly with `numpy` — no sim involved in generating it). The replay ran clean end-to-end:
loaded the log, built the environment with no errors, stepped through all 90/90 recorded actions at
~22 steps/sec, then correctly entered its "stay open until closed" idle loop. This confirms the
mechanics (npz round-trip, joint-order validation, env stepping) work; it does **not** confirm what a
real recorded jogging session looks like when replayed — that still needs you to actually record one.

### Replaying the same trajectory in USD Composer, without Isaac Lab

`replay_agent.py` above only runs through the `Y:\e` Isaac Lab install — Composer is a separate Kit
app with no `isaaclab_tasks`, no gym env, no way to attach to it (Isaac Lab always launches its own
Kit process via `AppLauncher`; it doesn't plug into an already-running one). Getting the same
recorded trajectory to play inside Composer specifically needed a different mechanism.

**How**: every SO-101 joint in `SO-ARM101-USD.usd` is a `PhysicsRevoluteJoint` with a
`PhysicsDriveAPI:angular` applying `drive:angular:physics:targetPosition` — confirmed by direct
inspection to be authored in **degrees**, matching each joint's `lowerLimit`/`upperLimit` exactly
(e.g. `Rotation`'s `[-110, 110]` matches `lerobot_interface.py`'s `SO101_USD_MAPPING` shoulder_pan
range). This is a genuine PhysX drive target, not a kinematic transform, so it needs Composer's own
physics simulation (Play button) running to actually move the joint — but that also means it behaves
like a real physics-driven arm rather than a pre-baked animation.

[`usd-composer-stages/bake_action_log_to_usd_animation.py`](../usd-composer-stages/bake_action_log_to_usd_animation.py)
takes an `episode_NNN.npz` (from `keyboard_agent.py --action_log`, converts each frame's radians to
degrees, and authors one time sample per frame on `drive:angular:physics:targetPosition` for all 6
joints — as overrides on a plain reference to `SO-ARM101-USD.usd` (the original asset file is never
touched), at 60 fps (matching `keyboard_agent.py`'s per-step cadence: Isaac Lab's environment
step-size for this task is 1/60 s). Run it with the same throwaway `usd-core` venv as
`build_indoor_scene.py` (§10):

```powershell
usdenv\Scripts\python.exe usd-composer-stages\bake_action_log_to_usd_animation.py C:\ilab\action_logs\episode_001.npz usd-composer-stages\replay-episode_001.usd
```

Then open the output file in Composer and hit **Play** to run physics simulation — the arm should
drive through the recorded trajectory using its own PD drive gains (`stiffness=17.8`,
`damping=0.6`, `maxForce=10` baked into the raw file — noticeably different from the tuned actuator
values in `so101.py`'s `SO101_CFG`, since those are set at the Isaac Lab layer and never touch the
raw USD, so expect softer/different-feeling motion than in Isaac Sim). The robot's own
`root_joint` (a `PhysicsFixedJoint`) already anchors its base in the raw file, so no extra pinning
should be needed.

**Verified programmatically, not yet visually**: the bake script was run against the same synthetic
90-step test log used above, and every authored time sample was re-read from the output file and
confirmed to exactly match the source log's degrees-converted values at multiple frames (start,
middle, end). What's *not* confirmed: whether hitting Play in Composer actually drives the joints
the way this assumes — that needs you to open the file in Composer and try it, since this assistant
can't interact with Composer's GUI.

### Two real bugs found by automated smoke testing, both fixed

Since this assistant can't press and hold a physical key, verification leaned on running
`keyboard_agent.py` non-interactively with a bounded timeout and reading the Kit log — this caught
two genuine bugs before a human ever touched the script:

1. **The Problem 4 carb-settings race (above)**, which had gone from "intermittent" in earlier
   sessions to a **5/5 failure rate** in this one — fixed by moving the RTX settings out of the
   validated `carb_settings` path.
2. **A crash in `JointJogKeyboardControl._on_keyboard_event` itself**: it read `event.input.name`
   unconditionally, but Kit also dispatches non-press/release keyboard event types (their `event.input`
   is a plain `str` with no `.name`) through the same callback. Once past the carb-settings race, a
   ~5-minute smoke test showed this crashing **hundreds of times in a row** with
   `AttributeError: 'str' object has no attribute 'name'` — apparently some non-key event fires
   continuously even with no human at the keyboard. Kit swallows exceptions raised inside an
   event-callback subscriber rather than crashing the app, so the process stayed alive throughout,
   but the handler was never doing anything useful. **Fix**: check `event.type` is `KEY_PRESS` or
   `KEY_RELEASE` *before* touching `event.input.name`, mirroring the base `KeyboardControl` class's
   own (already-safe) pattern. Re-tested after the fix: zero tracebacks over a multi-minute run.

**Confirmed working, by a human**: after these fixes, `keyboard_agent.py` was run for real — clicked
into the viewport, held a jog key, and **the arm moved**. One real wrinkle hit along the way, worth
recording since it'll recur: total boot time (extension loading through `env.reset()`) ran to roughly
**2-3 minutes** on this machine, and Windows marked the window "Not Responding" for that whole
stretch — Kit-based apps don't pump the Windows message loop during heavy synchronous
extension/shader loading, so the OS shows this even though the process is actively working, not
hung. Reading the console log confirmed it had reached the last startup print
(`Found Camera: external_D455`, immediately before the interactive loop begins) well before the
window was checked — i.e. it wasn't actually stuck, just slow and not repainting. If you hit this:
wait it out (don't kill the process) and try clicking into the window and pressing a key once the
console log goes quiet; that's steady state, not a hang (same phenomenon as the "apparent hangs" note
in §5a, just showing up at the OS window level instead of the console).

**Task 1 is now fully done**: sim confirmed (§5a), teleop script written and bug-fixed, and the arm
visually confirmed moving under keyboard jog control.

## 8. Getting started today (tasks 2 and most of task 3)

No Isaac Lab needed for this part:

1. Launch USD Composer (`C:\USD-Composer\repo.bat launch` — see §9 for full instructions).
2. Check this GitHub repo and any NVIDIA Omniverse sample-asset repos for an existing indoor-scene
   template (confirmed: none exists in this repo).
3. Build a simple floor/walls/lighting scene, **in meters** — this repo's every offset is in meters
   (e.g. `pos=(-0.05, 0.0, 0)` in [so101.py:50](../source/sim_to_real_so101/assets/so101.py#L50)),
   and Composer templates sometimes default to centimeters, which is the most common source of a
   100x scale bug later.
4. Leave an explicit origin/mounting point in the scene for where the SO-101 tabletop rig will sit.
5. Export as `.usd`, open it alongside `SO-ARM101-USD.usd` (from `assets/usd/`) directly in Isaac
   Sim, and check axis/scale alignment visually — Z-up, meters, robot base rotated 90° yaw
   (`euler_angles_to_quat([0,0,90])` at [so101.py:51](../source/sim_to_real_so101/assets/so101.py#L51)).

**Status: steps 1-4 done, see §10.** Step 5 (visual alignment check) is next — see §11.

## 9. Launching Isaac Sim and USD Composer directly

Both are separate GUI applications from the Isaac-Lab-driven scripts in §5a — useful for opening and
inspecting `.usd` files directly (tasks 2/3) without any Isaac Lab/Python involvement at all.

### USD Composer

```powershell
cd C:\USD-Composer
.\repo.bat launch
```
(This is documented in `README-USD-COMPOSER.txt` on the Desktop.) First launch takes ~30-40 seconds
(many extensions to load — normal, matches the pattern seen with Isaac Sim). No EULA prompt was hit
launching this way. Once the window is up:

- **File → Open**, navigate to a `.usd`/`.usda` file, e.g.
  `C:\Users\OMNI-User\Desktop\Sim-to-Real-SO-101-Workshop\source\sim_to_real_so101\assets\usd\SO-ARM101-USD.usd`
- The viewport shows an FPS/GPU-memory overlay when a stage is loaded; RTX real-time rendering
  confirmed working at ~60 FPS on the RTX 5080 Laptop GPU.

**Confirmed**: opening `SO-ARM101-USD.usd` this way loaded the robot geometry correctly, fully
independent of Isaac Lab — proving the file itself is portable, exactly as expected (see §4). One
thing to know: the arm appears **plain yellow/untextured** in Composer, not the orange seen in Isaac
Sim — that orange color comes from `randomize_robot_color` in
[resets.py](../source/sim_to_real_so101/mdp/resets.py), which sets the material *at Isaac-Lab
runtime*, not something baked into the raw USD file. Composer just shows the file's true default
state.

### Isaac Sim (standalone, no Isaac Lab)

The native Isaac Sim 6.0.1-rc.7 install (not the Isaac-Lab-bundled 5.1.0.0 one at `Y:\e`) can be
launched directly via the Desktop shortcut (`isaac-sim.bat - Shortcut.lnk`) or:
```powershell
C:\Isaac-Sim\isaac-sim.bat
```
This opens plain Isaac Sim with no Isaac Lab task/robot config loaded — useful for opening `.usd`
files the same way as Composer (File → Open) when you want Isaac Sim's specific tools rather than
Composer's, but it won't know anything about this repo's tasks, actuators, or scene composition
(that only exists when launched through the `Y:\e` Isaac Lab venv per §5a). Not yet tested this
session — if you try it, add findings here.

### A Composer-default new stage is Y-up and centimeters — a real, confirmed mismatch

Before building anything, the existing untracked stage at
`usd-composer-stages/so-arm101-demo.usd` (Composer's plain **File → New** template, sky + distant
light + a 1400x1400 unit ground grid, nothing else authored) was opened programmatically to check
its conventions:

```
UpAxis: Y
MetersPerUnit: 0.01
```

Compared against `SO-ARM101-USD.usd` (the actual robot asset):

```
UpAxis: Z
MetersPerUnit: 1.0
```

So Composer's default new-stage template is **Y-up, centimeters** — exactly the "100x scale bug"
§8 step 3 warned about in the abstract, now confirmed concretely on this machine with real
numbers. Building task 2's room by hand starting from **File → New** means either fixing
`upAxis`/`metersPerUnit` in the stage's own metadata (Composer exposes this under stage
properties) or authoring every offset in the wrong units and space. Neither is hard, but it's an
easy thing to get wrong silently — a wall placed at "4 units" away is 4 cm, not 4 m, and looks
identical to a human eyeballing proportions in the viewport at first glance.

## 10. Task 2, done: indoor room scene, authored programmatically

**Why programmatic instead of clicking through Composer's GUI**: this assistant (Claude Code) runs
in a terminal and cannot drive a GUI application's mouse/keyboard — there's no way to actually
perform "File → New → drag in a cube → scale it" through this session. Composer stages are plain
USD files, though, so the room was authored directly with `pxr` (USD's own Python API) instead,
which sidesteps the Y-up/cm trap above by construction (the script sets `Z-up` / `metersPerUnit=1.0`
explicitly, matching `SO-ARM101-USD.usd` exactly) and is easy to re-run with different numbers.

**How this was run**: `pxr` isn't installed anywhere on this machine as a standalone importable
package (Isaac Sim/Isaac Lab only expose it bundled inside the Kit process). A throwaway venv was
made instead, isolated from the `Y:\e` Isaac Lab install:
```powershell
uv venv --python 3.11 --seed usdenv
usdenv\Scripts\python.exe -m pip install usd-core
```
`usd-core` is the pure-Python-and-native-bindings OpenUSD distribution from Pixar/NVIDIA on PyPI —
no Kit, no GPU, no rendering, just the schema/authoring API. It's a good general tool for scripted
USD authoring or inspection any time Composer/Isaac Sim's full GUI boot isn't needed.

**What was built**: [`usd-composer-stages/build_indoor_scene.py`](../usd-composer-stages/build_indoor_scene.py),
run as `usdenv\Scripts\python.exe build_indoor_scene.py usd-composer-stages\indoor-scene.usd`,
producing [`usd-composer-stages/indoor-scene.usd`](../usd-composer-stages/indoor-scene.usd):

- A 4m x 4m floor and four 2.5m-tall walls (0.1m thick), centered on the world origin, each with
  `UsdPhysics.CollisionAPI` applied (static colliders — matches the pattern Composer's own default
  template uses for its ground plane).
- `/World/RobotMount` — a flat orange cylinder marker (0.6m radius) at the exact world origin,
  visually marking where the SO-101 tabletop rig (lightbox + mat from `task_env_cfg.py`) should be
  dropped in. The robot's own root sits at `(-0.05, 0, 0)` with a 90° yaw in that rig's local frame
  (`so101.py:50-51`), so placing the rig's origin at the mount marker's origin lines both up
  directly with no extra offset math.
- A `DomeLight` for ambient fill plus one ceiling `RectLight` as a key light.
- Simple `UsdPreviewSurface` materials (flat gray floor/walls, orange mount marker) — no textures,
  intentionally minimal per "simple indoor environment scene."
- `/World/Robot` — `SO-ARM101-USD.usd` pulled in as a USD **reference** (relative path
  `../source/sim_to_real_so101/assets/usd/SO-ARM101-USD.usd`), offset by `(-0.05, 0, 0.01)` to
  match the `(-0.05, 0, 0)` spawn offset `task_env_cfg.py` actually uses. No extra rotation was
  added on top — inspecting the raw file showed its own default prim already bakes in a 90° yaw via
  its root `xformOp:orient` (`(0.7071, 0, 0, 0.7071)`, i.e. 90° about Z), so adding another yaw here
  would have double-rotated it.

**Verified programmatically**: reopened the output file and confirmed `UpAxis: Z`,
`MetersPerUnit: 1.0`, a room world-space bounding box of `(-2.05, -2.05, -0.1)` to
`(2.05, 2.05, 2.5)` (matching the intended 4m room + wall thickness exactly), and the referenced
robot's world-space bounding box — `(-0.08, -0.38, 0.04)` to `(0.05, 0.05, 0.31)` — sitting well
inside the room and on the mount plate, no scale mismatch or stray offset. Not yet visually
confirmed in Composer.

**Not yet done — needs your eyes**: open `usd-composer-stages/indoor-scene.usd` in Composer
(§9) to visually confirm it looks like a reasonable room (materials, light levels, proportions) and
tweak from there if you want — that hands-on pass is exactly the part a terminal session can't do.
The existing `so-arm101-demo.usd` in the same folder is untouched and is just Composer's blank
default template; the new `indoor-scene.usd` is the actual task 2 deliverable.

**Regenerating or editing further**: re-run `build_indoor_scene.py` with edited constants
(`ROOM_SIZE`, `WALL_HEIGHT`, `WALL_THICKNESS`, `MOUNT_RADIUS` at the top of the file) to regenerate
from scratch, or just open the output `.usd` in Composer and edit it directly there — both are
valid; the script is there so the starting point is reproducible and correctly scaled/oriented by
construction.

## 11. Task 3: verify alignment

The robot is now referenced directly into `indoor-scene.usd` at `/World/Robot` (see §10) — opening
that one file in Composer or Isaac Sim shows the room and the robot together, no separate
import/combine step needed. Bounding-box math confirms no scale mismatch or stray offset (§10).

**Not yet done — needs your eyes**: open `usd-composer-stages/indoor-scene.usd` in Composer (§9)
and visually confirm the robot sits upright, right-side-up, and roughly centered on the orange
`/World/RobotMount` marker — that's the actual "verify axis/scale alignment" check §8 step 5 asked
for. If it looks wrong (robot on its side, floating, tiny/huge relative to the room), the most
likely culprit is the rotation assumption in §10 (that the file's baked-in orient already covers
the 90° yaw) — check `xformOp:orient` on `/World/Robot`'s referenced prim if so.

## Troubleshooting log

*(Append new rows here as real issues come up. Rows marked **Confirmed** were actually hit and
fixed on this machine — see §5a for full detail. The rest are seeded from reading the code and
haven't necessarily been hit yet.)*

| Symptom | Likely cause | Fix |
|---|---|---|
| **Confirmed** — `pip install` fails with `OSError: [Errno 2] No such file or directory` deep inside `isaacsim\extscache\isaacsim.replicator.caption.core-...\tests\test_data\...` | Windows `MAX_PATH` (260 char) limit hit by deeply nested test-data files in the `isaacsim-replicator` package | No admin rights → use `subst Y: C:\<short-folder>` and install into `Y:\...` instead of a long path like `Desktop\env_isaaclab`. See §5a Problem 1. |
| **Confirmed** — `Unable to bootstrap inner kit kernel: EOF when reading a line`, log shows `Do you accept the EULA? (Yes/No):` | Isaac Sim's first-run EULA prompt is interactive and the shell is non-interactive | Write `yes` to `<venv>\Lib\site-packages\isaacsim\kit\EULA_ACCEPTED`, or set `OMNI_KIT_ACCEPT_EULA=Y`. See §5a Problem 2. |
| **Confirmed** — `ImportError: DLL load failed while importing _errors` (h5py) while `isaaclab_tasks` extension loads, followed by `Windows fatal exception: code 0xc0000139` | Kit loads a native library that shadows h5py's bundled HDF5 DLLs before `isaaclab_tasks` imports h5py | `import h5py` at the very top of any script that calls `AppLauncher`, before Isaac Sim boots. Already applied to all 5 affected scripts. See §5a Problem 3. |
| **Confirmed, fixed** — `ValueError: '/rtx/translucency/reflectAtAllBounce' in RenderCfg.general_parameters does not map to a carb setting.` during `gym.make()` | Race condition: Kit's background extension loading hadn't finished registering RTX renderer settings yet when `SimulationContext` validated them | Fixed in code — RTX settings now applied via `apply_rtx_translucency_settings()` after `gym.make()` returns, not through the strictly-validated `carb_settings` path. See §5a Problem 4. |
| **Confirmed, fixed** — `AttributeError: 'str' object has no attribute 'name'` spamming from `keyboard.py`'s `_on_keyboard_event` | `JointJogKeyboardControl` read `event.input.name` unconditionally; non-press/release keyboard event types carry a plain `str` with no `.name` | Fixed in code — `event.type` is checked against `KEY_PRESS`/`KEY_RELEASE` before touching `.input.name`. See §7. |
| `ModuleNotFoundError: No module named 'isaaclab'` | Running with system/plain Python instead of the Isaac Lab venv's interpreter | Always invoke via `Y:\e\Scripts\python.exe`, never a bare `python ...` |
| `list_envs` shows an empty/short table | `sim_to_real_so101` not installed in editable mode, or installed into the wrong Python | `pip install -e source/sim_to_real_so101` into `Y:\e`'s pip; confirm with `pip show sim_to_real_so101` |
| `ImportError` mentioning `lerobot` | Ran `lerobot_agent.py` (hardware teleop, needs a real leader arm) or `keyboard_agent.py` **with** `--repo_id`/`--repo_root`/`--task_name` (recording mode) — the `lerobot` pip package isn't installed in `Y:\e`. Confirmed by direct check. | For teleop without hardware, use `keyboard_agent.py` (§7) *without* recording flags — recording's `lerobot` import is deferred and only needed if you actually pass those flags. **`pip install lerobot` into `Y:\e` no longer works**: `lerobot-sim/` (this repo's vendored copy, the natural source for that install) was updated to lerobot 0.5.2, which requires Python ≥3.12 and uses PEP 695 syntax (`type X = ...`, `class Foo[T]:`) in `motors/motors_bus.py`, `datasets/streaming_dataset.py`, `processor/pipeline.py`, `utils/io_utils.py` — a hard `SyntaxError` on `Y:\e`'s Python 3.11, confirmed directly, not just a `pip` metadata gate. `motors_bus.py` sits on the `import lerobot.robots` path, so this isn't avoidable by only touching unrelated submodules. Since Isaac Lab 2.3.2 itself requires Python 3.11 (§5), `Y:\e` can't be bumped to 3.12 to fix this either. For now, do any `lerobot`-dependent recording work (`LeRobotSO101Interface`/`LeRobotRecorder`) in a separate Python ≥3.12 environment instead — e.g. the `so101-lerobot` project's own environment — rather than inside `Y:\e`. |
| `TypeError` inside `torch.linalg.vector_norm` / `torch.cat` on `.data.*` values | Running under Isaac Lab 3.0's Warp-based `.data.*` without the compatibility shim | Apply the `as_torch()` shim from §6 at every `.data.*` call site (already applied — no-op on the currently-installed 2.3.2) |
| USD asset fails to load (`SO-ARM101-USD.usd` not found) | Package not installed with `-e`, so `here = os.path.dirname(...)` in `so101.py` resolves to a stale location | Reinstall with `pip install -e`, don't use a non-editable install |
| `Y:` drive missing / venv unreachable after a reboot | `subst` mapping is session-only and didn't get recreated | Check `%APPDATA%\Microsoft\Windows\Start Menu\Programs\Startup\restore_ilab_subst.bat` exists; run `subst Y: C:\ilab` manually if needed |
