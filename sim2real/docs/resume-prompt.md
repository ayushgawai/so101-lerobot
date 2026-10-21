# Resume Prompt (paste this to start a new session)

Copy everything in the code block below as your first message in a fresh conversation when this
one runs out of context.

```
I'm continuing work on the Sim-to-Real-SO-101-Workshop repo at
c:\Users\OMNI-User\Desktop\Sim-to-Real-SO-101-Workshop.

Read docs/isaac-sim-windows-guide.md first, in full — it's a living document covering everything
done so far in detail: getting Isaac Lab 2.3.2 (with its bundled Isaac Sim 5.1.0.0) installed
natively on Windows, four real Windows-specific problems hit and fixed along the way, a
forward-compatibility shim for a possible future Isaac Lab 3.0 migration, and confirmed visual
tests of both Isaac Sim and USD Composer. Don't re-derive things that document already answers.

Quick state summary:
- Isaac Lab is installed and confirmed working at `Y:\e` (a Python 3.11 venv), where `Y:` is a
  `subst`-mapped drive pointing at `C:\ilab` (needed to work around a Windows path-length limit
  with no admin rights available). Run any workshop script via:
  `Y:\e\Scripts\python.exe -m sim_to_real_so101.scripts.<script> --task <task_name>`
  If `Y:` is missing (e.g. after a reboot), run `subst Y: C:\ilab` manually — a Startup-folder
  script should normally recreate it automatically at login.
- `list_envs` and `zero_agent --task Lerobot-So101-Teleop-Vials-To-Rack` were both run successfully
  and visually confirmed (SO-101 arm, lightbox, mat, 3 vials rendering correctly in the viewport).
- USD Composer (`cd C:\USD-Composer; .\repo.bat launch`) was also confirmed working — it opened
  `SO-ARM101-USD.usd` directly, fully independent of Isaac Lab.
- The original ask was three tasks:
  1. Confirm Isaac Sim can simulate the SO-101 doing pick-and-place via teleoperation.
  2. Build a simple indoor environment scene in USD Composer.
  3. Export that scene, import into Isaac Sim, verify axis/scale alignment with the robot.
- Task 1's teleop approach: since there's no physical SO-101 leader arm, we decided on a
  **keyboard-jogging** script (`keyboard_agent.py`, mapping key pairs to +/- deltas on each of the
  6 joints) rather than hardware teleop or Isaac Lab's IK-based `Se3Keyboard` device. **This script
  is NOT YET WRITTEN — it's the next concrete step for task 1.**
- Tasks 2 and 3 have not been started yet.

What I want to do next: <FILL THIS IN — e.g. "implement keyboard_agent.py", "help me build the
indoor scene in Composer", "pick up wherever makes sense">

Keep updating docs/isaac-sim-windows-guide.md as new things are discovered or done — it's meant to
stay a living record across sessions, not just for this one.
```
