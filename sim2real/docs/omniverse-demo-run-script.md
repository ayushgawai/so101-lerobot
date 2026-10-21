# Omniverse Lab Live Demo — Run Script

A narration + command script for presenting the real-to-sim SO-101 pipeline to a
**mixed audience** (some technical, some not): physical measurement → Claude Code scene
authoring → USD Composer → Isaac Sim → live teleop (sim-only, then real+sim together).
Nothing in this doc has been executed — it's a script to follow live.

**How to read this doc:**

- 🎤 = say this out loud, to everyone. Plain language, no jargon assumed.
- 🔧 = technical backup — exact values, file paths, code references. Skim past these
  live unless someone technical asks a follow-up, or the room is clearly technical.
- Code fences are exact commands/prompts to type at that beat.

**Quick glossary**, for framing the demo up front if the room is mixed:

- **Omniverse / Isaac Sim** — NVIDIA's 3D simulation software; think "video-game engine,
  but physically accurate" (real gravity, friction, collisions).
- **USD Composer** — the tool used to build the 3D scene (the room, the table) by hand.
- **USD stage** — the actual 3D scene file, like a saved video-game level.
- **Claude Code** — an AI coding assistant, used here to turn measurements and a photo
  into that 3D scene file automatically instead of building it by hand.
- **Teleop (teleoperation)** — controlling a robot arm remotely; here, a human moves a
  "leader" arm by hand, and that motion is copied onto a "follower" arm and/or a
  simulated one.
- **Leader arm / follower arm** — two identical physical robot arms. The leader is the
  one a person moves by hand; the follower copies it, in real time, with no one
  touching it.

Verified against this repo's current state — see the 🔧 notes throughout, and the
source files: [real-to-sim-environment-prompt.md](real-to-sim-environment-prompt.md),
`real-to-sim.usd`, and
[leader_arm_teleop_raw_isaacsim.py](../source/sim_to_real_so101/scripts/leader_arm_teleop_raw_isaacsim.py).

---

## Timeline

1. Measure the physical table
2. Photograph the workspace (reference angle)
3. Recap the ChatGPT material/texture description
4. Log into Omniverse, open the repo in Claude Code
5. Claude Code: measurements + photo → scene spec + regenerated USD
6. USD Composer: open the generated stage, verify dims in the Property panel
7. Show the prompt template next to the generated `.usd`
8. Isaac Sim: bring in the environment, reference the robot, frame a wide shot
9. Run sim-only teleop, one pick-and-place
10. Run real+sim together, one slow synchronized pick-and-place
11. Walk through what the script is doing

---

## Part 1 — Measure the physical table

> 🎤 "Everything you're about to see starts here — a tape measure. This whole digital
> version of the table only means something if it's the exact size of the real one, so
> I'm not guessing or eyeballing it."

Measure and say the numbers out loud as you record them:

- **Length** (long edge)
- **Width** (short edge)
- **Height**, floor to tabletop

🔧 Existing digital twin numbers to sanity-check against: length `1.2 m`, width `0.7 m`,
height `0.75 m`. If your live measurement lands close to those, that's expected — same
table this environment was already modeled from.

> 🎤 "If these come back close to what I already had, that's not a coincidence — it
> confirms the digital table is genuinely a match for the real one. I'm still measuring
> it live so you can see where these numbers actually come from."

Jot the three numbers somewhere you'll paste from in Part 5 (sticky note, Notes app,
whatever — no repo file needed yet).

---

## Part 2 — Photograph the workspace

> 🎤 "Now a photo, from the same spot and angle as a reference shot I took earlier —
> that way the two images line up and I can compare them side by side."

Take the photo from that same angle. No command needed — just have it ready on your
phone/desktop for Part 5.

🔧 You'll either describe the photo to Claude Code or drop the file into the repo, e.g.
`docs/reference-photos/table-2026-08-27.jpg`, and reference that path in the prompt.

---

## Part 3 — Recap the ChatGPT description

> 🎤 "Earlier, I showed a photo of this table to ChatGPT and asked it to describe what
> it looks like — the color, the texture, the material, how the legs and wheels are put
> together. That description is what gives the digital version its realistic look,
> instead of a plain gray placeholder box."

Have [real-to-sim-environment-prompt.md](real-to-sim-environment-prompt.md) open —
point at **Section 5 (Materials)** as the artifact of that step.

🔧 Specifically the `Laminate` material's diffuse color / roughness values, if a
technical question comes up about how the look is actually encoded.

---

## Part 4 — Log into Omniverse, open Claude Code

> 🎤 "Now I switch from measuring to building. First I log into my Omniverse account —
> that's the platform all of this runs on — and open my coding assistant, Claude Code,
> which is going to do the actual building for me."

- Log into the Omniverse account via the **Omniverse Launcher GUI** (no CLI step here).
- Open the VS Code workspace for this repo, open an integrated terminal, launch Claude
  Code from the repo root:

```powershell
claude
```

---

## Part 5 — Claude Code: measurements + photo → scene spec

> 🎤 "Here's the actual handoff: I give my AI assistant the numbers I measured and the
> photo I took, and it turns that into a precise, buildable description of the scene —
> instead of me manually modeling a table for hours."

Paste into the Claude Code prompt (fill in your live numbers/photo path first):

```
I just re-measured our physical lab table with a tape measure:
- length (long edge): <LENGTH_M> m
- width (short edge): <WIDTH_M> m
- height, floor to tabletop: <HEIGHT_M> m

Reference photo (same angle as before): <PATH_TO_PHOTO>

docs/real-to-sim-environment-prompt.md is the scene spec this environment was built
from, and usd-composer-stages/build_usd_file1.py is the script that authored
source/sim_to_real_so101/demo/usd-file1.usd from it.

Compare my new measurements against the numbers already in the prompt doc's Section 6
(Table). If they match within a few millimeters, tell me so and don't change anything.
If they differ, update the prompt doc's numbers to match what I just measured, update
build_usd_file1.py's table geometry to match, and regenerate usd-file1.usd. Don't touch
the room shell, lighting, or materials sections unless the table height changes the
robot-mount-disc or wall dimensions.
```

If Claude Code regenerates anything, it will tell you to re-run the build script — do
that when it says to:

```powershell
usdenv\Scripts\python.exe usd-composer-stages\build_usd_file1.py
```

---

## Part 6 — USD Composer: verify the table

> 🎤 "Let's open what Claude Code just built and check it against the tape measure with
> our own eyes, in the actual 3D editor — I don't just take the AI's word for it."

```powershell
cd C:\USD-Composer
.\repo.bat launch
```

Once Composer is up (~30-40s first launch):

1. **File → Open** →
   `source\sim_to_real_so101\demo\usd-file1.usd`
2. Click on the tabletop in the 3D view (or find it in the Stage panel on the side).
3. Open the **Property** panel — this shows its exact size in meters. Cross-check
   against the tape measure numbers from Part 1.

🔧 The tabletop prim is `/World/Table/TableTop`; its mesh **extent** should read
`(-0.6, -0.35, 0)` → `(0.6, 0.35, 0.035)` (the 1.2 × 0.7 m top). The table root
`/World/Table` should have `translate = (0, 0, 0)`, with the top surface landing at
world `z = 0.75` (TableTop's own `translate.z = 0.715` + the 3.5 cm slab thickness).

---

## Part 7 — Show the prompt template next to the result

> 🎤 "Here's the instruction sheet I wrote — every dimension, every material, every
> part of the table is spelled out, so this build is repeatable. Anyone could hand this
> same document to the AI and get the same table back out."

Have both open side by side:

- [real-to-sim-environment-prompt.md](real-to-sim-environment-prompt.md) (the spec)
- `source/sim_to_real_so101/demo/usd-file1.usd` in Composer (the result)

---

## Part 8 — Isaac Sim: environment + robot, wide shot

> 🎤 "Now I bring this room into Isaac Sim — the physics engine — and add the actual
> robot arm into it, so the two exist in the same digital space."

Launch plain Isaac Sim (no Isaac Lab needed for this step):

```powershell
C:\Isaac-Sim\isaac-sim.bat
```

1. **File → Open** →
   `source\sim_to_real_so101\demo\usd-file1.usd`
2. In the **Stage** outliner, right-click `/World` → **Add → Reference...** → browse to
   `source\sim_to_real_so101\assets\usd\SO-ARM101-USD.usd`.
3. Select the newly-referenced robot → **Property** panel → **Transform** → set its
   position so it sits on the table where the real arm is mounted.

   🔧 `translate = (0, 0.3, 0.72)`, `orient` = identity (no rotation) — the exact
   values already authored on `/World/SO_ARM101_USD` in `real-to-sim.usd`, so this
   lines the robot up with the table the same way the teleop script's world does.

4. **Articulation / joints** — nothing to type by hand here.

   > 🎤 "I don't need to manually tune how each joint moves — that's handled
   > automatically the moment the control script connects to the arm."

   🔧 The teleop script sets stiffness/damping/effort on all six joints (`Rotation`,
   `Pitch`, `Elbow`, `Wrist_Pitch`, `Wrist_Roll`, `Jaw`) every time it runs
   ([leader_arm_teleop_raw_isaacsim.py:307-320](../source/sim_to_real_so101/scripts/leader_arm_teleop_raw_isaacsim.py#L307-L320)).
   For the demo, just confirm the six joint prims exist under
   `/World/SO_ARM101_USD/joints` and each has a `drive:angular` API applied — that's
   the "everything's wired up correctly" check.

5. **Wide shot**: select both tables and the robot together, press **F** to frame
   selection, then back the camera off slightly so everything's in view for the
   audience.

   🔧 A known-good starting camera pose (from the original scene's authored `Persp`
   camera): `translate = (-0.0561, -0.5554, 1.4005)`, `rotateXYZ = (51.130, 0, -1.555)`,
   if you want to type it in directly instead of dragging.

---

## Part 9 — Sim-only pick-and-place

> 🎤 "First pass: I move the real leader arm, but only the simulated robot in the
> computer copies me — nothing physical is moving yet. This proves the connection and
> the control logic work before I bring a second real robot into it."

```powershell
C:\Isaac-Sim\python.bat source\sim_to_real_so101\scripts\leader_arm_teleop_raw_isaacsim.py
```

- Boots in ~30-45s. Click into the viewport once it opens (needs focus for keyboard
  reset).
- Wait for the console to say the leader arm is connected and driving joints.

  🔧 Exact lines:
  ```
  [INFO]: Leader arm connected: port=COM4 id=my_so_arm
  [INFO]: Driving joints from the leader arm. Ctrl+C or close the window to stop.
  ```
- Do one full pick-and-place with the leader arm (cube → bowl).
- Press `R` to reset the cube/bowl to their starting position if a grasp goes wrong.
- `Ctrl+C` (or close the window) to stop when done.

---

## Part 10 — Real + sim together

> 🎤 "Now for the real payoff: the same hand motion on the leader arm drives *two*
> things at once — the simulated robot on screen, and a second, completely real robot
> arm sitting right here — at the same time, matching each other move for move."

```powershell
C:\Isaac-Sim\python.bat source\sim_to_real_so101\scripts\leader_arm_teleop_raw_isaacsim.py --follower_port COM3
```

> 🎤 "I'm not touching the leader arm yet — I'm waiting for the connection to confirm
> first, so the follower doesn't lurch on startup."

Wait for the console to confirm **all three** of: leader connected, follower connected,
and the startup sync finishing — before moving anything.

🔧 Exact lines:
```
[INFO]: Leader arm connected: port=COM4 id=my_so_arm
[INFO]: Follower arm connected: port=COM3 id=my_so_arm
[INFO]: Follower startup sync complete, entering normal mirroring.
```
That last line matters — the script gently ramps the follower from wherever it's
sitting up to match the leader's current position over ~1.5s, instead of snapping,
specifically so the real follower arm doesn't jerk on connect
([leader_arm_teleop_raw_isaacsim.py:352-363](../source/sim_to_real_so101/scripts/leader_arm_teleop_raw_isaacsim.py#L352-L363)).

- One **slow** pick-and-place, watching the sim and the real follower move together.
- After placing, bring the leader arm back to a neutral/rest pose slowly — the sim and
  the follower both track it back down.
- `Ctrl+C` to stop. Both arms disconnect and everything stops cleanly no matter how the
  run ends
  ([leader_arm_teleop_raw_isaacsim.py:716-728](../source/sim_to_real_so101/scripts/leader_arm_teleop_raw_isaacsim.py#L716-L728)).

---

## Part 11 — Explaining the script

> 🎤 "In plain terms: this script is a translator. Thirty times a second, it reads the
> position of every joint on the leader arm, converts that into a target the simulated
> robot's motors understand, and — if a second real arm is connected — sends that exact
> same reading to it too. All three (leader, sim, follower) end up in lockstep, moving
> off the same live signal."

That's the version for a general audience. If the room is technical or it comes up in
Q&A, here's what's actually happening under the hood, in the order the script executes:

🔧 **Technical detail:**

- **Arms connect before anything else loads.** Leader (and follower, if
  `--follower_port` is given) connect over serial *before* the USD stage, physics, or
  rendering come up — a real bug was found where connecting a follower while physics
  was already playing silently killed the whole process, so connect-first sidesteps it
  ([leader_arm_teleop_raw_isaacsim.py:409-435](../source/sim_to_real_so101/scripts/leader_arm_teleop_raw_isaacsim.py#L409-L435)).
- **Cube/bowl placement** comes from a fixed-workspace layout (base-frame math relative
  to the robot mount), not hardcoded world coordinates — `--layout` can point at a
  measured JSON, same idea as Part 5's measurement-to-spec step but for object poses
  instead of the table.
- **Per-tick loop**: read the leader's raw action → convert to per-joint degree targets
  → apply a wrist-roll-specific alignment correction (the leader's zero and the sim
  joint's authored zero don't coincide for that one joint — a `-90°` offset, tuned
  2026-08-24) → write all six joint drive targets → optionally mirror the same raw
  leader reading to the real follower, unmodified → step the sim.
- **Actuator gains** (`JOINT_GAINS`) are Isaac-Lab-tuned PD values applied directly to
  each joint's `PhysicsDriveAPI`, unconverted — the `Jaw` joint is deliberately capped
  at `effort_limit=3` (not 30) so it can grip the cube without ejecting it.
- **Diagnostics** print every 5s, not every tick: average tick time, average follower
  `send_action()` time, and a full wrist-roll trace (raw leader reading → scaled degrees
  → alignment-corrected → clamped) so a visual mismatch can be traced to a number
  without flooding the console.
- **Shutdown** always disconnects both arms and stops the timeline, even on a lost
  connection or an unhandled exception mid-loop.

---

## Quick reference

| Step | Command |
|---|---|
| Claude Code | `claude` |
| Regenerate `usd-file1.usd` | `usdenv\Scripts\python.exe usd-composer-stages\build_usd_file1.py` |
| USD Composer | `cd C:\USD-Composer && .\repo.bat launch` |
| Isaac Sim (standalone) | `C:\Isaac-Sim\isaac-sim.bat` |
| Sim-only teleop | `C:\Isaac-Sim\python.bat source\sim_to_real_so101\scripts\leader_arm_teleop_raw_isaacsim.py` |
| Real + sim teleop | `C:\Isaac-Sim\python.bat source\sim_to_real_so101\scripts\leader_arm_teleop_raw_isaacsim.py --follower_port COM3` |
