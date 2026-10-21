# Sim-to-Real SO-101: From USD Composer to Live Teleop

![SO-101 Vial to Rack Task](images/so101_banner.png)

This is a beginner-friendly walkthrough for building a 3D "digital twin" of a real
workspace and then driving a real robot arm and its simulated copy at the same time.
**No coding experience is required.** You'll mostly be talking to AI assistants in
plain English and clicking through a few applications.

By the end of this tutorial you will have:

1. Set up your computer with the tools this workshop uses.
2. Used ChatGPT + Claude Code to turn a photo of a real object into a realistic 3D
   model, edited live in **USD Composer**.
3. Opened a 3D scene in **Isaac Sim** and added the SO-101 robot arm to it.
4. Run a live teleoperation script that lets a physical "leader" robot arm control a
   simulated robot arm on screen (and optionally a second real "follower" arm too).

---

## Glossary (read this first if any of these words are new to you)

| Term | What it means |
|---|---|
| **Omniverse / Isaac Sim** | NVIDIA's 3D simulation software — think "video-game engine, but physically accurate" (real gravity, friction, collisions). |
| **USD Composer** | The tool used to build and edit a 3D scene (a room, a table, an object) visually. |
| **USD stage / `.usd` file** | The actual 3D scene file — like a saved video-game level. |
| **Claude Code** | An AI coding assistant. Here, it turns a plain-English description into the technical instructions that build or edit a `.usd` file. |
| **Prim** | Short for "primitive" — a single named object inside a USD scene (a table, a wheel, a light). |
| **Reference** | A way of pulling one `.usd` file into another without copying it — like inserting a linked object instead of a flat picture of it. |
| **Teleoperation ("teleop")** | Controlling a robot arm remotely. Here, a person moves a **leader** arm by hand, and that motion is copied in real time onto a **follower** arm and/or a simulated one. |

---

## What You'll Need

- **Your own Windows PC**, with **Isaac Sim 6.0.1** and **USD Composer** already
  installed (same versions used to build this workshop).
- **VS Code** — download from [code.visualstudio.com](https://code.visualstudio.com/)
  if you don't already have it.
- The **Claude Code** extension for VS Code.
- A **ChatGPT** account (used to describe reference photos in plain English).
- **uv** (a Python package manager) — used to run small USD-authoring scripts. Install
  it if you don't have it yet (see Part 1).
- A physical **SO-101 leader arm** (and optionally a second SO-101 as a follower), plus
  a calibration file for it — your instructor will give you this file via GitHub, see
  Part 1.

---

## Part 1 — Set Up Your Computer

This assumes **Isaac Sim 6.0.1** and **USD Composer** are already installed on your
PC. What's *not* set up yet on a fresh machine is the software this repo needs on top
of them — that's most of what this part covers.

### 1. Log in to your PC

Log in to your Windows account as usual.

### 2. Install VS Code

Download and install it from [code.visualstudio.com](https://code.visualstudio.com/)
if it isn't already on your machine.

### 3. Install the Claude Code extension

In VS Code, click the **Extensions** icon in the left sidebar (or press
`Ctrl+Shift+X`), search for **"Claude Code"**, and click **Install**.

### 4. Sign in

Open the Claude Code panel in VS Code and sign up or log in with your
Anthropic/Claude account when prompted.

### 5. Clone the workshop repo

In VS Code, open the **Source Control** panel (the branch icon in the left sidebar),
click **Clone Repository**, and paste in:

```text
https://github.com/aakashvardhan/Real2Sim2Real.git
```

Choose a folder to save it to, and open the cloned folder when VS Code asks.

> **Tip:** Everything from here on happens *inside* this cloned folder — Claude Code,
> USD Composer, and Isaac Sim all read and write files from it.

### 6. Ask Claude Code to install the required packages

You don't need to type any install commands yourself — Claude Code can do this for
you. Open the Claude Code panel in VS Code (with this repo open) and paste in:

```text
Set up this machine, one time, to run this repo's teleop scripts:

1. Install `uv` (a Python package manager) if it isn't already on this machine,
   using its official Windows install command.
2. Install the `lerobot` package (0.6.1) at the root of this repo — the folder
   one level above `sim2real/` — with its `[so101]` extra, into Isaac Sim's own Python at
   `C:\Isaac-Sim\python.bat` — not a separate virtual environment.
3. Then install `torch==2.11.0+cu128` and `torchvision==0.26.0+cu128` into that
   same `C:\Isaac-Sim\python.bat`, from the
   `https://download.pytorch.org/whl/cu128` index, so the versions match what
   Isaac Sim's bundled extensions expect.

Show me the exact commands you're about to run before running them, and tell me
clearly at the end whether every step succeeded.
```

Claude Code will likely ask you to approve running each command — click **Allow**
(or the equivalent) when it does.

> **Why the order matters:** the `lerobot` install pulls in a generic `torch` build
> first; installing this exact `torch`/`torchvision` version afterward overrides it
> to match what Isaac Sim expects — skipping it, or getting the order backwards, can
> crash Isaac Sim on startup with a native DLL/extension error. This is a **one-time
> step per PC** — you don't need to repeat it if you re-clone the repo later on the
> same machine.

### 7. Add your calibration file

The teleop script needs a calibration file matched to *your* specific SO-101 leader
arm — your instructor will give you a `my_so_arm.json` file via GitHub. Save it to:

```text
calibration\teleoperators\so_leader\my_so_arm.json
```

(If you're setting up a real follower arm too, its calibration file goes in
`calibration\robots\so_follower\` instead.) You'll point the script at this file by
name in Part 4.

---

## Part 2 — Turn a Real Object Into a Realistic 3D Model

This is the core workflow you'll reuse for anything you want to add or improve in the
scene (a table, a bin, a fixture — anything):

```mermaid
graph LR
    A["Take a photo"] --> B["Describe it in ChatGPT"]
    B --> C["Turn that into a technical\nprompt for Claude Code"]
    C --> D["Claude Code creates\nthe .usd file"]
    D --> E["Check it visually\nin USD Composer"]
```

**Step by step:**

1. **Take a photo** of the real object you want to recreate (a table, in this
   example).
2. **Ask ChatGPT to describe it.** Upload the photo and ask ChatGPT to describe the
   materials, colors, structure, and proportions in plain language — for example,
   *"describe this table's tabletop material, leg/frame design, and wheels in
   detail."*
3. **Turn that description into a technical prompt.** Take ChatGPT's description and
   shape it into a specific, structured request — listing what should stay the same,
   what should change, and what "realistic" means for this object. You can ask Claude
   Code itself to help tighten this prompt.
4. **Give the prompt to Claude Code**, in the VS Code Claude Code panel, so it can
   create the `.usd` file directly.
5. **Open the result in USD Composer** (Part 3 below) to look it over with your own
   eyes.

### Example prompt (condensed)

Here's a shortened example of what a good Claude Code prompt for this kind of task
looks like — adapt the specifics to your own object and photo:

```text
Create a new USD file at the root of this repo called `USD-Tutorial.usd`, containing
one table built to look like a realistic mobile classroom/lab table:

- Tabletop: light gray wood-look laminate, ~25-40mm thick, softly rounded corners
  and edges, matte finish (not glossy or metallic).
- Frame: dark gray powder-coated metal support columns, widening into V/Y-shaped
  feet near the floor.
- Casters: 4 black caster wheels, all touching the floor, no floating or clipped
  geometry.
- Realistic proportions: floor-to-tabletop height around 0.75m, with the tabletop,
  frame, and caster sizes all proportional to that.

Set the stage up correctly before building: Z-up, metersPerUnit = 1.0, table
resting on the ground plane at the world origin.

After building, report the tabletop thickness, corner radius, table height,
caster count, and confirm the table is actually resting on the floor with no
floating or clipped geometry.
```

> **Why this shape works:** it gives Claude Code a concrete file name and location so
> you know exactly where to find the result, spells out what "realistic" means in
> concrete numbers instead of vague adjectives, and asks for a report so you can
> verify the result instead of just trusting it.

---

## Part 3 — Open Your Scene in USD Composer

1. **Launch USD Composer.**

   ```powershell
   cd C:\USD-Composer
   .\repo.bat launch
   ```

   The first launch takes about 30-40 seconds while it loads extensions — this is
   normal.

2. **Open the file you just built.** Once Composer is up:
   - Click **File → Open**.
   - Navigate to `USD-Tutorial.usd` in the repo's root folder — the file Claude Code
     created for you in Part 2.

3. **Add the SO-101 robot into your scene.** With your file open:
   - Right-click on `/World` in the **Stage** panel (usually on the right side of
     the window), or use the **File** menu.
   - Click **File → Add Reference...**
   - Browse to and select:

     ```text
     usds\SO-ARM101-USD.usd
     ```

   This inserts the SO-101 robot model into your scene as a **reference** — it's
   linked in, not copied, so any future updates to the robot asset carry over
   automatically.

4. **Look it over.** Use your mouse to orbit/pan/zoom the viewport and check that the
   robot sits where you expect, right-side up, and roughly the right size relative to
   your scene.

> **Tip:** A brand-new empty Composer scene defaults to **centimeters** and
> **Y-up**, while this repo's assets are all built in **meters** and **Z-up**. If
> something looks 100x too big or lying on its side, that mismatch is the most likely
> cause — check the scene's stage properties.

---

## Part 4 — Run the Live Teleoperation Script

This is the finished demo: a physical SO-101 "leader" arm, moved by hand, drives a
simulated SO-101 arm in Isaac Sim in real time — and optionally a second real
"follower" arm at the same time.

### 1. Open a terminal

In VS Code:

- Click **Terminal → New Terminal** in the top menu, or press `` Ctrl+` ``.
- Make sure the terminal's working folder is the root of this repo (it should say
  something like `...\Sim-to-Real-SO-101-Workshop>` at the prompt — if not, use `cd`
  to get there).

### 2. Run the script

**Sim-only** (the leader arm controls only the simulated robot):

```powershell
C:\Isaac-Sim\python.bat source\sim_to_real_so101\scripts\leader_arm_teleop_raw_isaacsim.py
```

**Sim + a real follower arm together** (add `--follower_port` with the follower's COM
port):

```powershell
C:\Isaac-Sim\python.bat source\sim_to_real_so101\scripts\leader_arm_teleop_raw_isaacsim.py --follower_port COM3
```

### 3. What to expect

- It takes roughly **30-45 seconds** to boot. The window may say "Not Responding"
  during this time — that's normal (it's loading, not frozen); wait for the console
  log to settle before assuming something's wrong.
- Click once **inside the viewport window** so it has keyboard focus — this is needed
  for the reset hotkey to work.
- Watch the terminal for confirmation lines, for example:

  ```text
  [INFO]: Leader arm connected: port=COM4 id=my_so_arm
  [INFO]: Driving joints from the leader arm. Ctrl+C or close the window to stop.
  ```

  If you passed `--follower_port`, wait for **all three** of these before moving the
  leader arm:

  ```text
  [INFO]: Leader arm connected: port=COM4 id=my_so_arm
  [INFO]: Follower arm connected: port=COM3 id=my_so_arm
  [INFO]: Follower startup sync complete, entering normal mirroring.
  ```

  That last line matters — the script smoothly ramps the follower arm up to match the
  leader over about 1.5 seconds instead of snapping into position, so it doesn't
  jerk on startup.

- **Move the physical leader arm** — the simulated arm (and the real follower, if
  connected) copies it live.
- Press **`R`** (with the viewport focused) to reset the cube and bowl back to their
  starting positions if a grasp goes wrong.
- Press **`Ctrl+C`** in the terminal, or close the window, to stop. Both arms
  disconnect cleanly no matter how the run ends.

---

## Quick Reference

| Task | Command |
|---|---|
| Launch USD Composer | `cd C:\USD-Composer` then `.\repo.bat launch` |
| Launch Isaac Sim (standalone) | `C:\Isaac-Sim\isaac-sim.bat` |
| Run sim-only teleop | `C:\Isaac-Sim\python.bat source\sim_to_real_so101\scripts\leader_arm_teleop_raw_isaacsim.py` |
| Run sim + real follower teleop | `C:\Isaac-Sim\python.bat source\sim_to_real_so101\scripts\leader_arm_teleop_raw_isaacsim.py --follower_port COM3` |
| SO-101 robot asset (for Add Reference) | `usds\SO-ARM101-USD.usd` |

---

## Troubleshooting

| Problem | Likely cause | What to do |
|---|---|---|
| Composer/Isaac Sim window says "Not Responding" right after launch | Normal — Kit-based apps don't repaint the window while loading extensions/shaders | Wait it out; check the terminal log for progress instead of the window title |
| Scene looks 100x too big/small, or objects are lying on their side | New Composer scenes default to centimeters + Y-up; this repo's assets use meters + Z-up | Check the stage's Up Axis / Meters Per Unit in its properties |
| `[ERROR]: Failed to connect to the leader arm` | Arm is unplugged, powered off, on the wrong COM port, or still held open by a previous run that didn't close cleanly | Check the physical connection and `--port` (check Windows Device Manager → Ports (COM & LPT) for the right number), and make sure no other teleop window is still running |
| Teleop script exits immediately with a calibration error | `--robot_id`/`--follower_robot_id` doesn't match a saved calibration file | Check `calibration\teleoperators\so_leader\` and `calibration\robots\so_follower\` for the correct id (see Part 1, step 7) |
| `ModuleNotFoundError: No module named 'lerobot'` (or Isaac Sim crashes on startup with a native DLL/extension error) | The one-time `lerobot` + `torch`/`torchvision` install into Isaac Sim's Python (Part 1, step 6) hasn't been done on this PC, or the `torch`/`torchvision` versions don't match | Ask Claude Code to re-run the install using the prompt from Part 1, step 6 |

---

## Contributions

We are not currently accepting contributions for this project.
