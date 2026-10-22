# Screenshot-to-USD: technical specification

Derived from [visual observations](screenshot-scene-visual-description.md) and
[the measurement/placement reference](real-to-sim-environment-prompt.md).
The JSON block below is the build script's input, not an illustrative duplicate.
Paths in it are relative to `sim2real/`.

## Ruler-photo revision (2026-10-21)

Evidence: `IMG_2204.jpeg` (base/cube ruler), `IMG_2205.jpeg` (cube/bowl
ruler), and `IMG_2206.jpeg` (overall setup). Readings below are approximate
photo interpretations, not surveyed measurements. The ruler zero is partly
occluded and raised objects introduce parallax. Allow about +/-10 mm for the
base gap and +/-5 mm for the lateral gap, with potentially larger systematic
error from perspective and datum identification.

| Quantity | Photo interpretation | Authored nominal |
|---|---|---|
| Base-plate forward edge to cube near edge | Cube contact edge near ruler 19 cm; zero assumed at visible plate tip | 190 mm |
| Cube edge to bowl near outer rim | Cube lower silhouette near 2 cm; near rim near 13 cm | 110 mm |
| Bowl outer span along cube-to-bowl direction | Near rim about 13 cm, far rim about 27.5 cm | 145 mm |
| Bowl perpendicular outer span | Approximately square in top view; no perpendicular ruler | 150 mm estimate |
| Cube side | Retained prior 55 mm size; elevated faces cannot establish a new side length reliably | 55 mm |
| Cube-to-bowl center distance | 27.5 + 110 + 72.5 mm | 210 mm |

The overview shows the robot clamped to the near edge, the cube forward of the
base, and the bowl to the robot's right at approximately the same forward
distance. Retain the existing stage convention: robot forward is -Y, its right
is +X, and up is +Z. The overview's near table edge therefore maps to +Y.
The current scene mirrors the bowl to the robot's **left** (-X), as subsequently
requested, while preserving the photographed 210 mm center spacing and 110 mm gap.
The robot root remains (0, 0.300, 0.720). The native base plate visual's forward
bound is local Y=-0.0488747604, world Y=0.2511252396. Matching that bound to
the photographed ruler datum is an explicit modeling assumption; the photos
do not locate the joint-axis/root origin. No robot rotation is inferred from
the apparent orientation of the bent arm.

The cube center is (0, 0.0336252396, 0.7775):
`0.300 - 0.0488747604 - 0.190 - 0.055/2` gives its Y coordinate.
The bowl bottom origin is (-0.210, 0.0336252396, 0.750):
`-(0.055/2 + 0.110 + 0.145/2)` gives its X coordinate. Aligning their forward
centers and aligning the cube with robot root X=0 are approximate layout
assumptions. This places the cube center 266.4 mm forward of the robot root.

The bowl's unmeasured height remains 65 mm. Its bottom footprint is reduced
proportionally to 110 x 115 mm, with 18 mm corner radius; these are estimates.
Keep the existing room, tables, native robot, materials, rigid cube and open
compound bowl colliders. Camera positions remain presentation views, not
calibrated reconstructions of the physical cameras. The camera stand, cables,
electronics and exact articulated pose in the overview are outside this layout
revision. Existing rendered previews predate this revision and must be regenerated
before using them to assess the updated layout. The local source photos are
not bundled as dependencies of the USD.

## Coordinate system and measured anchors

Meters; Z up; `/World` default prim; +X along table length; +Y toward the second
table. Ground surface is Z=0. Both tabletop surfaces are Z=0.750. Table 1 is
centered at XY=(0,0); table 2 at (0,0.703), retaining the measured 3 mm gap.
Use the repository's existing table meshes to preserve the detailed chamfer,
splayed supports and caster geometry. The 4 m room floor and four wall positions
come directly from the reference. There is no ceiling.

| Element | Dimensions (m) | World placement (m) | Evidence |
|---|---|---|---|
| Floor | 4 × 4 × 0.10 | center (0,0,-0.05), top 0 | Reference |
| North/South walls | 4 × 0.10 × 2.50 | centers (0,±2,1.25) | Reference |
| East/West walls | 0.10 × 4 × 2.50 | centers (±2,0,1.25) | Reference |
| Each table | 1.20 × 0.70; surface 0.750 high | (0,0,0), (0,0.703,0) | Reference |
| Cube visual | 0.055 on each axis; 0.002 corner radius | center (0,0.0336252396,0.7775) | Prior size; ruler-derived XY |
| Container | top 0.145 × 0.150; height 0.065 | bottom origin (-0.210,0.0336252396,0.75) | Ruler-derived span/gap; left side requested; remaining sizes estimated |
| Robot | native asset, unit scale | root (0,0.3,0.72), identity rotation | Reference; asset inspected |

The old 0.7754 m cube center belongs to another cube and would embed the new
55 mm cube by 2.1 mm. The corrected center is 0.750 + 0.055/2 = **0.7775 m**.
The robot's base geometry begins around local Z=0.03008 m; its root Z=0.72 m is
therefore compatible with the table. The clamp intentionally extends below the
top. Preserve the reference identity root rotation (the standalone robot asset
otherwise carries a +90-degree Z rotation).

## Geometry, surface and contact decisions

| Surface | Visual treatment | Roughness | Contact treatment |
|---|---|---|---|
| Table | gray-beige directional wood grain, matching edge band | 0.62 | Existing static table mesh colliders |
| Frame | dark gunmetal | 0.45 | Existing static colliders |
| Wheels | nearly black | 0.60 | Existing static colliders |
| Cube | white paper exterior, shallow creases and raised tape strips over hard-rubber PVC | 0.80; tape 0.30 | Rigid body, compliant paper contact; 0.06 kg assumption |
| Container | milky translucent flared shell, rounded lip, black B/red dot paper label | 0.22 | Static compound of bottom and four walls; open top |
| Room | neutral gray floor, pale warm walls; existing texture maps | 0.80 | Static floor and four wall colliders |
| Robot | off-white printed structure, native dark servos and metal | 0.48 for plastic | Retain native articulation and colliders |

The container tapers from a 0.110 × 0.115 m bottom footprint to a 0.145 × 0.150 m
outer rim, with a 0.002 m wall and 0.003 m bottom. Only the rim's X span is
ruler-constrained; the other dimensions are explicit **demo estimates**.
A watertight shell connects the outer bottom, outer side, rounded lip, inner side
and inner bottom. Separate trapezoidal convex wall colliders and a bottom collider
preserve the cavity, including its rounded corners. A single convex hull over the
whole container would incorrectly seal its opening and must not be used.

The white label is a 65 mm square inside the container, with vector geometry for
the black B and red dot. Its placement is visual only, with no extra collider.
Container opacity 0.34 and IOR 1.47 are illustrative USD Preview Surface settings.
Kit/RTX also gets a frosted transmission surface using the built-in `OmniGlass`
MDL template with IOR 1.47 and roughness 0.22; the template name does not identify
the real material as glass. These are appearance approximations, not a measured
polymer transmission/scattering model. Renderer settings affect the result.
Parameter names follow [NVIDIA's OmniGlass reference](https://docs.omniverse.nvidia.com/materials-and-rendering/latest/templates/OmniGlass.html).

The real object is PVC wrapped in white paper and should behave like **hard
rubber**: it keeps its shape when grasped, with only a slight surface give. The
cube is a rigid body (assumed mass 60 g) with a convex-hull collider, and its paper
material uses PhysX compliant contact: stiffness 5000 N/m and damping 30 N s/m
(just under critical for 60 g). A ~5 N jaw squeeze yields about 1 mm of contact
give; self-weight sinks the cube about 0.1 mm. Static friction 1.1 and dynamic
friction 0.9 use the `max` combine mode. These are grasp-tuning estimates, not
measured material properties.

A volume-deformable FEM cube was evaluated first. In this PhysX build it only held
a lift once the pads sank 4-10 mm into it, which made it visibly bulge like a gel
(about 3.5 % lateral bulge at 1 MPa; 13 % compression at 50 kPa). At hard-rubber
stiffness (0.5-3 MPa) with a ~1 mm squeeze the cube slipped out. Compliant
contact gives the firm feel without the gel-like deformation.

The jaw and fixed gripper collision hierarchy bind a dedicated material with
static friction 1.1, dynamic friction 0.9 and zero restitution. Their contact
offset is 1 mm and rest offset zero. The jaw uses stiffness 2.0, damping 0.2
and a 0.35 N m effort cap to reduce ejection from excessive squeeze. A fully
closed command does not require the jaws to reach zero opening through the cube:
contact and material stiffness determine the actual opening and deformation.
The scene uses a 120 Hz TGS physics step with 64 solver iterations.
`usd-composer-stages/validate_paper_cube_grasp.py` (Isaac Sim, no hardware) checks
that the cube keeps its shape, holds a 90 mm pinch-lift, releases and resets.

The robot pose is the supplied asset pose, not a
reconstruction of the pose in the photo and not a programmed pick-and-place task.
Desk clamps and lights are retained/recreated for the visual demonstration;
contact behavior of the full articulation has not been dynamically commissioned.

## Executable specification

```json
{
  "output": "source/sim_to_real_so101/demo/codex-pick-place-table.usd",
  "table_source": "source/sim_to_real_so101/demo/usd-file1.usd",
  "reference_stage": "source/sim_to_real_so101/demo/real-to-sim.usd",
  "robot_asset": "source/sim_to_real_so101/assets/usd/SO-ARM101-USD.usd",
  "meters_per_unit": 1.0,
  "up_axis": "Z",
  "table_surface_z": 0.75,
  "photo_measurements": {
    "base_edge_to_cube_edge_m": 0.190,
    "cube_edge_to_bowl_rim_m": 0.110,
    "sources": ["IMG_2204.jpeg", "IMG_2205.jpeg", "IMG_2206.jpeg"]
  },
  "tables": [[0, 0, 0], [0, 0.703, 0]],
  "floor": {"center": [0, 0, -0.05], "size": [4, 4, 0.1]},
  "walls": {
    "North": {"center": [0, 2, 1.25], "size": [4, 0.1, 2.5]},
    "South": {"center": [0, -2, 1.25], "size": [4, 0.1, 2.5]},
    "East": {"center": [2, 0, 1.25], "size": [0.1, 4, 2.5]},
    "West": {"center": [-2, 0, 1.25], "size": [0.1, 4, 2.5]}
  },
  "wood": {"grid": [240, 112], "roughness": 0.62, "color": [0.48, 0.45, 0.39]},
  "cube": {
    "center": [0, 0.0336252396, 0.7775], "side": 0.055, "corner_radius": 0.002,
    "wrinkle_amplitude": 0.00012, "mass_kg": 0.06,
    "color": [0.88, 0.88, 0.84], "roughness": 0.80,
    "static_friction": 1.1, "dynamic_friction": 0.9, "restitution": 0.0,
    "model": "rigid_compliant", "contact_stiffness_n_per_m": 5000.0, "contact_damping_ns_per_m": 30.0
  },
  "gripper_contact": {
    "static_friction": 1.1, "dynamic_friction": 0.9,
    "stiffness": 2.0, "damping": 0.2, "effort_limit_nm": 0.35
  },
  "container": {
    "origin": [-0.210, 0.0336252396, 0.75], "top_size": [0.145, 0.150],
    "bottom_size": [0.110, 0.115], "height": 0.065,
    "wall_thickness": 0.002, "bottom_thickness": 0.003,
    "corner_radius": 0.018, "rim_radius": 0.003,
    "label_side": 0.065, "color": [0.95, 0.96, 0.96],
    "roughness": 0.22, "opacity": 0.34, "ior": 1.47
  },
  "robot": {"origin": [0, 0.3, 0.72], "printed_color": [0.84, 0.84, 0.79]},
  "wrist_camera": {
    "position": [-0.005, 0.06, -0.062],
    "rotation_xyz_deg": [-45, 0, 0],
    "focal_length": 13.5,
    "horizontal_aperture": 20.955,
    "vertical_aperture": 15.71625,
    "clipping_range": [0.005, 100]
  },
  "lights": {
    "dome_intensity": 800, "key_intensity": 6000,
    "key_position": [0, 0, 2.45], "key_size": [1.5, 1.5], "temperature_kelvin": 6500
  },
  "cameras": {
    "DemoCamera": {"eye": [0.90, -1.10, 1.38], "target": [0.06, 0.15, 0.84], "focal_length": 42},
    "RoomCamera": {"eye": [-1.65, -1.70, 2.15], "target": [0, 0.35, 0.65], "focal_length": 24},
    "ObjectsCamera": {"eye": [0.40, -0.46, 1.30], "target": [0.12, 0.04, 0.79], "focal_length": 55}
  }
}
```

## Rebuild and present

From the repository root, using Python with `usd-core` and `numpy` installed:

```powershell
uv venv .venv-usd-demo --python 3.12
uv pip install --python .venv-usd-demo/Scripts/python.exe usd-core==26.11 numpy==2.5.4
.venv-usd-demo/Scripts/python.exe sim2real/usd-composer-stages/build_screenshot_room_demo.py
```

The builder reads this specification, authors an ASCII USD layer, then reopens it
and checks the table bounds, floor/wall anchors, cube contact, open-container
geometry, robot meshes/joints, referenced assets and mesh topology. A JSON
validation report is written beside the stage. Existing source stages are inputs.
Generated asset paths are relative; keep the scene inside this repository or
collect its dependencies before moving it elsewhere. It is not a standalone
single-file bundle.

Open `source/sim_to_real_so101/demo/codex-pick-place-table.usd` in USD Composer or
Isaac Sim. Select `/World/Cameras/DemoCamera` for the full tabletop,
`ObjectsCamera` for cube/container detail, or `RoomCamera` for the enclosing room.
Start with the timeline stopped to present the authored scene.

For the moving wrist viewpoint, select **WristCamera** in the viewport camera
menu: `/World/SO_ARM101_USD/gripper/WristCamera`. It is parented to the gripper
rigid link and follows wrist/arm motion, including during raw teleoperation.
Its local position (-0.005, 0.060, -0.062) m and rotation (-45, 0, 0) degrees
match `camera_ego` in `tasks/task_env_cfg.py`. USD cameras look along local -Z
with +Y up. Approximate pinhole optics use focal length 13.5, aperture
20.955 x 15.71625 (USD camera units), a 4:3 aspect ratio and roughly 75.6-degree
horizontal field of view; near clipping is 5 mm and depth of field is disabled.
These are a simulated viewpoint, not measured real-camera intrinsics. What
the wrist sees depends on the current arm pose; it does not track the cube
independently. The default overview camera remains `DemoCamera`.

Three 1280 × 720 Isaac Sim previews accompany the stage:
[tabletop](../source/sim_to_real_so101/demo/screenshot-room-demo-preview.png),
[objects](../source/sim_to_real_so101/demo/screenshot-room-demo-objects.png), and
[room](../source/sim_to_real_so101/demo/screenshot-room-demo-room.png).
Regenerate them using an installed Isaac Sim Python launcher:

```powershell
C:/Isaac-Sim/python.bat sim2real/usd-composer-stages/render_screenshot_room_demo.py
```

OpenUSD validation distinguishes local assets from renderer MDL modules. Core USD
cannot resolve Kit's built-in `OmniGlass.mdl`/`OmniPBR.mdl` or the native robot's
Omniverse MDL URL; these are listed separately in the JSON report. Kit rendering
is checked independently. Every new material has a USD Preview fallback.

Demo narration: **four photographs → visual observations → explicit dimensions,
materials and assumptions → executable specification → USD geometry, materials,
physics annotations and cameras**. This is a dimension-constrained reconstruction
for a demo, not metrically recovered photogrammetry or a calibrated soft-body twin.
