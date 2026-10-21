# USD Composer Recreation Prompt — `real-to-sim` Environment

Environment-only extraction from `source/sim_to_real_so101/demo/real-to-sim.usd`.
**Excluded:** the SO-ARM101 robot, the AWS Builder Cube, the paper bowl, and their paper labels.

---

## Prompt

Build a USD stage in USD Composer / Omniverse recreating a small indoor robotics-lab room with two
identical rolling lab tables. All units are **meters**. Follow the numbers exactly — this scene is a
digital twin of a real workspace, so dimensions and placements are load-bearing.

### 1. Stage metadata

- `upAxis = "Z"`
- `metersPerUnit = 1`
- `defaultPrim = "World"`
- `startTimeCode = -1`, `endTimeCode = 0`
- Root prim: `def Xform "World"` — everything below lives under `/World`.

### 2. Room shell

The floor and walls are **unit cubes (1×1×1, centered at origin) converted to Mesh** — 8 points,
6 quad faces, `doubleSided = true`, `subdivisionScheme = "none"`, and **face-varying `primvars:st`
that runs 0→1 on each of the six faces**. They are then scaled to size. Each carries
`PhysicsCollisionAPI` and `MaterialBindingAPI`.

> Note: because the UVs are 0–1 per face (not tiled), the floor/wall textures **stretch across the
> full 4 m span** rather than repeating, even though the texture shaders are set to `repeat`. Keep
> this if you want to match the original look; rescale the UVs if you'd rather tile.

| Prim | translate | scale | resulting box | material |
|---|---|---|---|---|
| `/World/Floor` | `(0, 0, -0.05)` | `(4, 4, 0.1)` | 4×4 m slab, top face at **z = 0** | `Floor` |
| `/World/Walls/North` | `(0, 2, 1.25)` | `(4, 0.1, 2.5)` | 4 m wide, 0.1 m thick, 2.5 m tall | `Wall` |
| `/World/Walls/South` | `(0, -2, 1.25)` | `(4, 0.1, 2.5)` | same | `Wall` |
| `/World/Walls/East` | `(2, 0, 1.25)` | `(0.1, 4, 2.5)` | same, running along Y | `Wall` |
| `/World/Walls/West` | `(-2, 0, 1.25)` | `(0.1, 4, 2.5)` | same | `Wall` |

`xformOpOrder = ["xformOp:translate", "xformOp:scale"]` on all five.

Room interior is therefore **4 × 4 m of floor, walls rising to z = 2.5 m**, wall inner faces at
x = ±1.95 and y = ±1.95.

### 3. Robot mount disc (hidden)

- `/World/RobotMount` — `Cylinder`, `axis = "Z"`, `radius = 0.6`, `height = 0.02`
- `xformOp:translate = (0, 0, 0.01)` (sits flush on the floor)
- material `Mount`, `PhysicsCollisionAPI`
- **`visibility = "invisible"`** — it is authored but hidden in the final scene. Keep it as a
  placement/collision reference.

### 4. Lighting

Two lights only, both direct children of `/World`:

| Prim | Type | Parameters |
|---|---|---|
| `/World/SkyLight` | `DomeLight` | `intensity = 800`, `enableColorTemperature = true`, `colorTemperature = 6500` (no HDRI texture — flat dome) |
| `/World/KeyLight` | `RectLight` | `width = 1.5`, `height = 1.5`, `intensity = 6000`, `translate = (0, 0, 2.45)` (ceiling-height softbox pointing down at the tables) |

### 5. Materials — `/World/Looks`

All six are `UsdPreviewSurface`. Textured materials use the pattern:
`UsdPrimvarReader_float2 (varname = "st") → st` of each `UsdUVTexture`, then
`DiffuseTexture.rgb → diffuseColor`, `RoughnessTexture.r → roughness`,
`NormalTexture.rgb → normal`. Normal maps use `bias = (-1,-1,-1,0)`, `scale = (2,2,2,1)`,
`sourceColorSpace = "raw"`; diffuse uses `sRGB`; roughness uses `raw`.

| Material | diffuseColor | roughness | metallic | Textures (relative to the USD) |
|---|---|---|---|---|
| `Floor` | `(0.55, 0.55, 0.58)` | 0.8 | 0.0 | `./tex/floor/floor_{diffuse,roughness,normal}.png`, wrap **repeat** |
| `Wall` | `(0.85, 0.84, 0.80)` | 0.8 | 0.0 | `./tex/wall/wall_{diffuse,roughness,normal}.png`, wrap **repeat** |
| `Laminate` | `(0.5149177, 0.5149177, 0.47353148)` | 0.62 | 0.0 (`opacity = 1.0`) | `./tex/table/table_{diffuse,roughness,normal}.png`, wrap **clamp** |
| `Metal` | `(0.16, 0.17, 0.19)` | 0.45 | 0.6 | none — plain dark gunmetal |
| `CasterWheel` | `(0.03, 0.03, 0.03)` | 0.6 | 0.1 | none — near-black rubber |
| `Mount` | `(0.9, 0.55, 0.1)` | 0.8 | 0.0 | none — orange, only used by the hidden disc |

Textures live in `demo/tex/{floor,wall,table}/` and must be copied alongside the stage.

### 6. Table (`/World/Table`)

A rolling lab table, **1.2 m × 0.7 m top at 0.75 m working height**, on two splayed-leg supports
with four casters. Everything is authored as explicit polygon meshes (no primitive shapes), each
with `doubleSided = true`, `subdivisionScheme = "none"`, `orientation = "rightHanded"`,
`purpose = "default"`, and both `PhysicsCollisionAPI` + `MaterialBindingAPI`.
All transforms use `xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]`.

**Root** `/World/Table`: identity transform (translate `(0,0,0)`, orient `(1,0,0,0)`, scale `(1,1,1)`).

#### 6a. `TableTop` — material `Laminate`

- Local extent: `(-0.6, -0.35, 0)` → `(0.6, 0.35, 0.035)`
- `translate = (0, 0, 0.715)` → **top surface lands at z = 0.750**
- Shape: a rounded rectangle **1.2 × 0.7 m with ~35 mm corner radius**, swept as a 24-segment
  profile (72 points, 50 faces) in three loops:
  - z = 0.000 — full profile
  - z = 0.031 — full profile (vertical side wall, 31 mm)
  - z = 0.035 — profile inset by 4 mm (a **4 mm × 4 mm top chamfer** running all the way around)
- UVs map the top face across the full 0–1 range so `table_diffuse.png` reads as one laminate sheet.

#### 6b. `Base` — two supports at x = ±0.4

`/World/Table/Base/SupportLeft` at **x = −0.4**, `/World/Table/Base/SupportRight` at **x = +0.4**.
The two are mirror-identical apart from the caster swivel angles. Each contains:

| Part | Local extent (size) | translate | orient (quat wxyz) | material |
|---|---|---|---|---|
| `Column` | `(±0.0275, ±0.0275, 0…0.545)` — **55 × 55 mm rounded-square post, 545 mm tall** (8-point profile, ~4 mm corner round) | `(±0.4, 0, 0.16)` → spans z 0.160 → 0.705 | identity | `Metal` |
| `MountPlate` | `(±0.07, ±0.05, 0…0.01)` — **140 × 100 × 10 mm rounded plate** | `(±0.4, 0, 0.705)` → z 0.705 → 0.715, caps the column under the top | identity | `Metal` |
| `FootA` | `(±0.0225, ±0.0225, 0…0.255)` — **45 × 45 mm square bar, 255 mm long** | `(±0.4, 0, 0.16)` | `(0.6339889, -0.7733421, 0, 0)` = **rotateX −101.31°** → splays toward **+Y** and downward | `Metal` |
| `FootB` | same bar | `(±0.4, 0, 0.16)` | `(0.6339889, +0.7733421, 0, 0)` = **rotateX +101.31°** → splays toward **−Y** | `Metal` |

Each foot ends in a caster, as a child scope `CasterA` (at **+Y**) / `CasterB` (at **−Y**):

| Part | Local extent (size) | translate | orient | material |
|---|---|---|---|---|
| `Stem` | `(±0.015, ±0.015, 0…0.0515)` — **30 × 30 mm rounded post, 51.5 mm tall** | `(±0.4, ±0.25, 0.0585)` | identity | `Metal` |
| `Wheel` | `(±0.045, ±0.016, ±0.045)` — **disc, radius 45 mm, 32 mm thick, spin axis = Y**, 16-sided (32 points, 18 faces) | `(±0.4, ±0.25, 0.045)` → **bottom of wheel touches z = 0** | small swivel about Z (see below) | `CasterWheel` |

Caster swivel angles (purely cosmetic — the casters are turned slightly differently so the table
doesn't look mechanically perfect):

| Caster | rotateZ | quat (w,x,y,z) |
|---|---|---|
| SupportLeft / CasterA | **+8°** | `(0.99756405, 0, 0, 0.06975647)` |
| SupportLeft / CasterB | **−12°** | `(0.99452190, 0, 0, -0.10452846)` |
| SupportRight / CasterA | **+14°** | `(0.99254615, 0, 0, 0.12186934)` |
| SupportRight / CasterB | **−6°** | `(0.99862953, 0, 0, -0.05233596)` |

**Resulting table bounds:** x ∈ [−0.6, 0.6], y ∈ [−0.35, 0.35], z ∈ [0, 0.75].

### 7. Second table (`/World/Table_02`)

An **exact duplicate** of `/World/Table` — same hierarchy, same meshes, same caster angles — with
only the root transform changed:

```
xformOp:translate = (0, 0.703, 0)
xformOp:orient    = (1, 0, 0, 0)
xformOp:scale     = (1, 1, 1)
```

The two tables are butted together along +Y with a **3 mm gap** (each top is 0.7 m deep, offset
0.703 m), forming one continuous **1.2 m × 1.403 m** work surface at z = 0.75.

### 8. Viewport cameras (optional, Kit-authored)

Four `/OmniverseKit_*` cameras at stage root, `xformOpOrder = [translate, rotateXYZ, scale]`:

- **Persp** — `translate = (-0.0561, -0.5554, 1.4005)`, `rotateXYZ = (51.130, 0, -1.555)`,
  `focalLength = 18.148`, `clippingRange = (0.01, 1e7)`, `focusDistance = 400`,
  `fStop = 5.0`, `exposure:time = 0.02`, `exposure:responsivity = 1.1027`.
  Looks down at the tables from the −Y side — this is the authoring view.
- **Front** — orthographic, `translate = (5, 0, 0)`, `rotateXYZ = (90, 0, 90)`, apertures 50/50
- **Right** — orthographic, `translate = (0, -5, 0)`, `rotateXYZ = (90, 0, 0)`, apertures 50/50
- **Top** — orthographic, `translate = (0, 0, 5)`, `rotateXYZ = (0, 0, -90)`, apertures 50/50

### 9. Render settings (optional)

`/Render/OmniverseGlobalRenderSettings` drives two `RenderProduct`s, both with:

- `omni:rtx:rendermode = "RealTimePathTracing"`
- `omni:rtx:background:source:type = "domeLight"`, `textureMode = "repeatMirrored"`
- `omni:rtx:rt:ambientLight:color = (0.1, 0.1, 0.1)`
- all denoisers off (`ambientOcclusion`, `indirectDiffuse`, `reflections`), DLSS frame-gen off,
  `dlss:execMode = "performance"`, `pt:adaptiveSampling:fixedSppIterations = 2`
- second product renders at **1280 × 720** into the robot's wrist camera; output var `LdrColor`

---

## Placement anchors (for reference only — not part of this build)

Where the excluded assets sit, so the recreated environment lines up with them later:

| Asset | translate |
|---|---|
| SO-ARM101 robot base | `(0, 0.3, 0.72)` |
| AWS Builder Cube | `(0, 0.03, 0.7754)` |
| Paper bowl | `(0.2, 0.03, 0.75)` |
| AWS cube paper label | `(0, 0.03, 0.7504)` |

The robot mounts on the near table's far edge; the manipulated objects sit near the front edge on
the table top at z ≈ 0.75.

---

## Source-file structure note

In the original, `/World` carries `prepend references = @../assets/usd/indoor-room.usd@`, which
supplies `Floor`, `Walls`, `RobotMount`, `SkyLight`, `KeyLight`, and the `Floor`/`Wall`/`Mount`
materials. `real-to-sim.usd` then adds the tables, the texture-shader networks on the referenced
materials (as `over`s), and overrides `RobotMount` to `invisible`. For a from-scratch rebuild in
USD Composer you can author everything in one flat layer — the section order above already does that.
