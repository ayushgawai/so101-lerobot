# Joined Tables: Long-Edge Join + Realism Pass

This document records the work done to create `room-and-table_joined.usd`: two
copies of `/World/Table` joined along their long tabletop edges, then upgraded
from plain boxes/stick-legs into a realistic mobile classroom/lab table
design (rounded-corner laminate tops, trestle-style metal bases, caster
wheels).

Source scene: `source/sim_to_real_so101/assets/usd/room-and-table.usd` (untouched).
Output scene: `source/sim_to_real_so101/assets/usd/room-and-table_joined.usd`.
All work was done with the OpenUSD Python API (`pxr`) via the `usdenv`
throwaway venv (`usdenv\Scripts\python.exe`).

## Stage facts (established by inspection)

- `upAxis`: Z
- `metersPerUnit`: 1.0
- `/World/Table` and `/World/Table_02` are plain `Xform` prims authored
  directly in the root layer (no references/payloads/instancing).
- Original tabletop (`/World/Table/TableTop`, a `UsdGeom.Cube`): world bbox
  X `[-0.6, 0.6]` (1.2 m), Y `[-0.35, 0.35]` (0.7 m), Z `[0.70, 0.75]`
  (0.05 m thick), center `(0, 0, 0.725)`.
- Floor sits at Z = 0; room floor spans X, Y in `[-2, 2]`.
- Materials live under `/World/Looks` (`Floor`, `Wall`, `Mount`), each a
  `UsdShade.Material` with a child `Shader` (`UsdPreviewSurface`,
  `inputs:diffuseColor` / `metallic` / `roughness`), bound via
  `UsdShade.MaterialBindingAPI`.
- Mesh authoring convention: geometry authored directly (no instancing),
  `xformOp:translate` / `xformOp:orient` (`quatd`) / `xformOp:scale` on the
  mesh prim itself, `subdivisionScheme = none`, faceVarying normals,
  `UsdPhysics.CollisionAPI` applied, `extent` authored.

## Part 1 — Joining the two tables along their long edges

### Key finding: an earlier draft script had the join axis backwards

A pre-existing draft (`build_joined_tables.py`) translated `Table_02` by
**1.2 m along X** — the tabletop's *long* axis. That joins the tables
**short-edge-to-short-edge** (lengthens the assembly), which is exactly the
wrong result the task explicitly warned against ("do not translate along the
tabletop's longest dimension").

Fix: derive the long/short axes programmatically from the bounding box
instead of assuming, and translate along the **short axis (Y, 0.7 m)**
instead.

```text
long axis  (X): 1.2 m  -> long edges run parallel to X
short/join axis (Y): 0.7 m -> perpendicular translation axis
seam target: 3 mm (2-5 mm range)
translation = short_dim + seam = 0.7 + 0.003 = 0.703 m along +Y
```

### Scripts (`usd-composer-stages/`)

| Script | Purpose |
|---|---|
| `inspect_table.py` | Read-only dump of `/World/Table` hierarchy, transforms, bbox, materials, upAxis/metersPerUnit. |
| `build_joined_tables.py` | Duplicates `/World/Table` → `/World/Table_02` via `Sdf.CopySpec`, computes the long/short axes from the bbox, translates `Table_02` by `(0, 0.703, 0)`. Writes `room-and-table_joined.usd`. |
| `validate_joined_tables.py` | 19-point checklist: existence, dims/rotation/elevation match, seam in range, no tabletop overlap, leg preservation, no leg collisions, floor level, material bindings, no unrelated `/World` prims touched, reopens cleanly. |
| `edge_case_tests.py` | Extra checks: exact axis isolation of the translation (no X/Z drift), all 28 cross-table leg-pair combinations checked for intersection, seam measured at both ends of the long edge (rules out skew), combined-tables bbox fits inside the room floor, combined span confirms **wider** (Y roughly doubled) not **longer** (X unchanged). |
| `render_joined_preview.py` | Top-down matplotlib schematic (from real USD bboxes) to visually confirm the layout — saved as `joined_tables_preview.png`. |

### Result

```text
Original top bounds:  min (-0.6, -0.35, 0.7)   max (0.6, 0.35, 0.75)
Table_02 top bounds:  min (-0.6, 0.353, 0.7)   max (0.6, 1.053, 0.75)
Measured seam:         3.000 mm
Top elevation diff:    0 (exactly coplanar)
```

All 19 validation checks + all edge-case checks passed.

## Part 2 — Realism pass (materials + geometry)

Goal: turn the plain box tabletop + 4 stick legs into a believable mobile
classroom/lab table — light gray laminate top with rounded/beveled edges,
dark gray powder-coated metal trestle base, V/Y-shaped feet, black caster
wheels — **without** moving, rotating, or re-joining the tables (the 3 mm
long-edge seam from Part 1 had to be preserved exactly).

### Design parameters used

```text
Tabletop footprint:     1.2 x 0.7 m   (UNCHANGED — preserves the seam)
Tabletop thickness:     35 mm          (was 50 mm; target 25-40 mm)
Top surface elevation:  0.75 m         (UNCHANGED)
Corner radius:          35 mm          (target 20-50 mm)
Top edge bevel:         4 mm chamfer   (target 2-6 mm)

Support assemblies:     2 per table, inset from tabletop ends (x = +-0.40,
                         vs. tabletop ends at +-0.6)
Column:                 55 mm square tube, 4 mm edge radius
V-feet:                 45 mm square tube, spread to y = +-0.25
                         (well inside the 0.35 m tabletop half-depth,
                         so inner feet never approach the seam)
Casters:                4 per table (8 total), 90 mm wheel diameter,
                         32 mm width, small per-caster swivel variation
                         (+-6 to +-15 degrees about the vertical stem axis)

Materials:
  Laminate  (tabletop):  RGB (0.80, 0.79, 0.76), metallic 0, roughness 0.55
  Metal     (frame):     RGB (0.16, 0.17, 0.19), metallic 0.6, roughness 0.45
  CasterWheel:           RGB (0.03, 0.03, 0.03), metallic 0.1, roughness 0.6
```

### Approach

Because `/World/Table_02` had always been an exact `Sdf.CopySpec` duplicate
of `/World/Table` offset by `(0, 0.703, 0)`, the realism geometry was
authored **once** under `/World/Table`, then `/World/Table_02` was deleted
and rebuilt with a fresh `Sdf.CopySpec` + the same Y offset. This guarantees
the two tables are pixel-identical in design (per spec: "avoid manually
creating two subtly mismatched tables") and that the seam offset can't drift.

New hierarchy (mirrored under both `/World/Table` and `/World/Table_02`):

```text
Table/
├── TableTop                          (rounded rect, beveled top edge)
└── Base/
    ├── SupportLeft/
    │   ├── Column
    │   ├── MountPlate
    │   ├── FootA, FootB               (V-shaped, +Y / -Y)
    │   ├── CasterA/{Stem, Wheel}
    │   └── CasterB/{Stem, Wheel}
    └── SupportRight/  (same layout, mirrored in X)
```

### Procedural mesh generation (`build_realistic_tables.py`)

No pre-made assets were available, so all new geometry is generated
procedurally with a small set of reusable helpers, matching the existing
file's raw-mesh authoring convention (no references/instancing):

- **`rounded_rect_loop`** — builds a rounded-rectangle 2D profile (arc
  segments per corner; degenerates cleanly to a sharp rectangle at
  `radius = 0`).
- **`build_rounded_box`** — extrudes that profile into a solid box with an
  optional top/bottom chamfer bevel (inset ring between the outer top edge
  and a smaller flat top cap). Used for the tabletop, columns, feet,
  mounting plates, and caster stems.
- **`build_cylinder`** — a capped cylinder (axis along local Y) used for
  caster wheels.
- **`face_normals`** — computes per-face normals via Newell's method
  directly from the authored points, so shading is correct regardless of
  manual winding-order bookkeeping.
- **`quat_between`** — computes the `quatd` rotation that aligns a local +Z
  axis to an arbitrary world direction, used to orient the diagonal V-feet
  between two 3D points.
- **`create_preview_material`** / **`bind_material`** — authors a
  `UsdShade.Material` + `UsdPreviewSurface` child shader and binds it,
  matching the existing `/World/Looks/Floor` etc. pattern.

All new meshes are `doubleSided = True` (a deliberate small deviation from
the existing `Leg1`-style convention of `doubleSided = False`) as a
robustness measure for hand-authored geometry, so no face is ever culled by
a backface-culling renderer even if a winding-order edge case was missed.

### Preserving the seam through the geometry change

The tabletop footprint (1.2 x 0.7 m) and top-surface elevation (Z = 0.75)
were kept byte-identical to the pre-realism file; only the *thickness* and
*corner treatment* changed, both of which are inset from the existing
outer bounding box rather than expanding it. That meant the 3 mm seam from
Part 1 required no re-computation or compensation — `Table_02`'s translate
was simply re-applied at the same `(0, 0.703, 0)` after the `Sdf.CopySpec`
rebuild.

### Validation (`validate_realistic_tables.py`)

45 checks, all passing, covering:

- Existence of both tables/tabletops and the full `Base/Support*` hierarchy.
- Seam still 3.00 mm; long (X) edges still aligned end-to-end.
- Tops still coplanar; table rotations still match.
- Tabletop footprint unchanged (1.2 x 0.7 m); thickness in the realistic
  25–40 mm range; corners demonstrably rounded (26 unique X/Y profile
  coordinates vs. 2 for a sharp box); top has a distinct beveled-edge
  profile (3 Z-levels).
- Both tables have exactly 2 support assemblies and 4 casters each.
- All 8 caster wheels touch the floor at exactly Z = 0 (none floating,
  none penetrating).
- **Zero intersections across all 256 checked part-pairs** between the two
  tables' support/caster geometry, and no support structure intersects the
  other table's tabletop.
- Material colors match spec (light gray laminate, dark gray metal,
  near-black wheels); both tables share the same tabletop material.
- No unrelated `/World` prim (`Floor`, `Walls`, `RobotMount`, `SkyLight`,
  `KeyLight`) or pre-existing `/World/Looks` material was modified.
- Stage reopens with zero composition errors; `defaultPrim` still `/World`;
  both tables still sit at floor level (Z = 0); table height still 0.75 m.

`usdchecker` was not available in the `usd-core` venv used for this work
(`usdenv`), so that specific external check could not be run; the manual
validation above covers the same ground (composition errors captured on
reopen were empty).

### Visual confirmation (`render_realistic_preview.py`)

Since no interactive renderer was available in this environment, a small
matplotlib-based renderer was used to shade and display the **actual
authored USD mesh geometry** (real points/faces/materials, not a mockup),
producing:

- `joined_tables_iso.png` — isometric view of both tables together.
- `joined_tables_front.png` — front elevation showing the trestle silhouette.
- `joined_tables_underside.png` — underside view showing all 8 casters,
  V-feet, and open legroom.
- `joined_tables_seam_closeup.png` — close-up on the two inner support
  assemblies nearest the seam, confirming visually (and numerically) that
  they don't collide.

All four confirm the result reads as two independent mobile lab tables —
light gray laminate tops, dark trestle bases, black caster wheels — pushed
together tightly along their long edges with a clean, visible seam.

## File inventory

```text
source/sim_to_real_so101/assets/usd/
├── room-and-table.usd              (source, untouched)
└── room-and-table_joined.usd       (output — two joined, realistic tables)

usd-composer-stages/
├── inspect_table.py                 (read-only inspection of the source table)
├── build_joined_tables.py           (Part 1: duplicate + long-edge join)
├── validate_joined_tables.py        (Part 1: 19-point validation)
├── edge_case_tests.py               (Part 1: extra edge-case checks)
├── render_joined_preview.py         (Part 1: top-down schematic render)
├── build_realistic_tables.py        (Part 2: procedural realism pass)
├── validate_realistic_tables.py     (Part 2: 45-point validation)
├── render_realistic_preview.py      (Part 2: shaded 3D preview renders)
├── joined_tables_preview.png
├── joined_tables_iso.png
├── joined_tables_front.png
├── joined_tables_underside.png
└── joined_tables_seam_closeup.png
```

## How to reproduce

```bash
usdenv\Scripts\python.exe usd-composer-stages\build_joined_tables.py
usdenv\Scripts\python.exe usd-composer-stages\validate_joined_tables.py
usdenv\Scripts\python.exe usd-composer-stages\edge_case_tests.py
usdenv\Scripts\python.exe usd-composer-stages\build_realistic_tables.py
usdenv\Scripts\python.exe usd-composer-stages\validate_realistic_tables.py
usdenv\Scripts\python.exe usd-composer-stages\render_realistic_preview.py
```
