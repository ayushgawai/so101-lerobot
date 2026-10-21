# AWS Builder Loft Promo Cube

This document records the work done to add a small AWS Builder Loft
promotional cube to the digital-twin demo scene, and — the part worth
remembering — the diagnosis of a rendering bug that looked exactly like an
unwanted drop shadow but wasn't one.

Source scene: `source/sim_to_real_so101/demo/real-to-sim.usd` (untouched).
Output scene: `source/sim_to_real_so101/demo/room-and-table-with-aws-cube.usd`.
Build script: `usd-composer-stages/build_aws_builder_cube.py`, run with the
`usdenv` throwaway venv (`usdenv\Scripts\python.exe`), which has `pxr`
(usd-core) and Pillow.

## Stage facts (established by inspection)

- `upAxis`: Z, `metersPerUnit`: 1.0.
- `/World` references `../assets/usd/indoor-room.usd`; room/table geometry is
  pulled in by reference, not authored directly in `real-to-sim.usd`.
- Two tabletops exist: `/World/Table/TableTop` (the original, at the world
  origin, with the SO-101 `RobotMount` marker beneath it) and
  `/World/Table_02/TableTop` (a second table joined along +Y). The cube is
  placed on `/World/Table` — the primary/original one — chosen generically by
  a tabletop-finder that picks the shortest matching path when more than one
  tabletop-like prim exists, rather than a hard-coded path.
- No `Camera` prims are authored; Kit's saved viewport bookmarks
  (`customLayerData["cameraSettings"]`) show the default Perspective camera
  sits in the +X/+Y/+Z octant looking back at the origin. Decorated faces are
  assigned so `+X` = "AWS Builder Loft" text, `+Y` = AWS logo, `+Z` (top) =
  pixel smiley — the three faces that default view actually sees.
- Existing material convention: `UsdPreviewSurface` shaders under
  `/World/Looks/*` (matches the cube's own material).

## Cube construction

- Six textures (AWS Builder Loft text, AWS logo + swoosh, `builder.aws.com`,
  pixel heart, pixel smiley, pixel cloud) generated procedurally with Pillow
  and packed into one 2048x3072 texture atlas (2 cols x 3 rows, 1024 px/face)
  rather than six separate files.
- Cube mesh: 8-point box, `faceVarying` UVs (one unique UV per face-corner)
  so each face maps cleanly to its own atlas cell with no stretching/mirroring.
- `UsdPhysics.CollisionAPI` + `MeshCollisionAPI` (`convexHull`) always
  applied; `RigidBodyAPI` + `MassAPI` only when `ENABLE_RIGID_BODY = True`
  (density-based mass, ABS/PLA ~1200 kg/m^3).
- Hierarchy: `/World/AWSBuilderCube/{Geometry/AWSBuilderCube_Geo,
  Looks/AWSBuilderCube_Material}`.

## The "shadow" that wasn't a shadow

After the first build, the user reported a dark curved shape under the cube
in the Isaac Sim viewport and asked to have it removed.

**Wrong turns (2 iterations, no visible effect):**

1. Assumed it was a real cast shadow from `KeyLight`/`SkyLight` and added the
   cube to each light's `UsdLux` `shadowLink` exclude collection. This is
   valid core USD schema, but **Omniverse's Hydra Storm/RTX render delegates
   do not implement arbitrary shadow-linking collections** — the change was
   correctly authored but silently had no effect in Kit.
2. Learned (via NVIDIA's own docs) that Kit's actual per-object switch is the
   `primvars:doNotCastShadows` bool — the attribute behind the "Add >
   Rendering > Set Do Not Cast Shadows" menu action. Authored it on the
   cube's geometry. Still no visible effect after a reload.

**Root cause, found by actually rendering the scene:** neither fix worked
because it was never a shadow. `usd-core` (the pip package in `usdenv`) has
no Hydra/imaging libraries, so it can't render — but Isaac Sim is installed
locally at `C:\Isaac-Sim`, with its own bundled Python
(`C:\Isaac-Sim\python.bat`) that has the real thing. A small headless
standalone script (`isaacsim.sensors.experimental.rtx.RtxCamera` +
`CameraSensor`, camera aimed close at the cube from roughly the screenshot's
angle, ~180 `simulation_app.update()` frames to let RTX accumulate) rendered
the actual output file through the actual RTX pipeline and saved a PNG.

The render showed the "shadow" was a **solid black, inverted-normal-looking
patch baked into the cube's own geometry** at the bottom corner — not
anything to do with lights at all. Cause: the cube mesh was an 8-point box
with `subdivisionScheme = catmullClark` plus edge creases (used to fake a
small manufactured-plastic bevel). That coarse-cage-plus-crease combination
broke in Kit's RTX/Hydra Storm delegate at the corner, regardless of crease
sharpness value — tightening `EDGE_CREASE_SHARPNESS` to 9.5 and additionally
authoring matching `cornerSharpnesses` (the textbook OpenSubdiv fix for
under-constrained valence-3 corners) made no difference.

**Fix, verified by re-rendering:** set `subdivisionScheme = "none"` (a flat,
sharp-edged box, no creases at all). Re-rendered the identical scene/camera —
the black patch was completely gone, table visible underneath with only a
normal, faint, physically-correct contact shadow. The two shadow-suppression
workarounds (`shadowLink` exclusion, `doNotCastShadows`) were then removed
from the script since the real defect was fixed and a small physical object
casting a normal shadow is the more realistic result anyway.

### Takeaways

- **When a `usd-core` pip venv can't render, check for a full Isaac Sim
  install before guessing at fixes.** `C:\Isaac-Sim\python.bat` plus
  `isaacsim.sensors.experimental.rtx.RtxCamera`/`CameraSensor` gives a real,
  scriptable, headless RTX render in under 20 seconds once Kit is warm — cheap
  enough to use as a verification step rather than trusting a diagnosis on
  reasoning alone.
- **`UsdLux` shadow-linking collections are not honored by Omniverse's RTX
  render delegate.** For per-object shadow control in Kit, the attribute that
  actually matters is `primvars:doNotCastShadows` (same one written by
  Property panel > Add > Rendering > Set Do Not Cast Shadows).
- **A dark shape near an object's base is not automatically a lighting bug.**
  Here it was a mesh/subdivision rendering defect that merely looked like a
  shadow. Isolating cause required an actual render, not just reasoning about
  USD attributes.
- Catmull-Clark subdivision + edge creases on a bare 8-point/6-quad box cage
  is a fragile way to fake a small bevel — it broke in Kit's RTX delegate at
  the corners independent of sharpness tuning. A flat, unsubdivided box is
  more reliable; real chamfer geometry (extra edge-loop geometry instead of
  creases) would be the way to get an actual small bevel back without
  depending on subdivision.
