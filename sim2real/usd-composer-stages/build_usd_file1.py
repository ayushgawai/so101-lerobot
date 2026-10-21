"""Authors `source/sim_to_real_so101/demo/usd-file1.usd`: the environment-only
half of `real-to-sim.usd`, per docs/real-to-sim-environment-prompt.md.

Keeps the room shell, the hidden robot mount disc, both lights, the six
materials, and both lab tables. Drops the SO-ARM101 robot, the AWS Builder
Cube, the paper bowl, and their paper labels.

Rather than re-deriving the meshes from the numbers in the prompt (which were
themselves measured off `real-to-sim.usd`), this flattens the source stage and
copies the surviving prim specs verbatim, so the geometry is identical to the
digital twin rather than a close rebuild. The reference to
`assets/usd/indoor-room.usd` is composed away in the process -- the output is a
single self-contained layer, which is what the prompt asks for.

Run with:
    usdenv\Scripts\python.exe usd-composer-stages\build_usd_file1.py
"""
import os

from pxr import Sdf, Usd, UsdGeom

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
DEMO_DIR = os.path.join(REPO_ROOT, "source", "sim_to_real_so101", "demo")

SRC_PATH = os.path.join(DEMO_DIR, "real-to-sim.usd")
OUT_PATH = os.path.join(DEMO_DIR, "usd-file1.usd")

# /World children that belong to the task objects, not the environment.
EXCLUDED = ["AWSBuilderCube", "SO_ARM101_USD", "PaperBowl", "AWSCubePaper"]

# Kit's viewport cameras live as `over`s at stage root; Kit's own session layer
# supplies the `def Camera`. Copying them keeps the authoring views intact.
VIEWPORT_CAMERAS = ["OmniverseKit_Persp", "OmniverseKit_Front",
                    "OmniverseKit_Right", "OmniverseKit_Top"]

# Product 1 framed the robot's wrist camera, which this stage no longer has.
FALLBACK_CAMERA = Sdf.Path("/OmniverseKit_Persp")


def relativize_texture_paths(layer):
    """Flattening resolves `@./tex/...@` to absolute paths. Put them back, so
    the stage stays portable alongside demo/tex/."""
    demo_prefix = DEMO_DIR.replace(os.sep, "/").rstrip("/") + "/"
    fixed = [0]

    def visit(path):
        if not path.IsPropertyPath():
            return
        spec = layer.GetAttributeAtPath(path)
        if spec is None or spec.typeName != Sdf.ValueTypeNames.Asset:
            return
        value = spec.default
        if value is None:
            return
        resolved = value.path.replace(os.sep, "/")
        if resolved.startswith(demo_prefix):
            spec.default = Sdf.AssetPath("./" + resolved[len(demo_prefix):])
            fixed[0] += 1

    layer.Traverse(Sdf.Path("/"), visit)
    return fixed[0]


def main():
    src_stage = Usd.Stage.Open(SRC_PATH)
    flat = src_stage.Flatten()

    if os.path.exists(OUT_PATH):
        os.remove(OUT_PATH)
    # .usd with a usda-format arg: Composer reads it fine and it stays diffable.
    out_layer = Sdf.Layer.CreateNew(OUT_PATH, args={"format": "usda"})

    for spec_path in ["/World", "/Render"] + ["/" + c for c in VIEWPORT_CAMERAS]:
        path = Sdf.Path(spec_path)
        if flat.GetPrimAtPath(path) is None:
            continue
        if not Sdf.CopySpec(flat, path, out_layer, path):
            raise RuntimeError(f"CopySpec of {spec_path} failed")

    for name in EXCLUDED:
        path = Sdf.Path("/World").AppendChild(name)
        if out_layer.GetPrimAtPath(path) is not None:
            del out_layer.GetPrimAtPath(path).nameParent.nameChildren[name]

    # Retarget the render product that pointed at the (now absent) wrist camera.
    products = "/Render/OmniverseKit/HydraTextures"
    for product in out_layer.GetPrimAtPath(products).nameChildren:
        rel = product.relationships.get("camera")
        if rel is None:
            continue
        targets = list(rel.targetPathList.explicitItems)
        if targets and out_layer.GetPrimAtPath(targets[0]) is None:
            rel.targetPathList.explicitItems = [FALLBACK_CAMERA]

    # Stage metadata: the prompt pins all of these. Layer metadata is not
    # carried by CopySpec, so it has to be re-authored here.
    out_layer.pseudoRoot.SetInfo(UsdGeom.Tokens.upAxis, UsdGeom.Tokens.z)
    out_layer.pseudoRoot.SetInfo(UsdGeom.Tokens.metersPerUnit, 1.0)
    out_layer.defaultPrim = "World"
    out_layer.startTimeCode = -1
    out_layer.endTimeCode = 0
    out_layer.customLayerData = {
        "cameraSettings": dict(flat.customLayerData.get("cameraSettings", {}),
                               boundCamera=str(FALLBACK_CAMERA)),
        "omni_layer": {"authoring_layer": "./usd-file1.usd", "locked": {}, "muteness": {}},
        "renderSettings": {},
    }
    out_layer.documentation = (
        "Environment-only extraction of real-to-sim.usd: room shell, lights, "
        "materials and both lab tables, without the robot or the task objects.\n"
    )

    n_textures = relativize_texture_paths(out_layer)
    out_layer.Save()

    # --- verify against the composed result -------------------------------
    stage = Usd.Stage.Open(OUT_PATH)
    assert UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z
    assert abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) < 1e-9
    assert stage.GetDefaultPrim().GetPath() == Sdf.Path("/World")

    print(f"Wrote {OUT_PATH} ({os.path.getsize(OUT_PATH) / 1024:.0f} KB)")
    print(f"Texture asset paths made relative: {n_textures}")
    print("Root prims:", [p.GetName() for p in stage.GetPseudoRoot().GetChildren()])
    print("/World:", [p.GetName() for p in stage.GetPrimAtPath("/World").GetChildren()])
    print("/World/Looks:", [p.GetName() for p in stage.GetPrimAtPath("/World/Looks").GetChildren()])

    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    for path in ["/World/Floor", "/World/Table", "/World/Table_02"]:
        r = bbox.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
        print(f"{path}: min={tuple(round(v, 4) for v in r.GetMin())} "
              f"max={tuple(round(v, 4) for v in r.GetMax())}")


if __name__ == "__main__":
    main()
