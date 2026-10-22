"""Finite, no-hardware test: settle, pinch, lift, release the paper-wrapped hard-rubber cube.

Uses symmetric kinematic test pads, without attachments, pose latching or USB I/O.
Run with C:/Isaac-Sim/python.bat, never a normal usd-core Python.
"""
import argparse
import json
from pathlib import Path
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--capture", action="store_true")
parser.add_argument("--settle-only", action="store_true")
parser.add_argument("--youngs-modulus", type=float)
parser.add_argument("--convex-table", action="store_true")
parser.add_argument("--closed-gap", type=float, default=.053, help="pad gap (m) when squeezed: ~1mm compliant-contact give per side on the 55mm cube")
args = parser.parse_args()
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
exit_code = 0
metrics = {}
root = Path(__file__).resolve().parents[1]
report = root / "source/sim_to_real_so101/demo/codex-pick-place-table.grasp-validation.json"
try:
    import numpy as np
    import omni.usd
    import omni.timeline
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade
    sys.path.insert(0, str(root / "source"))
    from sim_to_real_so101.utils.paper_cube import soft_cube_world_bounds, restore_soft_cube_points

    omni.usd.get_context().open_stage(str(report.with_name("codex-pick-place-table.usd")))
    for _ in range(30):
        app.update()
    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    if args.youngs_modulus:
        stage.GetPrimAtPath("/World/Looks/WhitePolymer").GetAttribute("omniphysics:youngsModulus").Set(args.youngs_modulus)
    if args.convex_table:
        UsdPhysics.MeshCollisionAPI.Apply(stage.GetPrimAtPath("/World/Table/TableTop")).CreateApproximationAttr("convexHull")
    stage.GetPrimAtPath("/World/SO_ARM101_USD").SetActive(False)
    center = np.asarray(stage.GetPrimAtPath("/World/Cube").GetAttribute("xformOp:translate").Get())
    metrics["test_fixture"] = "Simulated parallel kinematic pads, not a full SO-101 trajectory; no attachment/latch and no physical hardware"
    metrics["material_youngs_modulus_pa"] = stage.GetPrimAtPath("/World/Looks/WhitePolymer").GetAttribute("omniphysics:youngsModulus").Get()
    pads = []
    for name, sign in (("Left", -1), ("Right", 1)):
        pad = UsdGeom.Cube.Define(stage, "/World/TestPads/"+name)
        pad.CreateSizeAttr(1.)
        translation = pad.AddTranslateOp()
        translation.Set(Gf.Vec3d(*(center+np.array([sign*.05, 0, 0]))))
        pad.AddScaleOp().Set(Gf.Vec3f(.04, .08, .08))
        pad.CreateDisplayColorAttr([(.15, .18, .22)])
        UsdPhysics.CollisionAPI.Apply(pad.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(pad.GetPrim()).CreateKinematicEnabledAttr(True)
        UsdShade.MaterialBindingAPI.Apply(pad.GetPrim()).Bind(
            UsdShade.Material(stage.GetPrimAtPath("/World/Looks/GripperPaperContact")), materialPurpose="physics")
        pads.append((sign, translation))

    timeline = omni.timeline.get_timeline_interface()
    def steps(count, gap=.09, height=0):
        for _ in range(count):
            for sign, translation in pads:
                translation.Set(Gf.Vec3d(*(center+np.array([sign*(gap+.04)/2, 0, height]))))
            app.update()

    def sample(name):
        bounds = soft_cube_world_bounds(stage)
        lo, hi = np.asarray(bounds.GetMin()), np.asarray(bounds.GetMax())
        if not np.isfinite([lo, hi]).all():
            raise RuntimeError("Nonfinite cube vertices")
        metrics[name] = {"center_m": ((lo+hi)/2).tolist(), "size_m": (hi-lo).tolist()}
        print("[SOFT_CUBE]", name, metrics[name], flush=True)
        for mesh_name in ("SimulationMesh", "CollisionMesh"):
            prim = stage.GetPrimAtPath("/World/Cube/"+mesh_name)
            if not prim:
                continue
            points = np.asarray(prim.GetAttribute("points").Get())
            if points.ndim == 2 and points.size:
                print("[TETS]", name, mesh_name, len(points), points.min(axis=0), points.max(axis=0), flush=True)

    def capture(name, gap):
        if not args.capture:
            return
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        viewport = get_active_viewport()
        viewport.camera_path = "/World/Cameras/ObjectsCamera"
        matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*(center+np.array([.20, -.30, .22]))), Gf.Vec3d(*center), Gf.Vec3d(0, 0, 1)).GetInverse()
        stage.GetPrimAtPath(viewport.camera_path).GetAttribute("xformOp:transform").Set(matrix)
        steps(15, gap)
        capture_viewport_to_file(viewport, str(report.with_name("paper-cube-"+name+".png")))
        steps(20, gap)

    sample("rest")
    timeline.play()
    steps(90)
    sample("settled")
    if args.settle_only:
        stage.Export(str(root.parent / ".cache/paper-cube-probe.usda"))
        metrics["status"] = "settle_probe"
        raise SystemExit(0)
    capture("rest", .09)
    closed = args.closed_gap
    for i in range(120):
        steps(1, .09+(closed-.09)*(i+1)/120)
    steps(90, closed)
    sample("compressed")
    capture("compressed", closed)
    for i in range(120):
        steps(1, closed, .09*(i+1)/120)
    steps(90, closed, .09)
    sample("lifted")
    for i in range(90):
        steps(1, closed+(.1-closed)*(i+1)/90, .09)
    steps(120, .1, .09)
    sample("released")

    settled, compressed, lifted, released = (metrics[name] for name in ("settled", "compressed", "lifted", "released"))
    side = .055
    # Hard rubber: negligible self-weight sag, small bulge when pinched, elastic recovery.
    assert settled["size_m"][2] > side*.985, "Cube sags under its own weight (too soft)"
    assert args.closed_gap < settled["size_m"][0]-.001, "Pads did not load the cube"
    assert max(compressed["size_m"][1:]) < max(settled["size_m"][1:])*1.03, "Cube bulges like a gel (too soft)"
    assert lifted["center_m"][2] > settled["center_m"][2]+.06, "Cube slipped out during lift"
    assert released["size_m"][0] > settled["size_m"][0]*.985, "Cube did not recover after release"
    assert abs(released["center_m"][2]-center[2]) < .012, "Released cube did not settle on table"
    timeline.stop()
    app.update()
    restore_soft_cube_points(stage)
    timeline.play()
    steps(90, .1, .09)
    sample("reset")
    assert abs(metrics["reset"]["center_m"][2]-center[2]) < .01, "FEM reset did not settle correctly"
    metrics["status"] = "passed"
    print("[PASS] Hard-rubber cube keeps its shape, holds a 90mm lift, releases and resets; no attachments or hardware", flush=True)
except Exception as exc:
    metrics["status"] = "failed"
    metrics["error"] = str(exc)
    print(f"[FAIL] {type(exc).__name__}: {exc}", flush=True)
    exit_code = 1
finally:
    report.write_text(json.dumps(metrics, indent=2)+"\n", encoding="utf-8")
    app.close(exit_code=exit_code)
