"""Inspect /World/Table and /World/Table_02 TableTop prims in real-to-sim.usd:
type, UVs, material bindings, shared vs separate materials.

Run with Isaac Sim's own Python (needs pxr):
    C:\\Isaac-Sim\\python.bat usd-composer-stages\\inspect_tabletop_real_to_sim.py
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdShade, Sdf

STAGE_PATH = (
    r"c:\Users\OMNI-User\Desktop\Sim-to-Real-SO-101-Workshop\source"
    r"\sim_to_real_so101\demo\real-to-sim.usd"
)

stage = Usd.Stage.Open(STAGE_PATH)
assert stage, f"Failed to open {STAGE_PATH}"

for table_name in ["Table", "Table_02"]:
    print(f"\n=== /World/{table_name} ===")
    table_prim = stage.GetPrimAtPath(f"/World/{table_name}")
    print("exists:", table_prim.IsValid())
    if not table_prim.IsValid():
        continue
    for child in table_prim.GetChildren():
        print("  child:", child.GetPath(), child.GetTypeName())

    top_prim = stage.GetPrimAtPath(f"/World/{table_name}/TableTop")
    print(f"  TableTop exists: {top_prim.IsValid()}")
    if not top_prim.IsValid():
        continue
    print("  TableTop typeName:", top_prim.GetTypeName())
    print("  TableTop specifier:", top_prim.GetSpecifier())

    # UVs
    primvars_api = UsdGeom.PrimvarsAPI(top_prim)
    for pv in primvars_api.GetPrimvars():
        print("  primvar:", pv.GetName(), pv.GetTypeName(), pv.GetInterpolation())

    # material binding
    binding_api = UsdShade.MaterialBindingAPI(top_prim)
    bound_mat, rel = binding_api.ComputeBoundMaterial()
    print("  bound material:", bound_mat.GetPath() if bound_mat else None)

    # attributes
    if top_prim.GetTypeName() == "Mesh":
        mesh = UsdGeom.Mesh(top_prim)
        pts = mesh.GetPointsAttr().Get()
        print("  point count:", len(pts) if pts else 0)
        print("  subdivisionScheme:", mesh.GetSubdivisionSchemeAttr().Get())
    elif top_prim.GetTypeName() == "Cube":
        print("  size:", top_prim.GetAttribute("size").Get())

    xf = UsdGeom.Xformable(top_prim)
    for op in xf.GetOrderedXformOps():
        print("  xformOp:", op.GetOpName(), op.Get())

print("\n=== /World/Looks children ===")
looks = stage.GetPrimAtPath("/World/Looks")
if looks.IsValid():
    for child in looks.GetChildren():
        print("  ", child.GetPath(), child.GetTypeName())
        if child.GetTypeName() == "Material":
            shader_prim = None
            for gc in child.GetChildren():
                print("    ->", gc.GetPath(), gc.GetTypeName())
                if gc.GetTypeName() == "Shader":
                    shader_prim = gc
            if shader_prim:
                shader = UsdShade.Shader(shader_prim)
                for inp in shader.GetInputs():
                    src = inp.GetConnectedSources()
                    print("      input:", inp.GetBaseName(), "=", inp.Get(), "connected:", bool(src[0]) if src else False)

simulation_app.close()
