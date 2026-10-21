"""Inspect the Walls prim(s) and material in real-to-sim.usd.

Run with Isaac Sim's own Python:
    C:\\Isaac-Sim\\python.bat usd-composer-stages\\inspect_walls.py
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdShade

STAGE_PATH = (
    r"c:\Users\OMNI-User\Desktop\Sim-to-Real-SO-101-Workshop\source"
    r"\sim_to_real_so101\demo\real-to-sim.usd"
)

stage = Usd.Stage.Open(STAGE_PATH)
assert stage, f"Failed to open {STAGE_PATH}"

print("=== Searching for prims with 'wall' in name/path ===")
for prim in stage.Traverse():
    path_str = str(prim.GetPath())
    if "wall" in path_str.lower():
        print(prim.GetPath(), "-", prim.GetTypeName())
        for attr in prim.GetAttributes():
            if attr.HasAuthoredValue():
                print("   ", attr.GetName(), "=", attr.Get())
        rel = prim.GetRelationship("material:binding")
        if rel and rel.IsValid():
            targets = rel.GetTargets()
            print("    material:binding ->", targets)

print()
print("=== /World/Looks children ===")
looks = stage.GetPrimAtPath("/World/Looks")
if looks.IsValid():
    for child in looks.GetChildren():
        print(child.GetPath(), "-", child.GetTypeName())

simulation_app.close()
