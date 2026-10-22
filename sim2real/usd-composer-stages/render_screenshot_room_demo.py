"""Render the USD demo's three cameras with an installed Isaac Sim (timeline stopped).

Example from the repository root:
    C:/Isaac-Sim/python.bat sim2real/usd-composer-stages/render_screenshot_room_demo.py
"""

from pathlib import Path

from isaacsim import SimulationApp


app = SimulationApp({"headless": True, "width": 1280, "height": 720,
                     "renderer": "RaytracedLighting", "anti_aliasing": 3,
                     "extra_args": ["--/app/asyncRendering=false"]})

try:
    import carb
    import omni.usd
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

    root = Path(__file__).resolve().parents[1]
    scene = root / "source/sim_to_real_so101/demo/codex-pick-place-table.usd"
    settings = carb.settings.get_settings()
    settings.set("/persistent/app/viewport/displayOptions", 0)
    settings.set("/rtx/post/tonemap/op", 6)
    settings.set("/rtx/translucency/enabled", True)
    settings.set("/rtx/raytracing/fractionalCutoutOpacity", True)
    omni.usd.get_context().open_stage(str(scene))
    for _ in range(100):
        app.update()
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("No viewport available in this Isaac Sim experience")
    viewport.set_texture_resolution((1280, 720))
    for camera, filename in [("DemoCamera", "screenshot-room-demo-preview.png"),
                             ("ObjectsCamera", "screenshot-room-demo-objects.png"),
                             ("RoomCamera", "screenshot-room-demo-room.png")]:
        viewport.camera_path = "/World/Cameras/" + camera
        for _ in range(60):
            app.update()
        result = capture_viewport_to_file(viewport, str(scene.parent / filename))
        for _ in range(45):
            app.update()
        print("RENDERED", camera, scene.parent / filename, flush=True)
finally:
    app.close()
