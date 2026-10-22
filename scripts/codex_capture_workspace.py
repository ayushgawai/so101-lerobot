"""Read-only camera discovery and snapshots for the current physical task."""
import json
from pathlib import Path
import cv2

out = Path('.cache/codex_pick_place')
out.mkdir(parents=True, exist_ok=True)
for index in range(2):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    try:
        if not cap.isOpened():
            print(json.dumps({'camera': index, 'opened': False}), flush=True)
            continue
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        frame = None
        for _ in range(15):
            ok, image = cap.read()
            if ok:
                frame = image
        if frame is None:
            print(json.dumps({'camera': index, 'frame': False}), flush=True)
            continue
        path = out / f'camera_{index}.jpg'
        cv2.imwrite(str(path), frame)
        print(json.dumps({'camera': index, 'path': str(path), 'shape': frame.shape}), flush=True)
    finally:
        cap.release()
