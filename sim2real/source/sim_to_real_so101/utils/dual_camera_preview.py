"""Two non-blocking camera views inside Kit; no OpenCV GUI dependency."""
import threading
import time

import cv2
import numpy as np
import omni.ui as ui


class DualCameraPreview:
    def __init__(self, indices=(1, 0)):
        self.stop_event = threading.Event()
        self.frames = [None, None]
        self.timestamps = [0., 0.]
        self.errors = ["Opening camera", "Opening camera"]
        self.threads = []
        self.providers = [ui.ByteImageProvider(), ui.ByteImageProvider()]
        self.window = ui.Window("Real cameras: fixed / wrist", width=680, height=310)
        self.labels = []
        with self.window.frame:
            with ui.HStack():
                for i, name in enumerate(("Fixed", "Wrist")):
                    with ui.VStack():
                        self.labels.append(ui.Label(f"{name}: camera {indices[i]}", height=24))
                        ui.ImageWithProvider(self.providers[i], width=320, height=240)
        for slot, index in enumerate(indices):
            thread = threading.Thread(target=self._capture, args=(slot, index), daemon=True)
            self.threads.append(thread)
            thread.start()

    def _capture(self, slot, index):
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        try:
            if not cap.isOpened():
                self.errors[slot] = f"Cannot open camera {index}"
                return
            while not self.stop_event.is_set():
                ok, frame = cap.read()
                if ok:
                    self.frames[slot] = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
                    self.timestamps[slot] = time.monotonic()
                    self.errors[slot] = ""
                else:
                    self.errors[slot] = "No camera frame"
                    self.stop_event.wait(.1)
        finally:
            cap.release()

    def update(self):
        for i, name in enumerate(("Fixed", "Wrist")):
            frame = self.frames[i]
            age = time.monotonic()-self.timestamps[i]
            if frame is not None:
                thumbnail = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)
                self.providers[i].set_bytes_data(np.ascontiguousarray(thumbnail).ravel().tolist(), [320, 240])
            status = self.errors[i] or (f"STALE {age:.1f}s" if age > 1 else "live")
            self.labels[i].text = f"{name}: {status} (preview only)"

    def close(self):
        self.stop_event.set()
        for thread in self.threads:
            thread.join(timeout=1)
        self.window.visible = False
