"""Headless initial workspace registration; never opens motors or Isaac Sim.

Example (run from sim2real):
    python source/sim_to_real_so101/scripts/align_dual_camera_workspace.py \
        --config calibration/workspaces/measured_cameras.json --output aligned.json

Optional --frames fixed=PATH wrist=PATH reads each image in a directory once,
or each frame of a video once. A single image provides only ONE observation
and cannot satisfy the five-observation minimum. Offline output is explicitly
tagged offline_frames and must not be represented as a live alignment.
"""

import argparse
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import cv2

from sim_to_real_so101.utils.dual_camera_alignment import (
    AlignmentError, CAMERAS, WorkspaceAlignment, load_config,
)


IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


class FrameSource:
    def __init__(self, camera, offline_path=None):
        self.capture = None
        self.images = None
        self.offline = offline_path is not None
        if self.offline:
            path = Path(offline_path)
            if not path.exists():
                raise AlignmentError(f"Offline frame path does not exist: {path}")
            if path.is_dir():
                images = sorted(p for p in path.iterdir() if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES)
                if not images:
                    raise AlignmentError(f"No supported images in {path}")
                self.images = iter(images)
            elif path.suffix.lower() in IMAGE_SUFFIXES:
                self.images = iter([path])
            else:
                self.capture = cv2.VideoCapture(str(path))
        else:
            self.capture = cv2.VideoCapture(camera.index)
        if self.capture is not None:
            if not self.capture.isOpened():
                self.capture.release()
                raise AlignmentError(f"Cannot open {'video ' + str(offline_path) if self.offline else 'camera index ' + str(camera.index)}")
            if not self.offline:
                width, height = camera.intrinsics.frame_size
                self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def read(self):
        if self.images is not None:
            path = next(self.images, None)
            if path is None:
                return None
            frame = cv2.imread(str(path))
            if frame is None:
                raise AlignmentError(f"Cannot decode image {path}")
            return frame
        ok, frame = self.capture.read()
        if not ok and not self.offline:
            raise AlignmentError("Camera frame read failed; check connection and camera ownership")
        return frame if ok else None

    def close(self):
        if self.capture is not None:
            self.capture.release()


def parse_frame_paths(entries):
    if entries is None:
        return None
    paths = {}
    for entry in entries:
        name, separator, path = entry.partition("=")
        if separator != "=" or name not in CAMERAS or not path or name in paths:
            raise AlignmentError("--frames requires exactly fixed=PATH wrist=PATH")
        paths[name] = path
    if set(paths) != set(CAMERAS):
        raise AlignmentError("--frames must supply BOTH fixed=PATH and wrist=PATH")
    return paths


def collect_alignment(config, timeout, frame_paths=None):
    alignment = WorkspaceAlignment(config)
    sources = {}
    exhausted = set()
    last_diagnostics = {}
    deadline = time.monotonic() + timeout
    last_report = 0.0
    last_sample = {name: -math.inf for name in CAMERAS}
    print("[ALIGN] Keep cube/container and base marker stationary. Each accepted image needs base 2 plus object 0/1.", flush=True)
    try:
        for name in CAMERAS:
            sources[name] = FrameSource(config.cameras[name], frame_paths[name] if frame_paths else None)
        while not alignment.complete and time.monotonic() < deadline:
            for name in CAMERAS:
                if name in exhausted:
                    continue
                if all(count >= config.thresholds.min_samples_per_camera for count in alignment.progress()[name].values()):
                    exhausted.add(name)
                    continue
                frame = sources[name].read()
                if frame is None:
                    exhausted.add(name)
                    continue
                now = time.monotonic()
                if now >= deadline:
                    break
                # Drain live capture even between accepted exposures.
                if frame_paths is None and now - last_sample[name] < 0.2:
                    continue
                last_sample[name] = now
                last_diagnostics[name] = alignment.observe(name, frame)
                if "accepted" in last_diagnostics[name].values():
                    print(f"[ALIGN] {name}: {alignment.progress()[name]}", flush=True)
            now = time.monotonic()
            if now - last_report >= 1.0:
                print(f"[ALIGN] {alignment.progress()} | {last_diagnostics}", flush=True)
                last_report = now
            if len(exhausted) == len(CAMERAS):
                break
        if not alignment.complete:
            raise AlignmentError(f"Timeout or frames exhausted: {alignment.progress()}; last observations: {last_diagnostics}")
        return alignment.build_layout("offline_frames" if frame_paths else "live")
    finally:
        for source in sources.values():
            source.close()


def write_layout(path, layout):
    """Atomically publish only a fully validated result; preserve old output on failure."""
    path = Path(path)
    payload = json.dumps(layout, indent=2, allow_nan=False) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=path.name + ".", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=60.0, help="Capture deadline in seconds (default: 60)")
    parser.add_argument("--frames", nargs="+", metavar="CAMERA=PATH", help="Offline directories, images or videos for BOTH cameras")
    args = parser.parse_args(argv)
    try:
        if not math.isfinite(args.timeout) or args.timeout <= 0:
            raise AlignmentError("--timeout must be finite and positive")
        config = load_config(args.config)
        if args.output.resolve() in {args.config.resolve(), *(camera.intrinsics_path for camera in config.cameras.values())}:
            raise AlignmentError("--output must not overwrite configuration or intrinsics")
        paths = parse_frame_paths(args.frames)
        layout = collect_alignment(config, args.timeout, paths)
        write_layout(args.output, layout)
        print(f"[ALIGN] Wrote validated {'offline' if paths else 'live'} workspace: {args.output}", flush=True)
        return 0
    except (AlignmentError, OSError, cv2.error) as exc:
        print(f"[ALIGN ERROR] {exc}. No alignment output written.", file=sys.stderr, flush=True)
        return 1
    except KeyboardInterrupt:
        print("[ALIGN ERROR] Interrupted. No alignment output written.", file=sys.stderr, flush=True)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
