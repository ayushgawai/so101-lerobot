"""Click-to-measure placement error or gripper position from a recorded eval dataset's top camera.

--measure placement (default): for each episode, shows the final top_cam frame
(the cube at rest). You click the bowl center, then the cube center; the pixel
distance is converted to cm using a one-time scale calibration (two clicks
spanning a known distance, e.g. the bowl's outer rim diameter measured once
with a ruler). Results append to a CSV and a summary is printed for typing
into scoresheet column F.

--measure gripper: for each episode, auto-seeks to the first grasp attempt
(first gripper close after it first opens, from observation.state) and you
click the fingertip midpoint where the jaws meet the table plane. The pixel is
converted to a signed (x, y) cm offset from the workspace center in the same
frame as scoresheet column B, for the Random tab's "Gripper Pos" column.
Requires a one-time axis calibration: pass >=2 --ref TRIAL:X,Y (trials whose
column-B cube position you copy from the sheet); on each ref trial's first
frame you click the A-marker center, then the cube center. The solved
transform is stored in a .axes.json next to the CSV (delete it to redo).

Keys in the window:
  a / d   step 5 frames back / forward (occlusion, or fine-tune the grasp frame)
  r       clear clicks on this frame
  c       recalibrate scale (placement mode only; next 2 clicks span --calib-cm)
  s       save measurement and go to next episode
  n       skip episode (failed trial - no placement to measure)
  q       quit (already-saved rows are kept)

Usage:
  python scripts/measure_placement_error.py --calib-cm 11.4
  python scripts/measure_placement_error.py --repo-id aakashv100/eval_so101-pick-cube-v2-random --calib-cm 11.4
  python scripts/measure_placement_error.py --calib-cm 11.4 --start-episode 12   # resume
  python scripts/measure_placement_error.py --measure gripper \
      --repo-id aakashv100/eval_so101-pick-cube-v2-random --ref 1:1.4,-4.7 --ref 3:2.4,1.8
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

CAMERA = "observation.images.top_cam"
FRAME_STEP = 5


def load_episodes(root: Path):
    """Return [(episode_index, video_path, from_ts, to_ts)] from the meta parquets."""
    files = sorted((root / "meta" / "episodes").rglob("*.parquet"))
    if not files:
        raise FileNotFoundError(f"No episode metadata under {root}/meta/episodes")
    df = pd.concat([pd.read_parquet(f) for f in files]).sort_values("episode_index")
    episodes = []
    for _, row in df.iterrows():
        video = (root / "videos" / CAMERA
                 / f"chunk-{int(row[f'videos/{CAMERA}/chunk_index']):03d}"
                 / f"file-{int(row[f'videos/{CAMERA}/file_index']):03d}.mp4")
        episodes.append((int(row["episode_index"]),
                         video,
                         float(row[f"videos/{CAMERA}/from_timestamp"]),
                         float(row[f"videos/{CAMERA}/to_timestamp"])))
    return episodes


def read_frame(video_path: Path, ts: float):
    """Decode the frame at timestamp ts (seconds within the video file)."""
    cap = cv2.VideoCapture(str(video_path))
    cap.set(cv2.CAP_PROP_POS_MSEC, max(ts, 0) * 1000)
    ok, frame = cap.read()
    cap.release()
    return frame if ok else None


def put_text_lines(img, lines):
    for i, txt in enumerate(lines):
        cv2.putText(img, txt, (8, 22 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, txt, (8, 22 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 1, cv2.LINE_AA)


def draw_overlay(frame, clicks, scale, calibrating, ep_idx, offset):
    img = frame.copy()
    for x, y in clicks:
        cv2.drawMarker(img, (x, y), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
    header = f"ep {ep_idx} (trial {ep_idx + 1})  frame offset {offset}"
    if calibrating:
        status = "CALIBRATE: click 2 points spanning the known distance"
    elif scale is None:
        status = "no scale yet - press c to calibrate"
    elif len(clicks) == 0:
        status = "click BOWL CENTER"
    elif len(clicks) == 1:
        status = "click CUBE CENTER"
    else:
        (x1, y1), (x2, y2) = clicks
        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        err = math.dist(clicks[0], clicks[1]) * scale
        status = f"error = {err:.2f} cm   (s=save  r=redo  n=skip)"
    put_text_lines(img, (header, status))
    return img


def load_gripper_signal(root: Path):
    """Return {episode_index: gripper.pos array, one value per frame}."""
    files = sorted((root / "data").rglob("*.parquet"))
    if not files:
        raise FileNotFoundError(f"No data parquets under {root}/data")
    df = pd.concat([pd.read_parquet(f, columns=["episode_index", "observation.state"])
                    for f in files])
    return {int(ep): np.stack(grp["observation.state"].values)[:, -1]
            for ep, grp in df.groupby("episode_index")}


def find_grasp_frame(gripper):
    """Frame of the first grasp attempt: first close after the gripper first opens.

    The arm starts with the gripper shut (start pose), opens on approach, then
    closes onto the cube - so the first midpoint down-crossing is the grasp.
    """
    mid = (gripper.max() + gripper.min()) / 2
    above = np.nonzero(gripper > mid)[0]
    if len(above) == 0:
        return 0
    below = np.nonzero(gripper[above[0]:] < mid)[0]
    return int(above[0] + below[0]) if len(below) else 0


def parse_refs(refs):
    """Parse --ref TRIAL:X,Y strings into [(trial, x_cm, y_cm)]."""
    out = []
    for r in refs or []:
        trial, xy = r.split(":")
        x, y = xy.split(",")
        out.append((int(trial), float(x), float(y)))
    return out


def calibrate_axes(refs, episodes, win, clicks):
    """Solve the pixel -> table-frame (x, y) cm transform from reference trials.

    On each ref trial's first frame the user clicks the A-marker (workspace
    center), then the cube center, whose true position is the trial's column-B
    value. Solving the full 2x2 matrix from >=2 non-collinear refs fixes
    rotation, scale, and mirroring in one step. Returns (center_px, M).
    """
    by_ep = {ep: (video, f, t) for ep, video, f, t in episodes}
    centers, cube_px, targets = [], [], []
    for trial, x_cm, y_cm in refs:
        if trial - 1 not in by_ep:
            raise SystemExit(f"--ref trial {trial}: no episode {trial - 1} in dataset")
        video, from_ts, to_ts = by_ep[trial - 1]
        offset = 0
        clicks.clear()
        while True:
            frame = read_frame(video, min(from_ts + offset / 30, to_ts - 1 / 30))
            if frame is None:
                raise SystemExit(f"cannot decode first frame of trial {trial}")
            img = frame.copy()
            for x, y in clicks:
                cv2.drawMarker(img, (x, y), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
            status = ("click A-MARKER CENTER", "click CUBE CENTER",
                      "s=accept  r=redo  a/d=scrub")[min(len(clicks), 2)]
            put_text_lines(img, (f"AXIS CALIB trial {trial}: cube is at ({x_cm}, {y_cm}) cm",
                                 status))
            cv2.imshow(win, img)
            key = cv2.waitKey(30) & 0xFF
            if key == ord("q"):
                raise SystemExit("calibration aborted")
            elif key == ord("r"):
                clicks.clear()
            elif key == ord("a"):
                offset = max(0, offset - FRAME_STEP)
                clicks.clear()
            elif key == ord("d"):
                offset += FRAME_STEP
                clicks.clear()
            elif key == ord("s") and len(clicks) == 2:
                centers.append(clicks[0])
                cube_px.append(clicks[1])
                targets.append((x_cm, y_cm))
                break
    center = np.mean(centers, axis=0)
    V = (np.array(cube_px, float) - center).T   # 2xN pixel offsets from center
    T = np.array(targets, float).T              # 2xN cm
    v1, v2 = V[:, 0], V[:, 1]
    if abs(v1 @ v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)) > 0.9:
        print("WARNING: reference cubes nearly collinear with center - "
              "the solved axes will be unreliable; pick refs in different directions")
    M = T @ np.linalg.pinv(V)
    print(f"Axes calibrated from {len(refs)} refs: implied scale "
          f"{math.sqrt(abs(np.linalg.det(M))):.4f} cm/px, "
          f"max residual {np.abs(M @ V - T).max():.2f} cm")
    return tuple(center), M


def draw_gripper_overlay(frame, clicks, center, ep_idx, idx, gval, xy):
    img = frame.copy()
    cv2.drawMarker(img, (int(center[0]), int(center[1])), (255, 0, 0),
                   cv2.MARKER_CROSS, 18, 2)
    for x, y in clicks:
        cv2.drawMarker(img, (x, y), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
    gtxt = f"  gripper.pos {gval:.1f}" if gval is not None else ""
    header = f"ep {ep_idx} (trial {ep_idx + 1})  frame {idx}{gtxt}"
    if not clicks:
        status = "click GRIPPER FINGERTIP MIDPOINT (a/d to fine-tune the grasp frame)"
    else:
        status = f"gripper pos = ({xy[0]:.1f}, {xy[1]:.1f}) cm   (s=save  r=redo  n=skip)"
    put_text_lines(img, (header, status))
    return img


def run_gripper(args, root):
    name = args.repo_id.split("/")[-1]
    out_csv = Path(args.out) if args.out else Path(f"gripper_positions_{name}.csv")
    axes_file = out_csv.with_suffix(".axes.json")

    episodes = load_episodes(root)
    grippers = load_gripper_signal(root)
    print(f"{len(episodes)} episodes in {args.repo_id}")

    win = "gripper position"
    cv2.namedWindow(win)
    clicks = []
    cv2.setMouseCallback(
        win, lambda ev, x, y, flags, p: clicks.append((x, y))
        if ev == cv2.EVENT_LBUTTONDOWN and len(clicks) < 2 else None)

    if axes_file.exists():
        d = json.loads(axes_file.read_text())
        center, M = tuple(d["center_px"]), np.array(d["px_to_cm"])
        print(f"Loaded axes from {axes_file} (delete it to recalibrate)")
    else:
        refs = parse_refs(args.ref)
        if len(refs) < 2:
            raise SystemExit("No axes calibration yet - pass at least two "
                             "--ref TRIAL:X,Y (column-B positions from the scoresheet)")
        center, M = calibrate_axes(refs, episodes, win, clicks)
        axes_file.write_text(json.dumps(
            {"center_px": list(center), "px_to_cm": M.tolist(), "refs": refs}))
        print(f"Saved axes to {axes_file}")

    new_file = not out_csv.exists()
    csv_f = open(out_csv, "a", newline="")
    writer = csv.writer(csv_f)
    if new_file:
        writer.writerow(["episode", "trial", "x_cm", "y_cm", "gripper_px", "frame_index"])

    results = []
    i = args.start_episode
    while i < len(episodes):
        ep_idx, video, from_ts, to_ts = episodes[i]
        g = grippers.get(ep_idx)
        grasp = find_grasp_frame(g) if g is not None else 0
        n_frames = len(g) if g is not None else max(int((to_ts - from_ts) * 30), 1)
        offset = 0
        clicks.clear()
        while True:
            idx = min(max(grasp + offset, 0), n_frames - 1)
            frame = read_frame(video, min(from_ts + idx / 30, to_ts - 1 / 30))
            if frame is None:
                print(f"ep {ep_idx}: cannot decode frame {idx} in {video}")
                break
            xy = M @ (np.array(clicks[0], float) - center) if clicks else None
            cv2.imshow(win, draw_gripper_overlay(
                frame, clicks, center, ep_idx, idx,
                g[idx] if g is not None else None, xy))
            key = cv2.waitKey(30) & 0xFF

            if key == ord("q"):
                csv_f.close()
                print_gripper_summary(results, out_csv)
                return
            elif key == ord("r"):
                clicks.clear()
            elif key == ord("a"):
                offset -= FRAME_STEP
                clicks.clear()
            elif key == ord("d"):
                offset += FRAME_STEP
                clicks.clear()
            elif key == ord("n"):
                print(f"ep {ep_idx} (trial {ep_idx + 1}): skipped")
                break
            elif key == ord("s") and clicks:
                writer.writerow([ep_idx, ep_idx + 1, f"{xy[0]:.2f}", f"{xy[1]:.2f}",
                                 f"{clicks[0]}", idx])
                csv_f.flush()
                results.append((ep_idx + 1, xy[0], xy[1]))
                print(f"ep {ep_idx} (trial {ep_idx + 1}): ({xy[0]:.1f}, {xy[1]:.1f}) cm")
                break
        i += 1

    csv_f.close()
    cv2.destroyAllWindows()
    print_gripper_summary(results, out_csv)


def print_gripper_summary(results, out_csv):
    print(f"\nSaved {len(results)} measurements to {out_csv}")
    if results:
        print("For scoresheet column C (Gripper Pos):")
        for trial, x, y in results:
            print(f"  trial {trial:2d}: ({x:.1f}, {y:.1f})")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--repo-id", default="aakashv100/eval_so101-pick-cube-v2-fixed")
    ap.add_argument("--root", default=None,
                    help="Dataset root (default: HF cache for --repo-id).")
    ap.add_argument("--measure", choices=["placement", "gripper"], default="placement",
                    help="placement: bowl-to-cube distance on the final frame (column F). "
                         "gripper: grasp-attempt (x, y) cm from workspace center "
                         "(Random tab Gripper Pos column).")
    ap.add_argument("--calib-cm", type=float,
                    help="Known real-world distance for the 2 calibration clicks, "
                         "e.g. the bowl's outer rim diameter in cm. "
                         "Required for --measure placement.")
    ap.add_argument("--ref", action="append", metavar="TRIAL:X,Y",
                    help="Axis-calibration reference for --measure gripper: a trial number "
                         "and its column-B cube position, e.g. --ref 1:1.4,-4.7. Give >=2 "
                         "in different directions from center. Needed once per dataset; "
                         "the result is stored in a .axes.json.")
    ap.add_argument("--out", default=None,
                    help="Output CSV (default: placement_errors_<dataset>.csv or "
                         "gripper_positions_<dataset>.csv).")
    ap.add_argument("--start-episode", type=int, default=0)
    args = ap.parse_args()

    root = Path(args.root) if args.root else \
        Path.home() / ".cache" / "huggingface" / "lerobot" / args.repo_id
    if not root.exists():
        raise FileNotFoundError(f"Dataset not found at {root}")

    if args.measure == "gripper":
        run_gripper(args, root)
        return
    if args.calib_cm is None:
        ap.error("--calib-cm is required for --measure placement")
    name = args.repo_id.split("/")[-1]
    out_csv = Path(args.out) if args.out else Path(f"placement_errors_{name}.csv")
    scale_file = out_csv.with_suffix(".scale.json")

    scale = None  # cm per pixel
    if scale_file.exists():
        scale = json.loads(scale_file.read_text())["cm_per_px"]
        print(f"Loaded scale {scale:.4f} cm/px from {scale_file} (press c to redo)")

    episodes = load_episodes(root)
    print(f"{len(episodes)} episodes in {args.repo_id}")

    new_file = not out_csv.exists()
    csv_f = open(out_csv, "a", newline="")
    writer = csv.writer(csv_f)
    if new_file:
        writer.writerow(["episode", "trial", "error_cm", "cm_per_px",
                         "bowl_px", "cube_px", "frame_offset"])

    win = "placement error"
    cv2.namedWindow(win)
    clicks = []
    cv2.setMouseCallback(
        win, lambda ev, x, y, flags, p: clicks.append((x, y))
        if ev == cv2.EVENT_LBUTTONDOWN and len(clicks) < 2 else None)

    results = []
    calibrating = scale is None
    i = args.start_episode
    while i < len(episodes):
        ep_idx, video, from_ts, to_ts = episodes[i]
        offset = 0  # frames back from the episode's last frame
        clicks.clear()
        while True:
            # last frame sits one frame-length before to_timestamp
            ts = to_ts - (1 + offset) / 30
            frame = read_frame(video, max(ts, from_ts))
            if frame is None:
                print(f"ep {ep_idx}: cannot decode frame at {ts:.2f}s in {video}")
                break
            cv2.imshow(win, draw_overlay(frame, clicks, scale, calibrating, ep_idx, offset))
            key = cv2.waitKey(30) & 0xFF

            if calibrating and len(clicks) == 2:
                px = math.dist(clicks[0], clicks[1])
                scale = args.calib_cm / px
                scale_file.write_text(json.dumps({"cm_per_px": scale, "calib_cm": args.calib_cm}))
                print(f"Calibrated: {px:.1f} px = {args.calib_cm} cm -> {scale:.4f} cm/px")
                calibrating = False
                clicks.clear()

            if key == ord("q"):
                csv_f.close()
                print_summary(results, out_csv)
                return
            elif key == ord("r"):
                clicks.clear()
            elif key == ord("c"):
                calibrating = True
                clicks.clear()
            elif key == ord("a"):
                offset += FRAME_STEP
                clicks.clear()
            elif key == ord("d"):
                offset = max(0, offset - FRAME_STEP)
                clicks.clear()
            elif key == ord("n"):
                print(f"ep {ep_idx} (trial {ep_idx + 1}): skipped")
                break
            elif key == ord("s") and len(clicks) == 2 and not calibrating:
                err = math.dist(clicks[0], clicks[1]) * scale
                writer.writerow([ep_idx, ep_idx + 1, f"{err:.2f}", f"{scale:.5f}",
                                 f"{clicks[0]}", f"{clicks[1]}", offset])
                csv_f.flush()
                results.append((ep_idx + 1, err))
                print(f"ep {ep_idx} (trial {ep_idx + 1}): {err:.2f} cm")
                break
        i += 1

    csv_f.close()
    cv2.destroyAllWindows()
    print_summary(results, out_csv)


def print_summary(results, out_csv):
    print(f"\nSaved {len(results)} measurements to {out_csv}")
    if results:
        print("For scoresheet column F:")
        for trial, err in results:
            print(f"  trial {trial:2d}: {err:.1f}")


if __name__ == "__main__":
    main()
