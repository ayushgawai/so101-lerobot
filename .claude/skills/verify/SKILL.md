---
name: verify
description: How to verify changes in this SO-101 LeRobot fork (CI on GitHub Actions, hardware-free CLI checks).
---

# Verifying changes here

## CI (GitHub Actions, `.github/workflows/ci.yml`)
- Triggers on push to `main-windows` / `upgrade-*` and PRs. Windows runner, `uv sync --locked --extra so101`.
- No `gh` on the lab PC. Read runs via the public API (repo is public, 60 req/h anonymous):
  `curl -s https://api.github.com/repos/ayushgawai/so101-lerobot/actions/runs?branch=<b>&per_page=1`
  then `.../actions/runs/<id>/jobs` for step results. Job logs need auth (403);
  failed tests are emitted as error annotations: `.../check-runs/<job_id>/annotations`.
- Push upgrade branches **squashed** (one commit on top of main-windows): the local merge branch
  carries upstream LeRobot history, whose 899 LFS objects would be uploaded into this repo's quota.

## Reproducing CI locally
- Clone to a SHORT path (e.g. `..\ci-clone`); deep paths hit Windows MAX_PATH inside the venv.
  `GIT_LFS_SKIP_SMUDGE=1 git -c core.longpaths=true clone --depth 1 --branch <b> <url> ..\ci-clone`
- Windows checkouts turn git symlinks into plain files, so symlink bugs only show on CI —
  `tests/test_upgrade_invariants.py::test_no_dangling_symlinks` reads them from git instead.

## Hardware-free CLI checks
- Drive real entry points with a bad port (`--robot.port=COM99`, `--teleop.port=COM98`) to reach
  config parsing / connect error paths without moving the arm.
- NEVER run `goto_start_pose.py`, `verify_calib.py`, `test_servos.py`, `apply_calib.py` as a
  "--help" smoke test: they ignore --help and connect to COM3 immediately.
- Set `PYTHONUTF8=1` (cp1252 console can't print some help texts).
