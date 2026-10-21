# SmolVLA Holdout: Model Reference, Inference, and Eval Scripts

Copied from the sibling project `C:\Users\OMNI-User\Desktop\so101-lerobot\`
(its own git repo, `github.com/ayushgawai/so101-lerobot`) on 2026-08-18, for
convenience — **not** vendored/tracked as part of this repo's own pipeline,
and **not runnable under this repo's Isaac Sim Python**
(`C:\Isaac-Sim\python.bat`). These scripts talk to the real SO-101 follower
directly over serial + USB cameras; run them from `so101-lerobot`'s own
Python environment (`uv`/`.venv`, `lerobot[smolvla]` installed), same as
the originals — see `so101-lerobot`'s own docs for that setup.

## What's here

- **`SmolVLA_holdout_training_report.md`** — full writeup of the holdout
  finetune (45/5 episode split), the one whose weights are actually
  retrievable right now: pushed to the Hub as
  [`aakashv100/smolvla_so101_pick_cube_holdout`](https://hf.co/aakashv100/smolvla_so101_pick_cube_holdout).
  See its §8 handover section for exact eval commands.
- **`SmolVLA_training_report.md`** — companion report for the full 50-episode
  run (no holdout). That run's weights are **not** retrievable (no local
  `outputs/` dir in this copy of `so101-lerobot`, no Hub push) — kept here
  for context/comparison only, e.g. its documented 2026-07-28 OOD stall
  (§ "sanity-run findings") which motivated pulling in
  `probe_start_hesitation.py` below.
- **`pretrained-model/smolvla_base/`** — the *base* pretrained checkpoint's
  config/metadata only (`config.json`, saved normalizer
  pre/postprocessor `.safetensors`, README, the original finetuning
  notebook). **No `model.safetensors`** — this is not a loadable model by
  itself, just the config the holdout finetune was derived from.
- **`test_inference_offline.py`** — loads a trained checkpoint and runs it
  against real recorded frames (no physical arm needed); reports per-joint
  MAE/accuracy and per-query latency. Works for any lerobot policy type,
  not SmolVLA-specific.
- **`scripts/probe_start_hesitation.py`** — directly diagnoses the
  flow-matching-sampler stall behind the OOD/hesitation issue: re-queries
  the policy `--num-samples` times per observation and reports how many
  sampled action chunks contain real motion. **This is the right tool to
  test the ArUco-tag OOD question against** (see
  `docs/object-pose-mirroring-plan.md`'s Phase 0 note in the main repo).
- **`scripts/goto_start_pose.py`** — drives the real follower to the
  demonstration start pose before an eval run (eval rollouts starting
  elsewhere are themselves out-of-distribution).
- **`scripts/run_eval.ps1`** — main real-robot eval wrapper around
  `lerobot-record`; `-Policy smolvla -Checkpoint <path-or-hub-id>` drives a
  scored rollout. Defaults to the (unavailable) full-run checkpoint path —
  override `-Checkpoint` to point at a locally-downloaded copy of the
  holdout Hub model to actually use it.
- **`scripts/run_smolvla_holdout_sanity.ps1`** (2026-08-18) — one-shot
  wrapper: downloads the holdout checkpoint's weights from the Hub if not
  already local, pauses for the hardware checklist, optionally drives the
  arm to the start pose, then runs `run_eval.ps1` for a short (default
  3-episode) sanity rollout with `-EpisodeTime -1` (no timeout, since this
  checkpoint has hesitated up to ~43s before moving). Pass a different
  `-RepoId` per run to compare cube-with-tag vs. cube-without-tag for the
  ArUco OOD question. **This is the only copy — it lives here, not in
  `so101-lerobot`.** Unlike everything else in this folder it's meant to
  actually be run from here: it takes a `-TargetRepo` parameter (default
  `C:\Users\OMNI-User\Desktop\so101-lerobot`) and `cd`s there itself before
  doing anything, so it still executes in that repo's venv/paths even
  though the script file sits in this repo.
- **`scripts/log_eval_to_wandb.py`**, **`log_offline_acc_to_wandb.py`**,
  **`make_eval_scoresheet.py`**, **`score_eval_frames.py`**,
  **`measure_placement_error.py`** — eval-scoresheet pipeline (logging,
  scoring, placement-error measurement) used to produce the numbers in the
  reports above.

## Environment fixes applied to `so101-lerobot` while getting this running (2026-08-18)

Getting a sanity eval to actually complete surfaced several environment
problems in `so101-lerobot` itself, fixed in place (not just worked around):

- `.venv` was Python 3.14.4 with nothing installed (not even `pip`) —
  `pyproject.toml`'s `requires-python = ">=3.12"` has no upper bound, so
  `uv` picked the newest available interpreter, which numpy 2.2.6 has no
  prebuilt Windows wheel for (forcing a from-source build that needs a C
  compiler nobody has installed). Fixed by recreating `.venv` pinned to the
  already-locally-available Python 3.12.13 (`uv venv --python 3.12`).
- The documented `uv sync --extra feetech --extra viz --extra dataset`
  leaves out `transformers` (SmolVLA's backbone, behind the `smolvla`
  extra) and `pynput` (keyboard control during recording, behind the
  `hardware` extra). Both now included:
  `uv sync --extra feetech --extra viz --extra dataset --extra smolvla --extra hardware`.
- `pyproject.toml` pinned `opencv-python-headless` (no GUI backend at all)
  as a base dependency, even though the eval scripts default to
  `--display_cameras=true`. `cv2.namedWindow` doesn't exist in headless
  builds, so any eval run crashed the instant it tried to open the preview
  window. **Swapped to `opencv-python` (GUI build)** in `pyproject.toml`
  itself and re-synced — verified working
  (`cv2.namedWindow(...)` succeeds, `cv2.getBuildInformation()` reports
  `Win32 UI: YES`).
- No Hugging Face auth configured on this machine at all (`lerobot_record`
  defaults `dataset.push_to_hub=True`, which 401s with no token). Added a
  `-NoPushToHub` switch to `scripts/run_eval.ps1` (small, additive, default
  behavior unchanged) so a throwaway sanity run doesn't need Hub auth —
  `run_smolvla_holdout_sanity.ps1` passes it by default, opt back in with
  `-PushToHub` once you've run `hf auth login`.

## Not copied (stayed in `so101-lerobot`)

- `src/lerobot/policies/smolvla` — library code; it lives in the root
  `lerobot` package of this repo (`../src/lerobot`), so duplicating it
  here would just create a second copy to keep in sync.
- `pretrained-model/so101-act-pick-place_ACT_migrated/` — a different
  policy (ACT, not SmolVLA), out of scope for this copy.
- `scripts/train_smolvla.ps1` — the training script that produced the
  holdout checkpoint. Not copied since this is scoped to model/inference/
  eval, not training, but it's in `so101-lerobot/scripts/` if needed later
  (documents the exact `-JobName smolvla_so101_pick_cube_holdout` flags).
- `scripts/swap_camera_keys.py`, `validate_dataset.py`,
  `probe_gemini_er.py`, `apply_calib.py`, `verify_calib.py`,
  `test_servos.py` — dataset-maintenance / calibration / unrelated-model
  utilities, not inference or eval.
- `hf_data/`, `.venv/`, `calibration/` — large or environment-specific;
  reference `so101-lerobot` directly for these rather than duplicating.
