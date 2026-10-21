<#
.SYNOPSIS
  One-shot setup + short sanity eval for the SmolVLA holdout checkpoint.

.DESCRIPTION
  Wraps the manual 4-step sequence for testing the holdout checkpoint end to
  end: download its weights from the Hub if not already present locally
  (only outputs/train/smolvla_so101_pick_cube_holdout ever had them, and that
  dir doesn't exist in every checkout), pause for the physical hardware
  checklist, optionally drive the arm to the demo start pose, then run a
  short rollout via run_eval.ps1.

  This copy lives in Sim-to-Real-SO-101-Workshop\smolvla-holdout-eval\scripts\
  for reference, not in so101-lerobot itself -- so unlike run_eval.ps1 it
  can't derive the repo root from its own script path. Instead it takes
  -TargetRepo (defaulting to the real so101-lerobot checkout) and cd's
  there before running anything, so .venv\, pretrained-model\, scripts\
  all resolve against that repo regardless of where this file sits.

.PARAMETER TargetRepo
  Path to the so101-lerobot checkout to run this against. Default
  C:\Users\OMNI-User\Desktop\so101-lerobot.

.PARAMETER NumEpisodes
  Number of episodes to record. Default 3 (sanity run).

.PARAMETER RepoId
  Eval dataset repo id (must start with '<user>/eval_'). Default
  aakashv100/eval_sanity-smolvla. Pass a different id to keep separate
  sanity runs side by side instead of overwriting -- e.g. when comparing
  cube-with-tag vs cube-without-tag, per
  docs/object-pose-mirroring-plan.md's SmolVLA OOD note in the sibling
  Sim-to-Real-SO-101-Workshop repo.

.PARAMETER Mode
  fixed | random. Default fixed (matches the demo start scene).

.PARAMETER EpisodeTime
  Seconds per episode, -1 = no cap. Default -1 -- this checkpoint has taken
  up to ~43s just to start moving (see SmolVLA_training_report.md's
  sanity-run findings), so the default 60s-cap behavior can cut off a real
  attempt that just hasn't started yet.

.PARAMETER SkipStartPose
  Skip driving the arm to the demo start pose before recording.

.PARAMETER ForceDownload
  Re-download the holdout checkpoint even if it already exists locally.

.PARAMETER PushToHub
  Push the recorded sanity dataset to the Hub (needs `hf auth login` or
  HF_TOKEN configured first). Default off: this wrapper is for throwaway
  sanity checks, and no HF auth has been set up on this machine as of
  2026-08-18, so pushing 401s by default -- pass this once you've logged in
  and actually want the sanity recording kept on the Hub.

.PARAMETER NoDisplay
  Skip the live camera-preview window during recording. Default off (i.e.
  display is ON by default) -- so101-lerobot's pyproject.toml was pinned to
  `opencv-python-headless` (no GUI backend, `cv2.namedWindow` crashed
  instantly) until 2026-08-18, when it was swapped for the GUI-enabled
  `opencv-python` build and verified working (`cv2.namedWindow` succeeds).
  Pass this switch if you want the old no-window behavior back, e.g. for a
  faster/lower-overhead run.

.EXAMPLE
  .\scripts\run_smolvla_holdout_sanity.ps1

.EXAMPLE
  # Baseline vs. ArUco-tag comparison: run twice with different repo ids
  .\scripts\run_smolvla_holdout_sanity.ps1 -RepoId aakashv100/eval_sanity-smolvla-no-tag
  .\scripts\run_smolvla_holdout_sanity.ps1 -RepoId aakashv100/eval_sanity-smolvla-with-tag
#>
[CmdletBinding()]
param(
    [string]$TargetRepo    = "C:\Users\OMNI-User\Desktop\so101-lerobot",
    [int]$NumEpisodes      = 3,
    [string]$RepoId        = "aakashv100/eval_sanity-smolvla",
    [ValidateSet("fixed", "random")]
    [string]$Mode          = "fixed",
    [int]$EpisodeTime      = -1,
    [switch]$SkipStartPose,
    [switch]$ForceDownload,
    [switch]$PushToHub,
    [switch]$NoDisplay
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path $TargetRepo)) {
    throw "Target repo not found: $TargetRepo. Pass -TargetRepo to point at your so101-lerobot checkout."
}
$RepoRoot = (Resolve-Path $TargetRepo).Path
Set-Location $RepoRoot

$Python = Join-Path $RepoRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $Python)) {
    throw "venv python not found at $Python. Activate/create the project venv first."
}

# --- Preflight: the venv exists but may be an empty/unsynced interpreter, or
# synced without every extra this actually needs (seen in practice: `uv sync
# --extra feetech --extra viz --extra dataset` per the setup guide leaves out
# `transformers` (smolvla policy) and `pynput` (keyboard control during
# recording) -- both gated behind separate extras). Catch that here with a
# clear message instead of failing deep inside the download/eval steps,
# since a failed native .exe call below does NOT stop this script on its own
# (see the $LASTEXITCODE checks) -- better to never reach there broken.
& $Python -c "import huggingface_hub, transformers, pynput" 2>$null
if ($LASTEXITCODE -ne 0) {
    throw ("$Python is missing a required package -- the venv looks unsynced. Run this in " +
        "$RepoRoot first, then re-run this script:`n`n    uv sync --extra feetech --extra viz --extra dataset --extra smolvla --extra hardware`n")
}

$CheckpointDir = Join-Path $RepoRoot "pretrained-model\smolvla_so101_pick_cube_holdout"

# --- Step 1: download the holdout checkpoint's weights if not already local ---
if ($ForceDownload -or -not (Test-Path (Join-Path $CheckpointDir "config.json"))) {
    Write-Host "=== Downloading aakashv100/smolvla_so101_pick_cube_holdout -> $CheckpointDir ===" -ForegroundColor Cyan
    & $Python -c "from huggingface_hub import snapshot_download; print(snapshot_download('aakashv100/smolvla_so101_pick_cube_holdout', local_dir=r'$CheckpointDir'))"
    if ($LASTEXITCODE -ne 0) { throw "Checkpoint download failed (exit $LASTEXITCODE) -- see the traceback above." }
} else {
    Write-Host "=== Checkpoint already local: $CheckpointDir (use -ForceDownload to re-pull) ===" -ForegroundColor DarkGray
}

# --- Step 2: hardware checklist (manual, real robot -- this pauses for you) ---
Write-Host ""
Write-Host "=== Before continuing, confirm ===" -ForegroundColor Yellow
Write-Host "  - SO-101 follower powered on and connected on COM3"
Write-Host "  - gripper_cam at index 0, top_cam at index 1"
Write-Host "  - cube/bowl set up in the '$Mode' start scene"
Read-Host "Press Enter once ready (Ctrl+C to abort)"

# --- Step 3: optional start-pose move ---
if (-not $SkipStartPose) {
    Write-Host ""
    Write-Host "=== Driving arm to demo start pose ===" -ForegroundColor Cyan
    & $Python "scripts\goto_start_pose.py"
    if ($LASTEXITCODE -ne 0) { throw "goto_start_pose.py failed (exit $LASTEXITCODE) -- see the traceback above." }
} else {
    Write-Host "=== Skipping start-pose move (-SkipStartPose) ===" -ForegroundColor DarkGray
}

# --- Step 4: sanity eval ---
Write-Host ""
Write-Host "=== Running $NumEpisodes-episode SmolVLA holdout sanity eval ($Mode) ===" -ForegroundColor Cyan
# Hashtable splatting, not array splatting: an array's elements bind
# POSITIONALLY (verified empirically -- @("-Policy", "smolvla") ends up
# binding the literal string "-Policy" to the first declared parameter,
# which is $Policy itself, tripping its ValidateSet). A hashtable's keys
# bind by name regardless of declaration order, which is what we want here.
$evalArgs = @{
    Policy       = "smolvla"
    Mode         = $Mode
    NumEpisodes  = $NumEpisodes
    Checkpoint   = $CheckpointDir
    RepoId       = $RepoId
    EpisodeTime  = $EpisodeTime
    ClearCache   = $true
}
if (-not $PushToHub) { $evalArgs["NoPushToHub"] = $true }
if ($NoDisplay) { $evalArgs["NoDisplay"] = $true }
& (Join-Path $RepoRoot "scripts\run_eval.ps1") @evalArgs

exit $LASTEXITCODE
