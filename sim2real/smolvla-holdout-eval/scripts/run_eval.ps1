<#
.SYNOPSIS
  Run policy inference on the SO-101 for the eval scoresheet, with optional
  cache clearing and easily overridable parameters.

.DESCRIPTION
  Wraps `lerobot-record` (run via the venv python) with the eval defaults for
  the eval-scoresheet protocol: 50 episodes, both USB cameras, AMP on, camera
  display on. `lerobot-record` reads the policy type from the checkpoint, so the
  same wrapper drives ACT and SmolVLA.

  -Policy picks the checkpoint and eval dataset naming preset (act | smolvla);
  both policies are scored against the same protocol and the same 50 random cube
  positions. -Mode picks the scoresheet tab (fixed | random) and sets the eval
  dataset repo id accordingly. -ClearCache deletes that dataset's local HF cache
  dir first, so a crashed run can be restarted cleanly.

  Keyboard during recording: right-arrow ends the episode and saves,
  Escape stops without saving.

  -EpisodeTime -1 drops the per-episode time cap, so an episode ends when you decide the trial is
  over (right arrow) rather than after 60 s. The run stays bounded by -NumEpisodes.

.EXAMPLE
  # ACT, fixed-position tab, fresh start
  .\scripts\run_eval.ps1 -Mode fixed -ClearCache

.EXAMPLE
  # SmolVLA, random-position tab
  .\scripts\run_eval.ps1 -Policy smolvla -Mode random -ClearCache

.EXAMPLE
  # No per-episode clock: each trial runs until you end it with the right arrow
  .\scripts\run_eval.ps1 -Policy smolvla -Mode fixed -EpisodeTime -1 -ClearCache

.EXAMPLE
  # 3-episode sanity run before committing to the full 50
  .\scripts\run_eval.ps1 -Policy smolvla -Mode fixed -NumEpisodes 3 -RepoId aakashv100/eval_sanity-smolvla -ClearCache

.EXAMPLE
  # Try a different checkpoint for a quick 5-episode sanity run
  .\scripts\run_eval.ps1 -Mode fixed -Checkpoint outputs/train/act_so101_pick_cube_v2/checkpoints/030000/pretrained_model -NumEpisodes 5 -RepoId aakashv100/eval_sanity -ClearCache
#>
[CmdletBinding()]
param(
    # --- eval protocol ---
    [ValidateSet("act", "smolvla")]
    [string]$Policy      = "act",
    [ValidateSet("fixed", "random")]
    [string]$Mode        = "fixed",
    [string]$RepoId      = "",                            # default: see $Presets below
    [string]$RenameMap   = "",                            # default: see $Presets below
    [int]$NumEpisodes    = 50,
    [string]$Task        = "Pick up the cube and place it in the bowl",
    [switch]$ClearCache,                                  # delete the eval dataset's local HF cache before running
    [switch]$Resume,                                      # continue an interrupted run; -NumEpisodes = episodes to add THIS session

    # --- model ---
    [string]$Checkpoint  = "",                            # default: see $Presets below
    [string]$Device      = "cuda",                        # cuda | cpu
    [switch]$NoAmp,                                       # disable mixed-precision inference
    [switch]$CompilePolicy,                               # torch.compile the policy (slow first step, faster inference)
    [int]$InterpolationMultiplier = 1,                    # sub-steps between policy actions; smooths slow policies
    # SmolVLA only: flow-matching denoise steps per query, the main inference-latency
    # knob. Measured on this machine: 10 -> 252 ms, 5 -> 143 ms, 2 -> 99 ms per query.
    # 0 keeps the checkpoint's value (10).
    [int]$DenoiseSteps   = 0,

    # --- robot / cameras ---
    [string]$Port        = "COM3",
    [string]$RobotId     = "my_so_arm",
    [string]$CalibDir    = "./calibration/robots/so_follower",
    [int]$GripperCamIndex = 0,
    [int]$TopCamIndex     = 1,
    [int]$Fps             = 30,
    [switch]$NoDisplay,                                   # hide the side-by-side camera window
    [switch]$NoReturnToStart,                             # don't drive the arm back to its start pose after each episode

    # --- timing (seconds; 0 = keep lerobot defaults, -1 = no limit) ---
    # -EpisodeTime -1 removes the 60 s cap: the rollout then ends on its terminal condition
    # (right arrow when the trial is decided) instead of on the clock, which matters for SmolVLA
    # because it can hesitate for tens of seconds before it starts moving.
    [int]$EpisodeTime    = 0,
    [int]$ResetTime      = 0
)

$ErrorActionPreference = "Stop"

# Run from the repo root (parent of this script's folder) so relative paths resolve.
$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot

$Python = Join-Path $RepoRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $Python)) {
    throw "venv python not found at $Python. Activate/create the project venv first."
}

# Force UTF-8 so console logging doesn't crash on cp1252 Windows shells.
$env:PYTHONUTF8 = "1"
$env:PYTHONIOENCODING = "utf-8"

# Per-policy defaults. Both are evaluated on the same protocol; only the
# checkpoint and the eval dataset name differ.
# RenameMap: smolvla_base declares its cameras as observation.images.camera{1,2},
# so the robot's keys have to be mapped onto them exactly as during training.
# lerobot_record overrides the checkpoint's own rename step with this map, so
# leaving it empty for SmolVLA would feed the policy camera keys it never saw.
$Presets = @{
    act = @{
        Checkpoint = "outputs/train/act_so101_pick_cube_v2/checkpoints/060000/pretrained_model"
        RepoId     = "aakashv100/eval_so101-pick-cube-v2-$Mode"
        RenameMap  = ""
    }
    smolvla = @{
        Checkpoint = "outputs/train/smolvla_so101_pick_cube/checkpoints/020000/pretrained_model"
        RepoId     = "aakashv100/eval_so101-pick-cube-v2-smolvla-$Mode"
        RenameMap  = "{observation.images.top_cam: observation.images.camera1, observation.images.gripper_cam: observation.images.camera2}"
    }
}
$Preset = $Presets[$Policy]

if ([string]::IsNullOrEmpty($RepoId)) {
    $RepoId = $Preset.RepoId
}
if ([string]::IsNullOrEmpty($Checkpoint)) {
    $Checkpoint = $Preset.Checkpoint
}
if (-not $PSBoundParameters.ContainsKey("RenameMap")) {
    $RenameMap = $Preset.RenameMap
}
if ($RepoId -notmatch "/eval_") {
    throw "Eval dataset repo_id must start with 'eval_' after the user name (got '$RepoId')."
}
if (-not (Test-Path "$Checkpoint/config.json")) {
    throw "Checkpoint not found: $Checkpoint (need the pretrained_model dir)."
}

# --- Clear the local HF cache for this eval dataset ---
$CacheDir = Join-Path $env:USERPROFILE ".cache\huggingface\lerobot\$($RepoId -replace '/', '\')"
if ($Resume) {
    if ($ClearCache) { throw "-Resume and -ClearCache are mutually exclusive." }
    if (-not (Test-Path $CacheDir)) {
        throw "Nothing to resume: no cache at $CacheDir"
    }
    Write-Host "Resuming into existing dataset ($CacheDir); recording $NumEpisodes MORE episodes." -ForegroundColor Yellow
} elseif ($ClearCache) {
    if (Test-Path $CacheDir) {
        Write-Host "Clearing cache: $CacheDir" -ForegroundColor Yellow
        Remove-Item -Recurse -Force -Confirm:$false $CacheDir
    } else {
        Write-Host "Cache already clean: $CacheDir" -ForegroundColor DarkGray
    }
} elseif (Test-Path $CacheDir) {
    throw "Cache dir exists from a previous run: $CacheDir`nRe-run with -ClearCache to delete it, -Resume to continue it, or pass a different -RepoId."
}

$useAmp  = if ($NoAmp)           { "false" } else { "true" }
$display = if ($NoDisplay)       { "false" } else { "true" }
$goHome  = if ($NoReturnToStart) { "false" } else { "true" }

$cameras = "{gripper_cam: {type: opencv, index_or_path: $GripperCamIndex, width: 640, height: 480, fps: $Fps, fourcc: MJPG}, " +
           "top_cam: {type: opencv, index_or_path: $TopCamIndex, width: 640, height: 480, fps: $Fps, fourcc: MJPG}}"

$cmd = @(
    "-m", "lerobot.scripts.lerobot_record",
    "--robot.type=so101_follower",
    "--robot.port=$Port",
    "--robot.id=$RobotId",
    "--robot.calibration_dir=$CalibDir",
    "--robot.cameras=$cameras",
    "--policy.path=$Checkpoint",
    "--policy.device=$Device",
    "--policy.use_amp=$useAmp",
    "--dataset.repo_id=$RepoId",
    "--dataset.num_episodes=$NumEpisodes",
    "--dataset.single_task=$Task",
    "--display_cameras=$display",
    "--return_to_start_pose=$goHome"
)

if (-not [string]::IsNullOrEmpty($RenameMap)) { $cmd += "--dataset.rename_map=$RenameMap" }
if ($EpisodeTime -ne 0) { $cmd += "--dataset.episode_time_s=$EpisodeTime" }
if ($ResetTime -ne 0)   { $cmd += "--dataset.reset_time_s=$ResetTime" }
if ($CompilePolicy)     { $cmd += "--compile_policy=true" }
if ($DenoiseSteps -gt 0) { $cmd += "--policy.num_steps=$DenoiseSteps" }
if ($InterpolationMultiplier -gt 1) { $cmd += "--interpolation_multiplier=$InterpolationMultiplier" }
if ($Resume) {
    $cmd += "--resume=true"
    # resume() refuses to run without an explicit root (it must not write into
    # the Hub snapshot cache); point it at the dataset's recording cache dir.
    $cmd += "--dataset.root=$CacheDir"
}

Write-Host "=== $($Policy.ToUpper()) eval inference ($Mode tab) ===" -ForegroundColor Cyan
Write-Host "checkpoint : $Checkpoint"
Write-Host "dataset    : $RepoId   episodes: $NumEpisodes"
Write-Host "cameras    : gripper=$GripperCamIndex  top=$TopCamIndex  @ ${Fps}fps"
if ($RenameMap) { Write-Host "rename     : $RenameMap" }
Write-Host "amp        : $useAmp   display: $display   compile: $($CompilePolicy.IsPresent)"
Write-Host ""
Write-Host "$Python $($cmd -join ' ')" -ForegroundColor DarkGray
Write-Host ""

# Python logging writes warnings to stderr; don't let that abort the run.
$ErrorActionPreference = "Continue"
$PSNativeCommandUseErrorActionPreference = $false

& $Python @cmd
exit $LASTEXITCODE
