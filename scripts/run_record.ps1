<#
.SYNOPSIS
  Record SO-101 teleop demos one episode at a time, with the cube placed anywhere.

.DESCRIPTION
  Wraps scripts/record_episodes.py (same style as record_pickplace.ps1, but no position ids).
  Per episode:
    1. Prompt: place the cube, put the leader at the start pose, press Enter (q = quit).
    2. The follower slides to the leader pose over 1 s (not recorded), so episodes don't start
       with a jump.
    3. Record with no time limit:
         Right arrow  - save, then prompt for the next episode
         Left arrow   - discard and redo the same episode
         Escape       - stop (the in-progress episode is discarded)
  No reset phase.

  Data kept consistent for ACT training and SmolVLA fine-tuning: the same layout as the project's
  other SO-101 datasets (6-joint state/action, gripper_cam + top_cam 640x480 MJPG, 30 fps, h264),
  and one task string for every episode. Appending with a different -Task is refused.

  The dataset is appended automatically if -Root already holds one.

.EXAMPLE
  # 50 episodes, checking arm + cameras first
  .\scripts\run_record.ps1 -NumEpisodes 50 -TeleopTest

.EXAMPLE
  # Keep going until you type q (or press Escape)
  .\scripts\run_record.ps1
#>
[CmdletBinding()]
param(
    [string]$RepoId       = "aakashv100/so101-pick-place-random",
    [string]$Root         = "hf_data/so101-pick-place-random",
    [string]$Task         = "Pick up the cube and place it in the box",
    [int]$NumEpisodes     = 0,                            # 0 = until q / Escape
    [string]$RobotPort    = "COM3",
    [string]$TeleopPort   = "COM4",
    [string]$RobotId      = "my_so_arm",
    [string]$TeleopId     = "my_so_arm",
    [string]$RobotCalib   = "./calibration/robots/so_follower",
    [string]$TeleopCalib  = "./calibration/teleoperators/so_leader",
    [int]$GripperCamIndex = 0,
    [int]$TopCamIndex     = 1,
    [int]$Fps             = 30,
    [switch]$Fresh,                                       # refuse to start if -Root already has a dataset
    [switch]$PushToHub,
    [switch]$DisplayData,                                 # also open Rerun
    [switch]$NoDisplayCameras,
    [switch]$TeleopTest                                   # free teleop first (not recorded), then confirm
)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot

$Python = Join-Path $RepoRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $Python)) {
    throw "venv python not found at $Python. Activate/create the project venv first."
}

# Put venv executables (e.g. rerun.exe, spawned by -DisplayData) on PATH; the venv isn't activated.
$env:PATH = "$(Join-Path $RepoRoot '.venv\Scripts');$env:PATH"

$env:PYTHONUTF8 = "1"
$env:PYTHONIOENCODING = "utf-8"

if ($RepoId -match "/eval_") {
    throw "'$RepoId' is an eval dataset name; use run_eval.ps1 for policy rollouts."
}

$cmd = @(
    "scripts/record_episodes.py",
    "--repo-id=$RepoId",
    "--root=$Root",
    "--task=$Task",
    "--num-episodes=$NumEpisodes",
    "--robot-port=$RobotPort",
    "--teleop-port=$TeleopPort",
    "--robot-id=$RobotId",
    "--teleop-id=$TeleopId",
    "--robot-calib-dir=$RobotCalib",
    "--teleop-calib-dir=$TeleopCalib",
    "--gripper-cam=$GripperCamIndex",
    "--top-cam=$TopCamIndex",
    "--fps=$Fps"
)

if ($Fresh)             { $cmd += "--fresh" }
if ($PushToHub)         { $cmd += "--push-to-hub" }
if ($DisplayData)       { $cmd += "--display-data" }
if ($NoDisplayCameras)  { $cmd += "--no-display-cameras" }
if ($TeleopTest)        { $cmd += "--teleop-test" }

$target = if ($NumEpisodes -gt 0) { "$NumEpisodes episodes" } else { "until q / Escape" }
Write-Host "=== Episode recording (cube anywhere) ===" -ForegroundColor Cyan
Write-Host "dataset : $RepoId"
Write-Host "root    : $Root  (auto-appends if it already exists)"
Write-Host "task    : $Task"
Write-Host "session : $target"
Write-Host "Keys: Enter=start episode | Right=save | Left=discard & redo | Esc=stop | q at prompt=quit"
Write-Host ""

$ErrorActionPreference = "Continue"
$PSNativeCommandUseErrorActionPreference = $false

& $Python @cmd
exit $LASTEXITCODE
