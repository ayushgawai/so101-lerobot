<#
.SYNOPSIS
  Record pick-and-place demos. Prompts for position + iteration count.

.DESCRIPTION
  Same dataset is appended automatically if it already exists.
  You enter pickup position and how many iterations, record that block,
  then enter the next position (or q to quit).

  Keyboard while recording / resetting:
    Right arrow  — end episode (save) / end reset
    Left arrow   — discard and re-record
    Escape       — stop session

.EXAMPLE
  .\scripts\record_pickplace.ps1
#>
[CmdletBinding()]
param(
    [string]$RepoId       = "aakashv100/so101-pick-place-positions",
    [string]$Root         = "hf_data/so101-pick-place-positions",
    [string]$RobotPort    = "COM3",
    [string]$TeleopPort   = "COM4",
    [string]$RobotId      = "my_so_arm",
    [string]$TeleopId     = "my_so_arm",
    [string]$RobotCalib   = "./calibration/robots/so_follower",
    [string]$TeleopCalib  = "./calibration/teleoperators/so_leader",
    [int]$GripperCamIndex = 0,
    [int]$TopCamIndex     = 1,
    [int]$Fps             = 30,
    [switch]$Fresh,
    [switch]$PushToHub,
    [switch]$DisplayData,
    [switch]$NoDisplayCameras
)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot

$Python = Join-Path $RepoRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $Python)) {
    throw "venv python not found at $Python. Activate/create the project venv first."
}

$env:PYTHONUTF8 = "1"
$env:PYTHONIOENCODING = "utf-8"

$cmd = @(
    "scripts/record_pickplace.py",
    "--repo-id=$RepoId",
    "--root=$Root",
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

Write-Host "=== Pick-and-place recording ===" -ForegroundColor Cyan
Write-Host "dataset : $RepoId"
Write-Host "root    : $Root  (auto-appends if it already exists)"
Write-Host "cameras : side-by-side popup (gripper | top) enabled"
Write-Host "You will be asked: position, then iterations."
Write-Host "Keys: Right=save/end reset | Left=rerecord | Esc=stop | q at prompt=quit"
Write-Host ""

$ErrorActionPreference = "Continue"
$PSNativeCommandUseErrorActionPreference = $false

& $Python @cmd
exit $LASTEXITCODE
