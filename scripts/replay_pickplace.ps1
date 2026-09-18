<#
.SYNOPSIS
  Interactive random replay of pick-and-place episodes on the SO-101 follower.

.DESCRIPTION
  Prompts for how many random episodes and which sheet position to place the cube.
  Only episodes recorded at that position are sampled. After each reset (Right arrow),
  the episode is replayed and the arm returns to the session start pose.

.EXAMPLE
  .\scripts\replay_pickplace.ps1

.EXAMPLE
  .\scripts\replay_pickplace.ps1 -AllowFewer -Seed 42
#>
[CmdletBinding()]
param(
    [string]$RepoId      = "aakashv100/so101-pick-place-positions",
    [string]$Root        = "hf_data/so101-pick-place-positions",
    [string]$RobotPort   = "COM3",
    [string]$RobotId     = "my_so_arm",
    [string]$RobotCalib  = "./calibration/robots/so_follower",
    [int]$Seed           = -1,
    [switch]$AllowFewer,
    [switch]$NoPlaySounds
)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot

$Python = Join-Path $RepoRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $Python)) {
    throw "venv python not found at $Python."
}

$env:PYTHONUTF8 = "1"
$env:PYTHONIOENCODING = "utf-8"

$cmd = @(
    "scripts/replay_pickplace.py",
    "--repo-id=$RepoId",
    "--root=$Root",
    "--robot-port=$RobotPort",
    "--robot-id=$RobotId",
    "--robot-calib-dir=$RobotCalib"
)

if ($Seed -ge 0)   { $cmd += "--seed=$Seed" }
if ($AllowFewer)   { $cmd += "--allow-fewer" }
if ($NoPlaySounds) { $cmd += "--no-play-sounds" }

Write-Host "=== Pick-and-place random replay ===" -ForegroundColor Cyan
Write-Host "dataset : $RepoId"
Write-Host "root    : $Root"
Write-Host "robot   : $RobotPort"
Write-Host "You will enter: # episodes, then sheet position for the cube."
Write-Host "Keys: Right=ready after reset | Left=skip episode | Esc=stop | q=quit"
Write-Host ""

$ErrorActionPreference = "Continue"
$PSNativeCommandUseErrorActionPreference = $false

& $Python @cmd
exit $LASTEXITCODE
