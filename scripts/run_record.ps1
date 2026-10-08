<#
.SYNOPSIS
  Record SO-101 teleoperation demonstrations (leader COM4 -> follower COM3) with
  the same camera setup the eval wrapper (run_eval.ps1) uses.

.DESCRIPTION
  Wraps `lerobot-record` (run via the venv python). Data-collection rules baked in:
  - One rate everywhere: control loop, dataset and both cameras run at -Fps (30),
    the cameras' native rate, so every dataset row has a fresh frame.
  - Cameras identical to eval: same names, indices, 640x480, MJPG. The policy keys on
    observation.images.<name>, so changing any of these between train and eval breaks it.
  - h264 encoding (the default libsvtav1 crashes on this Windows build).
  - Rerun gets JPEG-compressed frames so the viewer doesn't hit its memory limit.

  Keyboard: right arrow ends the episode (or the reset) early, left arrow re-records
  it, Escape stops and saves.

.EXAMPLE
  # 2-episode sanity run, local only
  .\scripts\run_record.ps1 -RepoId aakashv100/so101-pick-cube-test -NumEpisodes 2 -NoPush -ClearCache

.EXAMPLE
  # Full 30-episode session
  .\scripts\run_record.ps1 -RepoId aakashv100/so101-pick-cube-v3

.EXAMPLE
  # Continue an interrupted session with 10 more episodes
  .\scripts\run_record.ps1 -RepoId aakashv100/so101-pick-cube-v3 -NumEpisodes 10 -Resume
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$RepoId,
    [int]$NumEpisodes    = 30,
    [string]$Task        = "Pick up the cube and place it in the bowl",
    [int]$EpisodeTime    = 45,
    [int]$ResetTime      = 15,
    [switch]$ClearCache,                                  # delete the dataset's local HF cache before running
    [switch]$Resume,                                      # continue an interrupted run; -NumEpisodes = episodes to add
    [switch]$NoPush,                                      # keep the dataset local (no Hub upload)

    # --- robot / leader / cameras ---
    [string]$Port         = "COM3",
    [string]$LeaderPort   = "COM4",
    [string]$RobotId      = "my_so_arm",
    [string]$CalibDir     = "./calibration/robots/so_follower",
    [string]$LeaderCalibDir = "./calibration/teleoperators/so_leader",
    [int]$GripperCamIndex = 0,
    [int]$TopCamIndex     = 1,
    [int]$Fps             = 30,
    [switch]$NoDisplay                                    # no Rerun viewer
)

$ErrorActionPreference = "Stop"

# Run from the repo root (parent of this script's folder) so relative paths resolve.
$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot

$Python = Join-Path $RepoRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $Python)) {
    throw "venv python not found at $Python. Activate/create the project venv first."
}

# Put venv executables (e.g. rerun.exe, spawned by --display_data) on PATH; the venv isn't activated.
$env:PATH = "$(Join-Path $RepoRoot '.venv\Scripts');$env:PATH"

# Force UTF-8 so console logging doesn't crash on cp1252 Windows shells.
$env:PYTHONUTF8 = "1"
$env:PYTHONIOENCODING = "utf-8"

if ($RepoId -match "/eval_") {
    throw "'$RepoId' is an eval dataset name; use run_eval.ps1 for policy rollouts."
}

# --- Local HF cache handling (same rules as run_eval.ps1) ---
$CacheDir = Join-Path $env:USERPROFILE ".cache\huggingface\lerobot\$($RepoId -replace '/', '\')"
if ($Resume) {
    if ($ClearCache) { throw "-Resume and -ClearCache are mutually exclusive." }
    if (-not (Test-Path $CacheDir)) { throw "Nothing to resume: no cache at $CacheDir" }
    Write-Host "Resuming into existing dataset ($CacheDir); recording $NumEpisodes MORE episodes." -ForegroundColor Yellow
} elseif ($ClearCache) {
    if (Test-Path $CacheDir) {
        Write-Host "Clearing cache: $CacheDir" -ForegroundColor Yellow
        Remove-Item -Recurse -Force -Confirm:$false $CacheDir
    }
} elseif (Test-Path $CacheDir) {
    throw "Cache dir exists from a previous run: $CacheDir`nRe-run with -ClearCache to delete it, -Resume to continue it, or pass a different -RepoId."
}

$display = if ($NoDisplay) { "false" } else { "true" }
$push    = if ($NoPush)    { "false" } else { "true" }

# Must stay identical to the camera string in run_eval.ps1.
$cameras = "{gripper_cam: {type: opencv, index_or_path: $GripperCamIndex, width: 640, height: 480, fps: $Fps, fourcc: MJPG}, " +
           "top_cam: {type: opencv, index_or_path: $TopCamIndex, width: 640, height: 480, fps: $Fps, fourcc: MJPG}}"

$cmd = @(
    "-m", "lerobot.scripts.lerobot_record",
    "--robot.type=so101_follower",
    "--robot.port=$Port",
    "--robot.id=$RobotId",
    "--robot.calibration_dir=$CalibDir",
    "--robot.cameras=$cameras",
    "--teleop.type=so101_leader",
    "--teleop.port=$LeaderPort",
    "--teleop.id=$RobotId",
    "--teleop.calibration_dir=$LeaderCalibDir",
    "--dataset.repo_id=$RepoId",
    "--dataset.no_stamp=true",                            # keep -RepoId as given (0.6.1 would timestamp it)
    "--dataset.fps=$Fps",
    "--dataset.num_episodes=$NumEpisodes",
    "--dataset.single_task=$Task",
    "--dataset.episode_time_s=$EpisodeTime",
    "--dataset.reset_time_s=$ResetTime",
    "--dataset.rgb_encoder.vcodec=h264",
    "--dataset.push_to_hub=$push",
    "--display_data=$display",
    "--display_compressed_images=true"
)
if ($Resume) {
    $cmd += "--resume=true"
    $cmd += "--dataset.root=$CacheDir"
}

Write-Host "=== Teleop data collection ===" -ForegroundColor Cyan
Write-Host "dataset : $RepoId   episodes: $NumEpisodes   push: $push"
Write-Host "rate    : loop/dataset/cameras @ ${Fps} Hz   episode ${EpisodeTime}s, reset ${ResetTime}s"
Write-Host "cameras : gripper=$GripperCamIndex  top=$TopCamIndex  (check they are not swapped before episode 0)"
Write-Host ""
Write-Host "$Python $($cmd -join ' ')" -ForegroundColor DarkGray
Write-Host ""

# Python logging writes warnings to stderr; don't let that abort the run.
$ErrorActionPreference = "Continue"
$PSNativeCommandUseErrorActionPreference = $false

& $Python @cmd
exit $LASTEXITCODE
