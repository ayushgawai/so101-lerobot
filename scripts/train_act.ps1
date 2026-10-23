<#
.SYNOPSIS
  Train an ACT policy on the SO-101 pick-cube demonstrations, with flexible
  checkpointing, resume, and Weights & Biases logging.

.DESCRIPTION
  Wraps `lerobot-train` (run via the venv python) with sensible defaults for the
  locally recorded `aakashv100/so101-pick-cube` dataset (50 episodes, 2 cameras,
  6-DoF state/action, 30 fps).

  Trains ACT from random init (no --policy.path => fresh weights). Checkpoints
  land in outputs/train/<JobName>/checkpoints, saved every -SaveFreq steps plus a
  rolling `last` symlink. Resume with -Resume (optionally -ResumeFrom <dir>).

  On resume, the checkpoint's config is loaded and any flags you pass here are
  applied as overrides (so you can change -Steps, -SaveFreq, or W&B settings).

  NOTE on Windows: keep -NumWorkers 0. Multiprocessing dataloaders (num_workers>0
  with persistent workers) stall on this platform during video decode.

.EXAMPLE
  # Fresh run with W&B logging
  .\scripts\train_act.ps1 -Wandb -WandbProject so101-act

.EXAMPLE
  # Resume the last checkpoint and keep logging to the same W&B run
  .\scripts\train_act.ps1 -Resume -Wandb -WandbProject so101-act -WandbRunId abc123

.EXAMPLE
  # Resume from a specific checkpoint, extend training, save more often
  .\scripts\train_act.ps1 -Resume -ResumeFrom outputs/train/act_so101_pick_cube/checkpoints/010000 -Steps 40000 -SaveFreq 2000
#>
[CmdletBinding()]
param(
    # --- data / model ---
    [string]$RepoId      = "aakashv100/so101-pick-cube",
    [string]$DatasetRoot = "hf_data/so101-pick-cube",   # set "" to download from the Hub
    [string]$JobName     = "act_so101_pick_cube",
    [string]$OutputDir   = "",                            # default: outputs/train/<JobName>
    [string]$Device      = "cuda",                        # cuda | cpu
    [int]$BatchSize      = 8,
    [int]$Steps          = 100000,
    [int]$ChunkSize      = 100,
    [int]$NumWorkers     = 0,                             # keep 0 on Windows
    [int]$Seed           = 1000,                          # lerobot's default; vary to measure run-to-run spread
    [switch]$ReturnUint8,                                 # faster dataloading (uint8 frames over IPC)
    [switch]$Augment,                                     # enable image augmentation (color/affine jitter) for closed-loop robustness

    # --- validation ---
    # Whole episodes held out of training and scored as `val/loss`, e.g. -ValEpisodes (47,45,37,42,39).
    # Empty = no validation. See train_smolvla.ps1 for the cost of a validation pass.
    [int[]]$ValEpisodes  = @(),
    [int]$ValFreq        = 5000,                          # validate every N steps (and at the last step)

    # --- checkpointing / logging cadence ---
    [int]$SaveFreq       = 5000,
    [int]$LogFreq        = 200,
    [switch]$NoSaveCheckpoint,                            # disable checkpointing entirely

    # --- resume ---
    [switch]$Resume,
    [string]$ResumeFrom  = "",                            # checkpoint dir (default: <OutputDir>/checkpoints/last)

    # --- weights & biases ---
    [switch]$Wandb,
    [string]$WandbProject = "so101-act",
    [string]$WandbEntity  = "",
    [string]$WandbRunId   = "",                           # set to continue an existing run on resume
    [string]$WandbNotes   = "",
    [string]$WandbMode    = "online",                     # online | offline | disabled

    # --- hub ---
    [switch]$PushToHub
)

$ErrorActionPreference = "Stop"

# Run from the repo root (parent of this script's folder) so relative paths resolve.
$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot

$Python = Join-Path $RepoRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $Python)) {
    throw "venv python not found at $Python. Activate/create the project venv first."
}

# Force UTF-8 so console logging (e.g. the 'pi' symbol) doesn't crash on cp1252 Windows shells.
$env:PYTHONUTF8 = "1"
$env:PYTHONIOENCODING = "utf-8"

if ([string]::IsNullOrEmpty($OutputDir)) {
    $OutputDir = "outputs/train/$JobName"
}

$saveCkpt = if ($NoSaveCheckpoint) { "false" } else { "true" }
$pushHub  = if ($PushToHub)        { "true" }  else { "false" }

$cmd = @(
    "-m", "lerobot.scripts.lerobot_train",
    "--policy.type=act",
    "--policy.device=$Device",
    "--policy.push_to_hub=$pushHub",
    "--policy.chunk_size=$ChunkSize",
    "--policy.n_action_steps=$ChunkSize",
    "--dataset.repo_id=$RepoId",
    "--batch_size=$BatchSize",
    "--steps=$Steps",
    "--save_checkpoint=$saveCkpt",
    "--save_freq=$SaveFreq",
    "--log_freq=$LogFreq",
    "--num_workers=$NumWorkers",
    "--seed=$Seed",
    "--job_name=$JobName",
    "--output_dir=$OutputDir"
)

# Use the local dataset tree when provided (avoids re-downloading from the Hub).
if (-not [string]::IsNullOrEmpty($DatasetRoot)) {
    $cmd += "--dataset.root=$DatasetRoot"
}

if ($ReturnUint8) {
    $cmd += "--dataset.return_uint8=true"
}

# Held-out validation: the policy never trains on these episodes.
if ($ValEpisodes.Count -gt 0) {
    $cmd += "--dataset.val_episodes=[$($ValEpisodes -join ',')]"
    $cmd += "--val_freq=$ValFreq"
}

# Enable the image_transforms already defined in the dataset config (brightness/contrast/
# saturation/hue/sharpness/affine). Off by default; turn on for closed-loop robustness.
if ($Augment) {
    $cmd += "--dataset.image_transforms.enable=true"
}

# --- Weights & Biases ---
if ($Wandb) {
    $cmd += "--wandb.enable=true"
    $cmd += "--wandb.project=$WandbProject"
    $cmd += "--wandb.mode=$WandbMode"
    if (-not [string]::IsNullOrEmpty($WandbEntity)) { $cmd += "--wandb.entity=$WandbEntity" }
    if (-not [string]::IsNullOrEmpty($WandbRunId))  { $cmd += "--wandb.run_id=$WandbRunId" }
    if (-not [string]::IsNullOrEmpty($WandbNotes))  { $cmd += "--wandb.notes=$WandbNotes" }
} else {
    $cmd += "--wandb.enable=false"
}

# --- Resume ---
# On resume, lerobot loads the checkpoint's train_config.json and applies the
# above flags as overrides, so changing -Steps/-SaveFreq/-Wandb* here works.
if ($Resume) {
    $ckptDir = if (-not [string]::IsNullOrEmpty($ResumeFrom)) { $ResumeFrom } else { "$OutputDir/checkpoints/last" }
    $configPath = "$ckptDir/pretrained_model/train_config.json"
    if (-not (Test-Path $configPath)) {
        throw "Cannot resume: $configPath not found. Check -OutputDir/-ResumeFrom (need a saved checkpoint)."
    }
    $cmd += "--resume=true"
    $cmd += "--config_path=$configPath"
}

Write-Host "=== Training ACT ($(if ($Resume) { 'resume' } else { 'from scratch' })) ===" -ForegroundColor Cyan
Write-Host "dataset : $RepoId  (root: $(if ($DatasetRoot) { $DatasetRoot } else { 'Hub' }))"
Write-Host "output  : $OutputDir"
Write-Host "device  : $Device   steps: $Steps   batch: $BatchSize   save_freq: $SaveFreq   seed: $Seed"
Write-Host "val     : $(if ($ValEpisodes.Count -gt 0) { "episodes $($ValEpisodes -join ',') every $ValFreq steps" } else { 'disabled' })"
Write-Host "wandb   : $(if ($Wandb) { "$WandbProject (mode=$WandbMode)" } else { 'disabled' })"
Write-Host ""
Write-Host "$Python $($cmd -join ' ')" -ForegroundColor DarkGray
Write-Host ""

# Python logging writes warnings to stderr; don't let that abort the run.
$ErrorActionPreference = "Continue"
$PSNativeCommandUseErrorActionPreference = $false

& $Python @cmd
exit $LASTEXITCODE
