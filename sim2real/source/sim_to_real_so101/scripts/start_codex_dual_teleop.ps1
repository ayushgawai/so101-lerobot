param(
    [ValidateSet('Preflight', 'PhysicsCheck', 'SimOnly', 'Dual')]
    [string]$Mode = 'Dual',
    [string]$IsaacPython = 'C:\Isaac-Sim\python.bat',
    [string]$Layout = '',
    [string]$LeaderPort = 'COM4',
    [string]$FollowerPort = 'COM3',
    [string]$CalibrationRoot = '',
    [string]$LeaderId = 'my_so_arm',
    [string]$FollowerId = 'my_so_arm',
    [double]$ControlHz = 30,
    [double]$ControlTimeoutSec = 2.0,
    [ValidateSet('Match', 'Ramp')]
    [string]$StartupMode = 'Match',
    [switch]$ShowCameras,
    [switch]$RequireCameraAlignment,
    [int]$FixedCamera = 1,
    [int]$WristCamera = 0,
    [double]$WristRollOffsetDeg = -90
)
$ErrorActionPreference = 'Stop'
$simRoot = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$repoRoot = Split-Path -Parent $simRoot
if (-not $CalibrationRoot) { $CalibrationRoot = Join-Path $repoRoot 'calibration' }
if (-not (Test-Path -LiteralPath $IsaacPython -PathType Leaf)) {
    throw "Python launcher not found: $IsaacPython. Set -IsaacPython to your Isaac Sim python.bat."
}
$controller = Join-Path $PSScriptRoot 'leader_arm_teleop_raw_isaacsim.py'
$controllerArgs = @($controller, '--scene_profile', 'codex',
    '--calibration_dir', $CalibrationRoot, '--robot_id', $LeaderId,
    '--follower_robot_id', $FollowerId, '--control_hz', $ControlHz,
    '--control_timeout_s', $ControlTimeoutSec,
    '--startup_mode', $StartupMode.ToLowerInvariant(),
    '--wrist_roll_offset_deg', $WristRollOffsetDeg)
if ($Mode -eq 'PhysicsCheck') {
    $controllerArgs += @('--headless', '--sim_test_seconds', '4')
} else {
    $controllerArgs += @('--start_paused', '--port', $LeaderPort)
    if ($Mode -in @('Dual', 'Preflight')) {
        $controllerArgs += @('--follower_port', $FollowerPort)
    }
    if ($Mode -eq 'Preflight') { $controllerArgs += '--preflight_only' }
    if ($ShowCameras) { $controllerArgs += @('--show_cameras', '--camera_indices', $FixedCamera, $WristCamera) }
}
if ($RequireCameraAlignment) { $controllerArgs += '--require_camera_alignment' }
if ($Layout) { $controllerArgs += @('--layout', (Resolve-Path -LiteralPath $Layout).Path) }
& $IsaacPython @controllerArgs
exit $LASTEXITCODE
