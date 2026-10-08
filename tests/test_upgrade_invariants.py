"""Upgrade guards: fail when a LeRobot upgrade silently changes this project's settings.

Each test pins something the SO-101 setup depends on but that upstream is free to change: CLI flag
names the wrapper scripts pass, the motor register profile, the calibration file format, video
encoding defaults the datasets were recorded with, and that the project scripts still start.
Run on every upgrade (CI does). No hardware: nothing here opens a serial port or a camera.
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

REPO = Path(__file__).resolve().parents[1]
SCRIPTS = REPO / "scripts"

FOLLOWER_MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def _cli_help(module: str, *type_args: str) -> str:
    out = subprocess.run(
        [sys.executable, "-m", f"lerobot.scripts.{module}", *type_args, "--help"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        timeout=300,
        cwd=REPO,
    )
    assert out.returncode == 0, out.stderr[-2000:]
    return out.stdout


def _wrapper_flags(ps1: str) -> set[str]:
    """Every `--flag=` the PowerShell wrapper passes to the CLI."""
    return set(re.findall(r'"--([A-Za-z_][\w.]*)=', (SCRIPTS / ps1).read_text(encoding="utf-8")))


# --------------------------------------------------------------------------- CLI flags used by wrappers

ROBOT_TYPES = ("--robot.type=so101_follower", "--teleop.type=so101_leader")


@pytest.mark.parametrize(
    "ps1, module, type_args",
    [
        ("run_eval.ps1", "lerobot_rollout", ("--strategy.type=episodic", ROBOT_TYPES[0])),
    ],
)
def test_wrapper_flags_exist_in_cli(ps1, module, type_args):
    help_text = _cli_help(module, *type_args)
    # policy.* overrides are applied to the checkpoint's own config, so they're checked separately below.
    flags = {f for f in _wrapper_flags(ps1) if not f.startswith("policy.") or f == "policy.path"}
    missing = sorted(f for f in flags if f"--{f}" not in help_text and f not in ("strategy.type", "robot.type"))
    assert not missing, f"{ps1} passes flags that {module} no longer accepts: {missing}"


@pytest.mark.parametrize(
    "ps1, script", [("run_record.ps1", "record_episodes"), ("record_pickplace.ps1", "record_pickplace")]
)
def test_wrapper_flags_exist_in_script(ps1, script):
    out = subprocess.run(
        [sys.executable, str(SCRIPTS / f"{script}.py"), "--help"],
        capture_output=True, text=True, encoding="utf-8", timeout=300, cwd=REPO,
        env={**__import__("os").environ, "PYTHONUTF8": "1"},
    )
    assert out.returncode == 0, out.stderr[-2000:]
    missing = sorted(f for f in _wrapper_flags(ps1) if f"--{f}" not in out.stdout)
    assert not missing, f"{ps1} passes flags that {script}.py doesn't accept: {missing}"


def test_eval_policy_overrides_exist():
    """run_eval.ps1 forwards --policy.device / use_amp (all policies) and num_steps (SmolVLA)."""
    import lerobot.policies  # noqa: F401 — registers the policy config choices
    from lerobot.configs.policies import PreTrainedConfig

    flags = {f.removeprefix("policy.") for f in _wrapper_flags("run_eval.ps1") if f.startswith("policy.")}
    flags.discard("path")
    flags |= {m for m in re.findall(r"--policy\.(\w+)=", (SCRIPTS / "run_eval.ps1").read_text(encoding="utf-8"))} - {
        "path"
    }
    act = PreTrainedConfig.get_choice_class("act")()
    smolvla = PreTrainedConfig.get_choice_class("smolvla")()
    for flag in flags:
        assert hasattr(smolvla, flag), f"SmolVLA config has no '{flag}' (run_eval.ps1 passes --policy.{flag})"
        if flag != "num_steps":
            assert hasattr(act, flag), f"ACT config has no '{flag}' (run_eval.ps1 passes --policy.{flag})"


def test_teleop_flags_from_setup_guide_exist():
    help_text = _cli_help("lerobot_teleoperate", *ROBOT_TYPES)
    for flag in ["fps", "display_data", "display_compressed_images", "display_cameras", "robot.cameras", "robot.port"]:
        assert f"--{flag}" in help_text, f"lerobot-teleoperate no longer accepts --{flag}"


# --------------------------------------------------------------------------- hardware profile


def _follower(calibration_dir: Path):
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    return SO101Follower(SO101FollowerConfig(port="COM99", id="my_so_arm", calibration_dir=calibration_dir))


def test_follower_motor_layout():
    from lerobot.motors import MotorNormMode

    robot = _follower(REPO / "calibration" / "robots" / "so_follower")
    motors = robot.bus.motors
    assert list(motors) == FOLLOWER_MOTORS
    assert [m.id for m in motors.values()] == [1, 2, 3, 4, 5, 6]
    assert all(m.model == "sts3215" for m in motors.values())
    assert motors["gripper"].norm_mode == MotorNormMode.RANGE_0_100
    assert all(motors[n].norm_mode == MotorNormMode.DEGREES for n in FOLLOWER_MOTORS[:-1])


def test_follower_motor_register_profile(tmp_path):
    """The exact values configure() writes to the motors (gains, torque and overload protection)."""
    robot = _follower(tmp_path)
    writes: dict[tuple[str, str], int] = {}
    robot.bus.write = MagicMock(side_effect=lambda reg, motor, value, **kw: writes.__setitem__((motor, reg), value))
    robot.bus.configure_motors = MagicMock()
    robot.bus.torque_disabled = MagicMock()

    robot.configure()

    common = {"Operating_Mode": 0, "P_Coefficient": 16, "I_Coefficient": 0, "D_Coefficient": 32}
    arm = {**common, "Max_Torque_Limit": 1000, "Protection_Current": 500, "Overload_Torque": 40}
    gripper = {**common, "Max_Torque_Limit": 500, "Protection_Current": 250, "Overload_Torque": 25}
    for motor in FOLLOWER_MOTORS:
        expected = gripper if motor == "gripper" else arm
        actual = {reg: writes.get((motor, reg)) for reg in expected}
        assert actual == expected, f"{motor} register profile changed"


def test_gripper_stall_relief_constants():
    from lerobot.robots.so_follower import so_follower

    assert so_follower.GRIPPER_HOLD_SQUEEZE_TICKS == 50
    assert so_follower.GRIPPER_STALL_S == 0.5


@pytest.mark.parametrize(
    "kind, folder",
    [("robot", "calibration/robots/so_follower"), ("teleop", "calibration/teleoperators/so_leader")],
)
def test_calibration_files_load(kind, folder):
    """The saved calibration must load exactly as stored, motor for motor, under the upgraded library.

    Compares against the JSON itself rather than pinned numbers, so recalibrating the arm doesn't
    trip this guard but a changed calibration format/loader does.
    """
    import json
    from dataclasses import asdict

    if kind == "robot":
        device = _follower(REPO / folder)
    else:
        from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

        device = SO101Leader(SO101LeaderConfig(port="COM98", id="my_so_arm", calibration_dir=REPO / folder))
    stored = json.loads((REPO / folder / "my_so_arm.json").read_text(encoding="utf-8"))
    assert list(device.calibration) == FOLLOWER_MOTORS
    assert {name: asdict(cal) for name, cal in device.calibration.items()} == stored


def test_opencv_has_gui_support():
    """The camera windows (record_pickplace preview, --display_cameras) need a GUI OpenCV build.

    LeRobot depends on opencv-python-headless; installing it alongside or instead of opencv-python
    leaves cv2 without HighGUI and cv2.namedWindow raises "The function is not implemented".
    """
    import cv2

    gui = next(line for line in cv2.getBuildInformation().splitlines() if line.strip().startswith("GUI:"))
    assert "NONE" not in gui, f"cv2 has no GUI backend ({gui.strip()}); camera windows will crash"


def test_project_camera_config_is_accepted():
    from lerobot.cameras.opencv import OpenCVCameraConfig

    cfg = OpenCVCameraConfig(index_or_path=0, width=640, height=480, fps=30, fourcc="MJPG")
    assert (cfg.width, cfg.height, cfg.fps, cfg.fourcc) == (640, 480, 30, "MJPG")
    assert hasattr(cfg, "exposure") and hasattr(cfg, "white_balance_temperature")


# --------------------------------------------------------------------------- dataset / video consistency


def test_video_encoding_matches_existing_datasets():
    """Datasets so far were encoded yuv420p, GOP 2, CRF 30; new episodes must decode the same way."""
    from lerobot.configs.video import RGBEncoderConfig

    enc = RGBEncoderConfig(vcodec="h264")
    assert (enc.pix_fmt, enc.g, enc.crf, enc.fast_decode) == ("yuv420p", 2, 30, 0)


def test_dataset_record_defaults():
    from lerobot.configs.dataset import DatasetRecordConfig

    fields = DatasetRecordConfig.__dataclass_fields__
    assert fields["fps"].default == 30
    assert "no_stamp" in fields, "wrappers pass --dataset.no_stamp=true to keep repo_ids as typed"


# --------------------------------------------------------------------------- scripts still start

# Only scripts whose --help returns without touching hardware. Never add goto_start_pose, verify_calib,
# test_servos or apply_calib: they connect to the arm immediately and ignore --help.
HELP_SAFE_SCRIPTS = [
    "record_pickplace",
    "record_episodes",
    "replay_pickplace",
    "measure_placement_error",
    "log_eval_to_wandb",
    "make_eval_scoresheet",
    "validate_dataset",
    "score_eval_frames",
    "trim_idle_frames",
    "swap_camera_keys",
    "probe_start_hesitation",
]


@pytest.mark.parametrize("script", HELP_SAFE_SCRIPTS)
def test_script_starts(script):
    out = subprocess.run(
        [sys.executable, str(SCRIPTS / f"{script}.py"), "--help"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        timeout=300,
        cwd=REPO,
        env={**__import__("os").environ, "PYTHONUTF8": "1"},
    )
    assert out.returncode == 0, out.stderr[-2000:]


# --------------------------------------------------------------------------- merge hygiene


def test_no_merge_conflict_markers():
    marker = re.compile(r"^(?:<{7}|>{7})(?: |$)", re.MULTILINE)
    offenders = [
        str(p.relative_to(REPO))
        for root in ("src", "scripts", "tests")
        for p in (REPO / root).rglob("*")
        if p.suffix in {".py", ".ps1", ".toml", ".md", ".json", ".yaml", ".yml"}
        and not p.is_symlink()  # symlinks are checked by test_no_dangling_symlinks
        and marker.search(p.read_text(encoding="utf-8", errors="ignore"))
    ]
    assert not offenders, f"leftover merge-conflict markers: {offenders}"


def test_no_dangling_symlinks():
    """Upstream links some policy READMEs into docs/, which this repo doesn't carry.

    Read from git rather than the disk: a Windows checkout turns symlinks into plain files, so a
    dangling link only breaks on Linux/CI checkouts.
    """
    git = lambda *a: subprocess.run(["git", *a], capture_output=True, text=True, cwd=REPO, check=True).stdout  # noqa: E731
    tracked = set(git("ls-files").splitlines())
    dangling = []
    for line in git("ls-files", "-s").splitlines():
        mode, _, _, path = line.split(maxsplit=3)
        if mode != "120000":
            continue
        target = git("cat-file", "-p", f":{path}").strip()
        resolved = (Path(path).parent / target).as_posix()
        resolved = str(Path(__import__("os").path.normpath(resolved)).as_posix())
        if resolved not in tracked:
            dangling.append(f"{path} -> {target}")
    assert not dangling, f"symlinks to files this repo doesn't carry: {dangling}"
