"""Pure control-state logic shared by the raw dual-arm teleoperation loop/tests."""
import math

KEYS = ("shoulder_pan.pos", "shoulder_lift.pos", "elbow_flex.pos",
        "wrist_flex.pos", "wrist_roll.pos", "gripper.pos")


def checked_action(action):
    result = {key: float(action[key]) for key in KEYS}
    for key, value in result.items():
        lower = 0 if key == "gripper.pos" else -100
        if not math.isfinite(value) or not lower <= value <= 100:
            raise ValueError(f"Invalid normalized position {key}={value}; expected {lower}..100")
    return result


class DualActionGate:
    """Explicit arming and a shared, time-based target slew limit for both outputs.

    Units are LeRobot normalized units, not degrees. Short render/serial stalls
    discard elapsed movement time; long gaps pause and require explicit rearming.
    """
    def __init__(self, units_per_second=60.0, max_start_difference=15.0, timeout_s=2.0,
                 allow_gripper_start_mismatch=False, startup_mode="match", startup_units_per_second=10.0,
                 max_step_s=1/30):
        if not all(math.isfinite(v) and v > 0 for v in (units_per_second, max_start_difference, timeout_s, startup_units_per_second, max_step_s)):
            raise ValueError("Control limits must be positive finite values")
        if startup_mode not in ("match", "ramp"):
            raise ValueError("startup_mode must be match or ramp")
        self.speed = units_per_second
        self.startup_mode = startup_mode
        self.startup_speed = min(startup_units_per_second, units_per_second)
        self.startup_ramping = False
        self.max_start_difference = max_start_difference
        self.timeout_s = timeout_s
        self.max_step_s = max_step_s
        self.last_gap_s = 0.0
        self.allow_gripper_start_mismatch = allow_gripper_start_mismatch
        self.active = False
        self.last_action = None
        self.last_time = None
        self.reason = "Paused; align the leader with the follower, then press F8"

    def arm(self, leader, follower, now):
        self.pause("Checking start pose")
        if not math.isfinite(float(now)):
            raise ValueError("Control timestamp must be finite")
        target, current = checked_action(leader), checked_action(follower)
        # A gripper aperture mismatch can converge through the same slew limiter.
        # The five arm joints still need matching poses before movement starts.
        start_keys = KEYS[:-1] if self.allow_gripper_start_mismatch else KEYS
        difference = max(abs(target[k]-current[k]) for k in start_keys)
        if self.startup_mode == "match" and difference > self.max_start_difference:
            mismatches = sorted(
                (k for k in start_keys if abs(target[k]-current[k]) > self.max_start_difference),
                key=lambda k: abs(target[k]-current[k]), reverse=True,
            )
            details = [
                f"  {k.removesuffix('.pos')}: leader={target[k]:+.1f}, "
                f"follower={current[k]:+.1f}; change leader reading by {current[k]-target[k]:+.1f}"
                for k in mismatches
            ]
            self.reason = (
                f"Start rejected: maximum difference {difference:.1f}; "
                f"limit {self.max_start_difference:.1f} normalized units (not degrees).\n"
                + "\n".join(details)
                + "\nMove the listed leader joints toward the follower readings. "
                "Leave the powered follower in place, then press F8 to recheck."
            )
            raise ValueError(self.reason)
        # Always seed ALL six targets from the observed follower, including on
        # resume. Exempting the gripper from pose matching must never cause a snap.
        self.last_action = current
        self.last_time = float(now)
        self.last_gap_s = 0.0
        self.startup_ramping = self.startup_mode == "ramp" and any(
            abs(target[k]-current[k]) > 1.0 for k in KEYS[:-1]
        )
        self.active = True
        self.reason = "Running: F9 pauses; Esc exits"

    def pause(self, reason="Paused; F8 resumes after a start-pose check"):
        self.active = False
        self.startup_ramping = False
        self.reason = reason

    def next_action(self, leader, now):
        if not self.active:
            return None
        try:
            target = checked_action(leader)
        except (ValueError, KeyError, TypeError):
            self.pause("Invalid leader sample; paused")
            raise
        dt = float(now)-self.last_time
        self.last_gap_s = dt
        if not math.isfinite(dt) or dt < 0 or dt > self.timeout_s:
            self.pause(f"Control gap {dt:.3f}s exceeded {self.timeout_s:.3f}s limit or timestamp was invalid; press F8 to resume")
            return None
        # Never accumulate movement credit while rendering/serial I/O stalls.
        # The fresh leader sample can advance the target by at most one step,
        # even if the operator moved the leader far during the gap.
        step = (self.startup_speed if self.startup_ramping else self.speed)*min(dt, self.max_step_s)
        self.last_action = {k: self.last_action[k]+max(-step, min(step, target[k]-self.last_action[k])) for k in KEYS}
        # Check command convergence on arm joints; contact-limited gripper
        # closure must not keep all joints in startup mode indefinitely.
        if self.startup_ramping and all(abs(target[k]-self.last_action[k]) <= 1.0 for k in KEYS[:-1]):
            self.startup_ramping = False
        self.last_time = float(now)
        return dict(self.last_action)
