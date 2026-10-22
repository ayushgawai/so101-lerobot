"""No hardware/Kit dependencies: python -m unittest discover -s .../utils."""
import unittest

from dual_teleop_control import DualActionGate, KEYS, checked_action


def action(value=0):
    return dict.fromkeys(KEYS, value)


class DualGateTests(unittest.TestCase):
    def test_initially_paused_and_explicit_pause(self):
        gate = DualActionGate()
        self.assertIsNone(gate.next_action(action(), 0))
        gate.arm(action(), action(), 0)
        gate.pause()
        self.assertIsNone(gate.next_action(action(50), 0.1))

    def test_start_pose_must_match(self):
        gate = DualActionGate()
        leader, follower = action(), action()
        leader['wrist_roll.pos'] = 37
        follower['shoulder_lift.pos'] = -20
        leader['gripper.pos'] = 15  # Exactly at the limit is acceptable.
        with self.assertRaises(ValueError) as error:
            gate.arm(leader, follower, 0)
        self.assertFalse(gate.active)
        self.assertIsNone(gate.next_action(leader, .1))
        message = str(error.exception)
        self.assertIn('maximum difference 37.0; limit 15.0', message)
        self.assertIn('wrist_roll: leader=+37.0, follower=+0.0; change leader reading by -37.0', message)
        self.assertIn('shoulder_lift: leader=+0.0, follower=-20.0; change leader reading by -20.0', message)
        self.assertLess(message.index('wrist_roll:'), message.index('shoulder_lift:'))
        self.assertNotIn('gripper:', message)
        leader['wrist_roll.pos'] = 0
        leader['shoulder_lift.pos'] = -20
        gate.arm(leader, follower, .2)
        self.assertTrue(gate.active)

    def test_slew_is_time_based_and_output_is_copy(self):
        gate = DualActionGate(units_per_second=60, max_step_s=.1)
        gate.arm(action(), action(), 0)
        first = gate.next_action(action(100), 0.1)
        self.assertEqual(first, action(6))
        first[KEYS[0]] = 100
        self.assertEqual(gate.next_action(action(100), 0.2), action(12))
        self.assertEqual(gate.next_action(action(10), 0.3), action(10))

    def test_gripper_start_mismatch_ramps_from_observed_position(self):
        gate = DualActionGate(allow_gripper_start_mismatch=True)
        leader, follower = action(), action()
        leader['gripper.pos'], follower['gripper.pos'] = 2.1, 39.1
        gate.arm(leader, follower, 0)
        self.assertAlmostEqual(gate.next_action(leader, 0)['gripper.pos'], 39.1)
        self.assertAlmostEqual(gate.next_action(leader, 1/30)['gripper.pos'], 37.1)
        for tick in range(2, 21):
            output = gate.next_action(leader, tick/30)
        self.assertAlmostEqual(output['gripper.pos'], 2.1)
        self.assertTrue(all(output[key] == 0 for key in KEYS[:-1]))

    def test_gripper_exemption_keeps_arm_pose_check_and_input_validation(self):
        gate = DualActionGate(allow_gripper_start_mismatch=True)
        leader = action()
        leader['gripper.pos'] = 100
        leader['wrist_roll.pos'] = 16
        with self.assertRaisesRegex(ValueError, 'wrist_roll:'):
            gate.arm(leader, action(), 0)
        self.assertFalse(gate.active)
        leader['wrist_roll.pos'] = 0
        for invalid in (-1, 101, float('nan')):
            leader['gripper.pos'] = invalid
            with self.assertRaises(ValueError):
                gate.arm(leader, action(), 0)
            self.assertFalse(gate.active)

    def test_resume_gripper_ramp_uses_fresh_follower_reading(self):
        gate = DualActionGate(allow_gripper_start_mismatch=True)
        gate.arm(action(), action(), 0)
        gate.pause()
        follower = action()
        follower['gripper.pos'] = 50
        gate.arm(action(), follower, 10)
        self.assertEqual(gate.next_action(action(), 10)['gripper.pos'], 50)
        self.assertAlmostEqual(gate.next_action(action(), 10.1)['gripper.pos'], 48)

    def test_default_gate_still_requires_gripper_match(self):
        leader = action()
        leader['gripper.pos'] = 37
        with self.assertRaisesRegex(ValueError, 'gripper:'):
            DualActionGate().arm(leader, action(), 0)

    def test_reported_arm_mismatch_converges_in_explicit_ramp_mode(self):
        gate = DualActionGate(startup_mode='ramp', allow_gripper_start_mismatch=True)
        leader, follower = action(), action()
        for key, target, current in (
            ('shoulder_lift.pos', -97.2, -51.0),
            ('wrist_flex.pos', 50.8, 33.8),
            ('elbow_flex.pos', 99.8, 83.6),
        ):
            leader[key], follower[key] = target, current
        gate.arm(leader, follower, 0)
        self.assertEqual(gate.next_action(leader, 0), follower)
        self.assertTrue(gate.startup_ramping)
        previous = follower
        for tick in range(1, 151):
            was_ramping = gate.startup_ramping
            output = gate.next_action(leader, tick/30)
            limit = (10 if was_ramping else 60)/30
            for key in KEYS:
                self.assertLessEqual(abs(output[key]-previous[key]), limit+1e-9)
            previous = output
        self.assertEqual(output, leader)
        self.assertFalse(gate.startup_ramping)

    def test_ramp_timeout_and_resume_use_fresh_pose(self):
        gate = DualActionGate(startup_mode='ramp', timeout_s=.5, max_step_s=.1)
        gate.arm(action(90), action(), 0)
        self.assertIsNone(gate.next_action(action(90), .6))
        self.assertFalse(gate.active)
        gate.arm(action(90), action(20), 1)
        self.assertEqual(gate.next_action(action(90), 1), action(20))
        self.assertEqual(gate.next_action(action(90), 1.1), action(21))
        invalid = action(90)
        invalid['elbow_flex.pos'] = float('nan')
        with self.assertRaises(ValueError):
            gate.arm(invalid, action(20), 2)
        self.assertFalse(gate.active)

    def test_ramp_parameters_validated(self):
        with self.assertRaises(ValueError):
            DualActionGate(startup_mode='skip')
        for value in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                DualActionGate(startup_units_per_second=value)

    def test_stall_backwards_and_nonfinite_time_pause(self):
        for now in (2.001, -1, float("nan"), float("inf")):
            gate = DualActionGate()
            gate.arm(action(), action(), 0)
            self.assertIsNone(gate.next_action(action(50), now))
            self.assertFalse(gate.active)

    def test_reported_727ms_gap_continues_without_catchup_jump(self):
        gate = DualActionGate()
        gate.arm(action(), action(), 0)
        first = gate.next_action(action(100), 1/30)
        delayed_at = 1/30 + .727
        delayed = gate.next_action(action(100), delayed_at)
        self.assertTrue(gate.active)
        self.assertAlmostEqual(gate.last_gap_s, .727)
        for key in KEYS:
            self.assertAlmostEqual(first[key], 2)
            self.assertAlmostEqual(delayed[key]-first[key], 2)
        # Discarded time must not carry forward into subsequent commands.
        following = gate.next_action(action(100), delayed_at+1/30)
        for key in KEYS:
            self.assertAlmostEqual(following[key], 6)

    def test_repeated_short_stalls_do_not_pause_or_accumulate_credit(self):
        gate = DualActionGate()
        gate.arm(action(), action(), 0)
        now = 0.0
        for tick, gap in enumerate((.727, .9, 1.999, 1/30), 1):
            now += gap
            output = gate.next_action(action(100), now)
            self.assertTrue(gate.active)
            for value in output.values():
                self.assertAlmostEqual(value, 2*tick)

    def test_startup_ramp_keeps_slow_step_after_render_stall(self):
        gate = DualActionGate(startup_mode='ramp')
        gate.arm(action(90), action(), 0)
        output = gate.next_action(action(90), .727)
        self.assertTrue(gate.active)
        self.assertTrue(gate.startup_ramping)
        for value in output.values():
            self.assertAlmostEqual(value, 10/30)

    def test_configured_hard_timeout_still_requires_manual_rearm(self):
        gate = DualActionGate(timeout_s=.5)
        gate.arm(action(), action(), 0)
        self.assertIsNone(gate.next_action(action(100), .727))
        self.assertFalse(gate.active)
        self.assertIsNone(gate.next_action(action(100), .8))
        gate.arm(action(), action(), .9)
        self.assertIsNotNone(gate.next_action(action(100), 1))

    def test_timeout_and_step_cap_parameters_validated(self):
        for field in ('timeout_s', 'max_step_s'):
            for value in (0, -1, float('nan'), float('inf')):
                with self.assertRaises(ValueError):
                    DualActionGate(**{field: value})

    def test_invalid_sample_pauses(self):
        for value in (float("nan"), float("inf"), 101, -101):
            gate = DualActionGate()
            gate.arm(action(), action(), 0)
            with self.assertRaises(ValueError):
                gate.next_action(action(value), .1)
            self.assertFalse(gate.active)

    def test_missing_joint_rejected(self):
        with self.assertRaises(KeyError):
            checked_action({})

    def test_gripper_has_unsigned_range(self):
        target = action(-10)
        with self.assertRaises(ValueError):
            checked_action(target)
        target["gripper.pos"] = 0
        self.assertEqual(checked_action(target), target)

    def test_limits_and_arm_time_validated(self):
        for value in (0, -1, float("inf"), float("nan")):
            with self.assertRaises(ValueError):
                DualActionGate(units_per_second=value)
        with self.assertRaises(ValueError):
            DualActionGate().arm(action(), action(), float("nan"))


if __name__ == "__main__":
    unittest.main()
