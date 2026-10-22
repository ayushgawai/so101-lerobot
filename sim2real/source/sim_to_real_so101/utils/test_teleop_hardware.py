"""Hardware failure-path tests; no serial ports or Isaac runtime are opened."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

from sim_to_real_so101.utils.dual_teleop_control import KEYS
from sim_to_real_so101.utils.teleop_hardware import (
    TeleopSO101Interface, connect_calibrated, validate_calibration,
)


class TeleopHardwareTests(unittest.TestCase):
    def robot(self):
        robot = Mock()
        robot.is_calibrated = True
        robot.bus.is_connected = True
        robot.bus.sync_read.return_value = {key.removesuffix('.pos'): 0.0 for key in KEYS}
        return robot

    def test_hold_precedes_configuration_and_torque_enable(self):
        robot = self.robot()
        connect_calibrated(robot, hold_current=True)
        calls = [call[0] for call in robot.mock_calls]
        self.assertLess(calls.index('bus.sync_write'), calls.index('configure'))
        robot.bus.sync_write.assert_called_once_with('Goal_Position', robot.bus.sync_read.return_value)
        robot.calibrate.assert_not_called()

    def test_mismatched_eeprom_never_configures_or_writes_goals(self):
        robot = self.robot()
        robot.is_calibrated = False
        with self.assertRaisesRegex(RuntimeError, 'EEPROM differs'):
            connect_calibrated(robot, True)
        robot.configure.assert_not_called()
        robot.bus.sync_write.assert_not_called()
        robot.bus.disconnect.assert_called_once_with(disable_torque=True)

    def test_invalid_read_does_not_enable_motion(self):
        for value in (float('nan'), 101):
            robot = self.robot()
            robot.bus.sync_read.return_value['shoulder_pan'] = value
            with self.assertRaises(ValueError):
                connect_calibrated(robot, True)
            robot.configure.assert_not_called()
            robot.bus.sync_write.assert_not_called()
            robot.bus.disconnect.assert_called_once_with(disable_torque=True)

    def test_partial_configuration_failure_disables_torque(self):
        robot = self.robot()
        robot.configure.side_effect = OSError('serial disconnected')
        with self.assertRaisesRegex(OSError, 'serial disconnected'):
            connect_calibrated(robot, True)
        robot.bus.disconnect.assert_called_once_with(disable_torque=True)

    def test_mapping_neutral_and_endpoints(self):
        interface = TeleopSO101Interface('cpu', 'COM4', 'test', {}, 30)
        neutral = dict.fromkeys(KEYS, 0)
        self.assertEqual(interface.mapped_degrees(neutral), [0, 0, -5, 0, 0, -10])
        self.assertEqual(interface.mapped_degrees(dict.fromkeys(KEYS, 100)), [110, 100, 90, 95, 160, 100])
        with self.assertRaises(ValueError):
            interface.mapped_degrees(dict.fromkeys(KEYS, float('nan')))

    def test_both_configs_explicitly_use_normalized_units_and_own_calibration(self):
        for kind, package, name, directory in (
            ('leader', 'teleoperators.so_leader', 'Leader', 'teleoperators/so_leader'),
            ('follower', 'robots.so_follower', 'Follower', 'robots/so_follower'),
        ):
            config, hardware = Mock(), Mock()
            stub = SimpleNamespace(**{f'SO101{name}Config': config, f'SO101{name}': hardware})
            with patch.dict('sys.modules', {f'lerobot.{package}': stub}), patch.dict(
                'os.environ', {'HF_LEROBOT_CALIBRATION': '/test/calibration'}
            ):
                interface = TeleopSO101Interface('cpu', 'COM4', 'my_so_arm', {}, 30, kind)
                interface.init_device()
            self.assertIs(config.call_args.kwargs['use_degrees'], False)
            self.assertEqual(config.call_args.kwargs['calibration_dir'], Path('/test/calibration') / directory)
            hardware.assert_called_once_with(config.return_value)

    def test_supplied_calibrations_and_bad_ranges(self):
        root = Path(__file__).resolve().parents[4] / 'calibration'
        for relative in ('teleoperators/so_leader', 'robots/so_follower'):
            data = validate_calibration(root / relative / 'my_so_arm.json')
            self.assertEqual(len(data), 6)
            data['gripper']['range_max'] = data['gripper']['range_min']
            with tempfile.TemporaryDirectory() as temp:
                path = Path(temp) / 'bad.json'
                path.write_text(json.dumps(data))
                with self.assertRaisesRegex(ValueError, 'invalid encoder range'):
                    validate_calibration(path)


if __name__ == '__main__':
    unittest.main()
