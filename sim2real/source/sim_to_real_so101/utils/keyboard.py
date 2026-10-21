# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import carb
import omni.appwindow
import omni.kit.app

from carb.eventdispatcher import get_eventdispatcher, Event

class KeyboardControl:

    START_RECORDING_EVENT: str = "lerobot_so101_teleop.start_recording"
    STOP_RECORDING_EVENT: str = "lerobot_so101_teleop.stop_recording"
    CANCEL_RECORDING_EVENT: str = "lerobot_so101_teleop.cancel_recording"

    def __init__(self):
        self.reset_world = False
        self.recording = False

        # Get the window to register keyboard callbacks
        self._window = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._window.get_keyboard()

        # Register keyboard callbacks
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._on_keyboard_event
        )

    def _on_keyboard_event(self, event, *args, **kwargs):
        """Keyboard event handler"""
        # Only process key press events
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name == "R":
                self.reset_world = True
                self.stop_recording()
                print(f"[INFO]: Reset world...")
                return True

            if event.input.name == "S":
                if self.recording:
                    self.stop_recording()
                    return True

                
                self.start_recording()
                return True

            if event.input.name == "C":
                if self.recording:
                    self.cancel_recording()
                    return True

        return False



    def cleanup(self):
        """Cleanup the keyboard interface"""
        if self._sub_keyboard:
            self._input.unsubscribe_to_keyboard_events(
                self._keyboard, self._sub_keyboard
            )
            self._sub_keyboard = None

    # This should not live in this class, but it works for now
    def start_recording(self):
        if not self.recording:
            print(f"[INFO]: Started recording.")
            self.recording = True

            omni.kit.app.queue_event(
                self.START_RECORDING_EVENT, 
                payload={}
                )

    def stop_recording(self):
        if self.recording:
            print(f"[INFO]: Stopped recording.")
            self.recording = False

            omni.kit.app.queue_event(
                self.STOP_RECORDING_EVENT, 
                payload={}
                )

    def cancel_recording(self):
        if self.recording:
            print(f"[INFO]: Cancelled recording.")
            self.recording = False

            omni.kit.app.queue_event(
                self.CANCEL_RECORDING_EVENT,
                payload={}
                )


class JointJogKeyboardControl(KeyboardControl):
    """Extends KeyboardControl with continuous per-joint jogging.

    Tracks which jog keys are currently held (not just pressed) so a caller
    can poll ``get_joint_deltas`` once per physics step and get a continuous
    +/- delta for as long as a key stays down, rather than a one-shot flag.
    """

    JOINT_ORDER = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]

    # (increase_key, decrease_key) per joint. Deliberately disjoint from R/S/C
    # (reset/start-stop-recording/cancel-recording) so no key does double duty.
    JOINT_KEYS = {
        "Rotation": ("D", "A"),
        "Pitch": ("W", "X"),
        "Elbow": ("E", "V"),
        "Wrist_Pitch": ("T", "G"),
        "Wrist_Roll": ("Y", "H"),
        "Jaw": ("U", "J"),
    }

    def __init__(self):
        self._held_keys = set()
        self._key_lookup = {}
        for joint_index, joint in enumerate(self.JOINT_ORDER):
            pos_key, neg_key = self.JOINT_KEYS[joint]
            self._key_lookup[pos_key] = (joint_index, 1.0)
            self._key_lookup[neg_key] = (joint_index, -1.0)
        super().__init__()

    def _on_keyboard_event(self, event, *args, **kwargs):
        # Let the base class handle R/S/C first.
        if super()._on_keyboard_event(event, *args, **kwargs):
            return True

        # Kit also dispatches non-press/release event types (e.g. CHAR) on
        # this same callback, where `event.input` is a plain str with no
        # `.name` -- only KEY_PRESS/KEY_RELEASE carry a KeyboardInput enum.
        # Confirmed by hitting `AttributeError: 'str' object has no
        # attribute 'name'` during a smoke test: some non-key event fired
        # continuously even with no human at the keyboard.
        if event.type not in (
            carb.input.KeyboardEventType.KEY_PRESS,
            carb.input.KeyboardEventType.KEY_RELEASE,
        ):
            return False

        key_name = event.input.name
        if key_name not in self._key_lookup:
            return False

        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self._held_keys.add(key_name)
        else:
            self._held_keys.discard(key_name)
        return True

    def get_joint_deltas(self, step: float) -> list[float]:
        """Per-joint delta for this tick, ``+step``/``-step`` per held key.

        A joint with both its keys held cancels out to 0.
        """
        deltas = [0.0] * len(self.JOINT_ORDER)
        for key_name in self._held_keys:
            joint_index, sign = self._key_lookup[key_name]
            deltas[joint_index] += sign * step
        return deltas