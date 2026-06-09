#!/usr/bin/env python3
"""Linux joystick helpers for OpenDoge command-file control."""

from __future__ import annotations

import array
import dataclasses
import errno
import fcntl
import glob
import os
import struct
from typing import Dict, Iterable, List, Optional


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JSIOCGNAME_128 = 0x80806A13
EVENT_STRUCT = struct.Struct("IhBB")


@dataclasses.dataclass(frozen=True)
class JoystickEvent:
    timestamp_ms: int
    value: int
    event_type: int
    number: int

    @property
    def is_init(self) -> bool:
        return bool(self.event_type & JS_EVENT_INIT)

    @property
    def base_type(self) -> int:
        return self.event_type & ~JS_EVENT_INIT


class LinuxJoystick:
    """Non-blocking reader for /dev/input/js* devices."""

    def __init__(self, device: Optional[str] = None):
        self.device = device or first_joystick_device()
        self.fd: Optional[int] = None

    def open(self) -> None:
        if not self.device:
            raise FileNotFoundError("no /dev/input/js* joystick device found")
        self.fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def __enter__(self) -> "LinuxJoystick":
        self.open()
        return self

    def __exit__(self, *_args) -> None:
        self.close()

    def name(self) -> str:
        if self.fd is None:
            return ""
        buf = array.array("B", [0] * 128)
        try:
            fcntl.ioctl(self.fd, JSIOCGNAME_128, buf, True)
        except OSError:
            return ""
        raw = bytes(buf).split(b"\0", 1)[0]
        return raw.decode("utf-8", errors="replace")

    def read_events(self) -> List[JoystickEvent]:
        if self.fd is None:
            raise RuntimeError("joystick is not open")

        events: List[JoystickEvent] = []
        while True:
            try:
                data = os.read(self.fd, EVENT_STRUCT.size)
            except BlockingIOError:
                break
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    break
                raise

            if len(data) != EVENT_STRUCT.size:
                break
            events.append(JoystickEvent(*EVENT_STRUCT.unpack(data)))
        return events


def first_joystick_device() -> Optional[str]:
    devices = sorted(glob.glob("/dev/input/js*"))
    return devices[0] if devices else None


def apply_deadzone(value: float, deadzone: float) -> float:
    value = max(-1.0, min(1.0, value))
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0.0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def normalize_axis(raw_value: int) -> float:
    if raw_value < 0:
        return max(-1.0, raw_value / 32768.0)
    return min(1.0, raw_value / 32767.0)


@dataclasses.dataclass
class RobotCommand:
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    active: bool = False
    estop: bool = False


class XboxCommandMapper:
    """Maps common Xbox joystick axes/buttons to OpenDoge command fields."""

    BUTTON_A = 0
    BUTTON_B = 1
    BUTTON_X = 2
    BUTTON_Y = 3
    BUTTON_BACK = 6
    BUTTON_START = 7
    BUTTON_RB = 5

    def __init__(
        self,
        max_vx: float,
        max_vy: float,
        max_yaw_rate: float,
        deadzone: float,
        axis_vy: int = 0,
        axis_vx: int = 1,
        axis_yaw: int = 3,
        require_rb: bool = False,
    ):
        self.max_vx = max_vx
        self.max_vy = max_vy
        self.max_yaw_rate = max_yaw_rate
        self.deadzone = deadzone
        self.axis_vy = axis_vy
        self.axis_vx = axis_vx
        self.axis_yaw = axis_yaw
        self.require_rb = require_rb
        self.axes: Dict[int, float] = {}
        self.buttons: Dict[int, bool] = {}
        self.active_requested = False
        self.estop = False

    def update(self, events: Iterable[JoystickEvent]) -> None:
        for event in events:
            if event.base_type == JS_EVENT_AXIS:
                self.axes[event.number] = normalize_axis(event.value)
            elif event.base_type == JS_EVENT_BUTTON:
                pressed = event.value != 0
                self.buttons[event.number] = pressed
                if pressed and not event.is_init:
                    self._handle_button_down(event.number)

    def command(self) -> RobotCommand:
        vx_axis = apply_deadzone(-self.axes.get(self.axis_vx, 0.0), self.deadzone)
        vy_axis = apply_deadzone(self.axes.get(self.axis_vy, 0.0), self.deadzone)
        yaw_axis = apply_deadzone(self.axes.get(self.axis_yaw, 0.0), self.deadzone)

        active = self.active_requested and not self.estop
        if self.require_rb and not self.buttons.get(self.BUTTON_RB, False):
            active = False

        if not active:
            vx_axis = 0.0
            vy_axis = 0.0
            yaw_axis = 0.0

        return RobotCommand(
            vx=vx_axis * self.max_vx,
            vy=vy_axis * self.max_vy,
            yaw_rate=yaw_axis * self.max_yaw_rate,
            active=active,
            estop=self.estop,
        )

    def _handle_button_down(self, button: int) -> None:
        if button == self.BUTTON_A:
            self.estop = False
            self.active_requested = True
        elif button == self.BUTTON_B:
            self.active_requested = False
        elif button == self.BUTTON_START:
            if not self.estop:
                self.active_requested = not self.active_requested
        elif button in (self.BUTTON_X, self.BUTTON_BACK):
            self.estop = True
            self.active_requested = False
        elif button == self.BUTTON_Y:
            self.estop = False
            self.active_requested = False


def format_command_file(command: RobotCommand) -> str:
    return (
        f"vx={command.vx:.6f}\n"
        f"vy={command.vy:.6f}\n"
        f"yaw_rate={command.yaw_rate:.6f}\n"
        f"active={'true' if command.active else 'false'}\n"
        f"estop={'true' if command.estop else 'false'}\n"
    )


def atomic_write(path: str, text: str) -> None:
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    tmp_path = f"{path}.tmp.{os.getpid()}"
    with open(tmp_path, "w", encoding="utf-8") as file:
        file.write(text)
        file.flush()
        os.fsync(file.fileno())
    os.replace(tmp_path, path)
