"""Vendor-neutral servo telemetry.

The only code in robot_arm_control that knows the servo state message types.
DynamixelState and FeetechState carry the same per-servo fields with the same
names and units; they differ only in type name, topic, and the name of the
error field (dxl_hw_state vs hw_state). ServoTelemetry subscribes to whichever
one the arm profile selects and exposes one ServoReading per joint.

ServoReading uses the message's field names and units unchanged:
  present_temperature    degC
  present_input_voltage  0.1 V  (120 = 12.0 V)
  present_current        mA, sign = direction
  present_load           0.1 %, sign = direction
A field is None when the profile says the joint's servo does not provide it.

The message type is imported at runtime, so only the selected backend's
interface package needs to be installed.
"""

from dataclasses import dataclass
from typing import Callable, Dict, Optional

from rosidl_runtime_py.utilities import get_message

from robot_arm_control.arm_profile import ArmProfile, TELEMETRY_FIELDS

# backend -> (message type, name of the per-servo error field)
_BACKENDS = {
    'dynamixel': ('dynamixel_interfaces/msg/DynamixelState', 'dxl_hw_state'),
    'feetech': ('feetech_interfaces/msg/FeetechState', 'hw_state'),
}


@dataclass(frozen=True)
class ServoReading:
    joint: str
    id: int
    torque_state: bool
    hw_state: int                           # dxl_hw_state / hw_state; 0 = no error, -1 = not read
    present_temperature: Optional[int]
    present_input_voltage: Optional[int]
    present_current: Optional[int]
    present_load: Optional[int]


class ServoTelemetry:
    def __init__(self, node, profile: ArmProfile,
                 on_update: Optional[Callable[[Dict[str, ServoReading]], None]] = None,
                 qos_depth: int = 10):
        msg_type_name, self._hw_state_field = _BACKENDS[profile.telemetry_backend]
        try:
            msg_type = get_message(msg_type_name)
        except (ModuleNotFoundError, AttributeError, ValueError) as e:
            raise RuntimeError(
                f"arm profile '{profile.name}' uses telemetry backend '{profile.telemetry_backend}', "
                f"but {msg_type_name} could not be imported ({e}). Is its interface package built and sourced?")

        self._profile = profile
        self._joint_by_id = profile.joint_by_id
        self._on_update = on_update
        self.readings: Dict[str, ServoReading] = {}   # joint name -> latest reading
        self.comm_state: Optional[int] = None
        self.message_count = 0

        self._sub = node.create_subscription(msg_type, profile.telemetry_topic, self._callback, qos_depth)
        node.get_logger().info(
            f"Servo telemetry: {msg_type_name} on '{profile.telemetry_topic}' (profile '{profile.name}')")

    @property
    def has_data(self) -> bool:
        return self.message_count > 0

    def _callback(self, msg):
        hw_states = getattr(msg, self._hw_state_field)

        for i, servo_id in enumerate(msg.id):
            spec = self._joint_by_id.get(servo_id)
            if spec is None:
                continue
            fields = {}
            for name in TELEMETRY_FIELDS:
                values = getattr(msg, name)
                fields[name] = int(values[i]) if name in spec.reports and i < len(values) else None
            self.readings[spec.name] = ServoReading(
                joint=spec.name,
                id=servo_id,
                torque_state=bool(msg.torque_state[i]) if i < len(msg.torque_state) else False,
                hw_state=int(hw_states[i]) if i < len(hw_states) else -1,
                **fields,
            )
        self.comm_state = msg.comm_state
        self.message_count += 1
        if self._on_update is not None:
            self._on_update(self.readings)
