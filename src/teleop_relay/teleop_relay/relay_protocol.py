"""Fixed-size, versioned, little-endian wire format for the local relay channel.

Must match core/Teleop.RobotHost/Relay/RelayProtocol.cs in the `teleoperation` repo exactly --
this is the one wire format shared between the two languages, kept deliberately tiny (no
sequence numbers, no staleness/coast logic) because it only ever crosses a local Unix domain
socket on one machine. See that file's own doc comment for the full reasoning.
"""

import struct

VERSION = 1

_ARM_COMMAND_FORMAT = "<Bf"  # version, base_direction
ARM_COMMAND_SIZE = struct.calcsize(_ARM_COMMAND_FORMAT)

_FEEDBACK_FORMAT = "<BBi"  # version, base_degrees_valid, base_degrees
FEEDBACK_SIZE = struct.calcsize(_FEEDBACK_FORMAT)


def decode_arm_command(data: bytes):
    """Returns `base_direction` (float), or None if `data` isn't a well-formed, version-matching
    LocalArmCommand -- mirrors RelayProtocol.TryDecodeCommand's "reject, don't throw" contract.
    """
    if len(data) < ARM_COMMAND_SIZE:
        return None
    version, base_direction = struct.unpack_from(_ARM_COMMAND_FORMAT, data, 0)
    if version != VERSION:
        return None
    return base_direction


def encode_feedback(base_degrees_valid: bool, base_degrees: int) -> bytes:
    return struct.pack(_FEEDBACK_FORMAT, VERSION, 1 if base_degrees_valid else 0, int(base_degrees))
