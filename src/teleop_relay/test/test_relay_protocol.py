"""Mirrors teleoperation's RelayProtocolTests.cs -- same wire format, verified from both sides."""

import struct

from teleop_relay import relay_protocol


def test_decode_arm_command_round_trips_with_the_dotnet_encoder_layout():
    # version=1, base_direction=-3.25, little-endian -- matches RelayProtocol.EncodeCommand's
    # exact byte layout in the .NET project.
    data = struct.pack("<Bf", 1, -3.25)

    result = relay_protocol.decode_arm_command(data)

    assert result == -3.25


def test_decode_arm_command_rejects_wrong_version():
    data = struct.pack("<Bf", 2, 1.0)

    assert relay_protocol.decode_arm_command(data) is None


def test_decode_arm_command_rejects_too_short_buffer():
    assert relay_protocol.decode_arm_command(b"\x01\x00\x00") is None


def test_encode_feedback_matches_dotnet_layout():
    encoded = relay_protocol.encode_feedback(base_degrees_valid=True, base_degrees=-42)

    version, valid, base_degrees = struct.unpack("<BBi", encoded)
    assert version == relay_protocol.VERSION
    assert valid == 1
    assert base_degrees == -42
