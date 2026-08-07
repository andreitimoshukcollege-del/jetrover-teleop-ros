"""Bridges Teleop.RobotHost (a .NET process, the real IRobotPlant/ITransport endpoint -- see
teleoperation's docs/adr/0007-jetrover-plant-and-robot-host.md) to jetrover_arm_control's
existing ROS topics, over a local Unix domain socket (relay_protocol.py). Deliberately thin: no
staleness/sequencing/gap-policy logic here at all -- that's JetRoverPlant's job, upstream of this
socket. This node only ever translates "here is the current direction value" into the one ROS
topic call jetrover_arm_control already exposes, and relays servo feedback back the same way.

Phase 1 scope only: the base servo alone, matching what's already been manually verified against
the real hardware.
"""

import os
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32

from teleop_relay import relay_protocol

LOCAL_RELAY_SOCKET = "/tmp/jetrover_relay.sock"
HOST_SOCKET = "/tmp/teleop_robot_host.sock"

_COMMAND_POLL_PERIOD_SECONDS = 0.02  # 50 Hz
_FEEDBACK_SEND_PERIOD_SECONDS = 0.05  # 20 Hz


class TeleopRelayNode(Node):
    def __init__(self):
        super().__init__("teleop_relay_node")

        # A stale socket file left over from a crashed previous run must not block binding --
        # same reasoning as UdsRelayClient's own constructor on the .NET side.
        if os.path.exists(LOCAL_RELAY_SOCKET):
            os.remove(LOCAL_RELAY_SOCKET)

        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._socket.bind(LOCAL_RELAY_SOCKET)
        self._socket.setblocking(False)

        self._base_direction_pub = self.create_publisher(Float32, "/arm/servo/base", 1)
        self.create_subscription(Int32, "/unity/robot/servo/base", self._on_base_feedback, 1)

        self._last_base_degrees = None

        self.create_timer(_COMMAND_POLL_PERIOD_SECONDS, self._poll_local_socket)
        self.create_timer(_FEEDBACK_SEND_PERIOD_SECONDS, self._send_feedback)

        self.get_logger().info(
            f"Relay listening on {LOCAL_RELAY_SOCKET}, forwarding feedback to {HOST_SOCKET}"
        )

    def _on_base_feedback(self, msg: Int32) -> None:
        self._last_base_degrees = msg.data

    def _poll_local_socket(self) -> None:
        # Drain every pending datagram but act only on the most recent -- IRelayClient's
        # contract is "here is the current setpoint," not a queue of every one ever sent, so a
        # backlog here (this node briefly starved of CPU) must not replay stale commands once
        # it catches up.
        latest = None
        while True:
            try:
                latest, _ = self._socket.recvfrom(relay_protocol.ARM_COMMAND_SIZE)
            except BlockingIOError:
                break

        if latest is None:
            return

        base_direction = relay_protocol.decode_arm_command(latest)
        if base_direction is None:
            return

        msg = Float32()
        msg.data = base_direction
        self._base_direction_pub.publish(msg)

    def _send_feedback(self) -> None:
        valid = self._last_base_degrees is not None
        base_degrees = self._last_base_degrees if valid else 0
        payload = relay_protocol.encode_feedback(valid, base_degrees)
        try:
            self._socket.sendto(payload, HOST_SOCKET)
        except OSError:
            # Teleop.RobotHost isn't up yet, or has restarted -- fire-and-forget, matching
            # UdsRelayClient.Send's own tolerance for the mirror-image case.
            pass


def main():
    rclpy.init()
    node = TeleopRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
