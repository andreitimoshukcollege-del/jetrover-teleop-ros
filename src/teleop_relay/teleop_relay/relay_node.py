"""Bridges Teleop.RobotHost (a .NET process, the real IRobotPlant/ITransport endpoint -- see
teleoperation's docs/adr/0007-jetrover-plant-and-robot-host.md) to jetrover_arm_control's
existing ROS topics, over a local Unix domain socket (relay_protocol.py). Deliberately thin: no
staleness/sequencing/gap-policy logic here at all -- that's JetRoverPlant's job, upstream of this
socket. This node only ever translates "here are the current direction/gripper values" into the
ROS topic calls jetrover_arm_control already exposes, and relays servo feedback back the same way.

Phase 2 scope: base/lower/middle/upper joints (relative "direction" commands, matching
ServoController.setPos) plus the gripper (absolute degrees, matching setGripperPos). Gripper
feedback is not relayed -- Teleop.RobotHost's JetRoverPlant sends the gripper open-loop, with no
delta tracking that would need it.
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

        self._base_pub = self.create_publisher(Float32, "/arm/servo/base", 1)
        self._lower_pub = self.create_publisher(Float32, "/arm/servo/joint/lower", 1)
        self._middle_pub = self.create_publisher(Float32, "/arm/servo/joint/middle", 1)
        self._upper_pub = self.create_publisher(Float32, "/arm/servo/joint/upper", 1)
        self._gripper_main_pub = self.create_publisher(Float32, "/arm/servo/gripper/main", 1)

        self.create_subscription(Int32, "/unity/robot/servo/base", self._on_base_feedback, 1)
        self.create_subscription(Int32, "/unity/robot/servo/joint/lower", self._on_lower_feedback, 1)
        self.create_subscription(Int32, "/unity/robot/servo/joint/middle", self._on_middle_feedback, 1)
        self.create_subscription(Int32, "/unity/robot/servo/joint/upper", self._on_upper_feedback, 1)

        self._last_base_degrees = None
        self._last_lower_degrees = None
        self._last_middle_degrees = None
        self._last_upper_degrees = None

        self.create_timer(_COMMAND_POLL_PERIOD_SECONDS, self._poll_local_socket)
        self.create_timer(_FEEDBACK_SEND_PERIOD_SECONDS, self._send_feedback)

        self.get_logger().info(
            f"Relay listening on {LOCAL_RELAY_SOCKET}, forwarding feedback to {HOST_SOCKET}"
        )

    def _on_base_feedback(self, msg: Int32) -> None:
        self._last_base_degrees = msg.data

    def _on_lower_feedback(self, msg: Int32) -> None:
        self._last_lower_degrees = msg.data

    def _on_middle_feedback(self, msg: Int32) -> None:
        self._last_middle_degrees = msg.data

    def _on_upper_feedback(self, msg: Int32) -> None:
        self._last_upper_degrees = msg.data

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

        decoded = relay_protocol.decode_arm_command(latest)
        if decoded is None:
            return
        base_direction, lower_direction, middle_direction, upper_direction, gripper_degrees = decoded

        self._publish_float(self._base_pub, base_direction)
        self._publish_float(self._lower_pub, lower_direction)
        self._publish_float(self._middle_pub, middle_direction)
        self._publish_float(self._upper_pub, upper_direction)
        self._publish_float(self._gripper_main_pub, gripper_degrees)

    @staticmethod
    def _publish_float(publisher, value: float) -> None:
        msg = Float32()
        msg.data = value
        publisher.publish(msg)

    def _send_feedback(self) -> None:
        payload = relay_protocol.encode_feedback(
            base_valid=self._last_base_degrees is not None, base_degrees=self._last_base_degrees or 0,
            lower_valid=self._last_lower_degrees is not None, lower_degrees=self._last_lower_degrees or 0,
            middle_valid=self._last_middle_degrees is not None, middle_degrees=self._last_middle_degrees or 0,
            upper_valid=self._last_upper_degrees is not None, upper_degrees=self._last_upper_degrees or 0,
        )
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
