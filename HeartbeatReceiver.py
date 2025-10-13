#!/usr/bin/env python3
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import threading


class HeartbeatReceiver(Node):
    def __init__(self, udp_ip="0.0.0.0", udp_port=8889):
        super().__init__('heartbeat_receiver')

        # ROS2 publisher
        self.publisher_ = self.create_publisher(Bool, 'speaker_playing', 10)

        # UDP socket (blocking mode)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((udp_ip, udp_port))
        self.get_logger().info(f"🎧 Listening for heartbeat on {udp_ip}:{udp_port}")

        # Track last state (default False)
        self.last_state = False

        # Run UDP listener in a separate thread
        self._running = True
        self.thread = threading.Thread(target=self._udp_loop, daemon=True)
        self.thread.start()

        # Publish at 5 Hz
        self.timer = self.create_timer(0.1, self._publish_state)

    def _udp_loop(self):
        """Listen for UDP packets and update state."""
        while self._running:
            try:
                data, addr = self.sock.recvfrom(1024)  # blocking
                if data == b"\x01":
                    self.last_state = True
                elif data == b"\x00":
                    self.last_state = False
                else:
                    self.get_logger().warn(f"⚠️ Unknown signal {data} from {addr}")
            except Exception as e:
                self.get_logger().error(f"UDP error: {e}")

    def _publish_state(self):
        """Publish the current state at fixed 5 Hz."""
        msg = Bool()
        msg.data = self.last_state
        self.publisher_.publish(msg)

    def destroy_node(self):
        self._running = False
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

