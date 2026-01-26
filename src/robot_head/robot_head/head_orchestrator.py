#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HeadOrchestrator(Node):
    def __init__(self):
        super().__init__('head_orchestrator')

        # Stato interno
        self.current_state = "idle"

        # Subscriber: comandi di alto livello
        self.cmd_sub = self.create_subscription(
            String,
            '/head/cmd',
            self.cmd_callback,
            10
        )

        # Publisher: stato attuale
        self.state_pub = self.create_publisher(
            String,
            '/head/state',
            10
        )

        # Timer heartbeat
        self.timer = self.create_timer(2.0, self.publish_state)

        self.get_logger().info("Head Orchestrator avviato (state=idle)")

    def cmd_callback(self, msg: String):
        cmd = msg.data.strip().lower()

        valid_states = ['idle', 'observe', 'listen', 'speak', 'demo']

        if cmd not in valid_states:
            self.get_logger().warn(f"Comando non valido: {cmd}")
            return

        self.current_state = cmd
        self.get_logger().info(f"Nuovo stato impostato: {self.current_state}")

    def publish_state(self):
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeadOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
