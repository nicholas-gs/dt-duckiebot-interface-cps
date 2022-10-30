#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from dt_interfaces_cps.msg import WheelsCmdStamped, BoolStamped


class WheelsDriverTestNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.pub_e_stop = self.create_publisher(
            BoolStamped,
            "~/emergency_stop",
            1)
        self.pub_wheels_cmd = self.create_publisher(
            WheelsCmdStamped,
            "~/wheels_cmd",
            1)

        self.wheels_cmd_timer = self.create_timer(3.0, self.send_wheels_cmd)

        self.get_logger().info("Initialized")

    def send_wheels_cmd(self):
        msg = WheelsCmdStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ngsduckie_motor'
        msg.vel_left = 0.0
        msg.vel_right = 0.5
        self.pub_wheels_cmd.publish(msg)

    def cleanup(self):
        self.wheels_cmd_timer.destroy()
        msg = WheelsCmdStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vel_left = 0.0
        msg.vel_right = 0.0
        self.pub_wheels_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelsDriverTestNode("wheels_driver_test_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
