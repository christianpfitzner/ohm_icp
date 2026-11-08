#!/usr/bin/env python3
"""The driver of the lab (given): the robot drives the loop of ohm_icp/route.py, lap after lap.

The driver steers by the exact pose on /<robot>/truth, like a person who sees where the robot is. Your node
never reads /truth: it has to find the robot from its scans (and its odometry) alone.

    ros2 run ohm_icp route_node.py --ros-args -p speed:=0.5      (the launch file starts it)
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import rclpy                                                  # noqa: E402
from geometry_msgs.msg import PoseStamped, Twist              # noqa: E402
from rclpy.executors import ExternalShutdownException         # noqa: E402
from rclpy.node import Node                                   # noqa: E402

from ohm_icp import route                                     # noqa: E402


class RouteNode(Node):

    def __init__(self):
        super().__init__("route")
        robot = self.declare_parameter("robot", "muster").value
        self.driver = route.Driver(speed=self.declare_parameter("speed", 0.5).value,      # m/s along the loop
                                   turn=self.declare_parameter("turn", 1.2).value)        # rad/s at most
        self.pub = self.create_publisher(Twist, f"/{robot}/cmd_vel", 10)
        self.create_subscription(PoseStamped, f"/{robot}/truth", self.on_truth, 10)
        self.get_logger().info(f"driving the loop ({route.length():.1f} m, {self.driver.speed} m/s) by /{robot}/truth")

    def on_truth(self, msg: PoseStamped) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = msg.pose.orientation
        vx, vy, omega = self.driver.step(t, msg.pose.position.x, msg.pose.position.y,
                                         math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z))
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, omega
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = RouteNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:                          # Ctrl-C in the middle of a callback: ROS is already gone
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
