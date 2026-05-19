#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class WaypointFollower(Node):
    def __init__(self):
        super().__init__("waypoint_follower_controller")

        # Waypoints in odom frame
        self.waypoints = [
            (-1.0, 0.0),
            (-1.0, -1.0),
            (0.0, -1.0),
            (0.0, 0.0),
        ]

        self.current_waypoint_index = 0

        # Controller gains
        self.k_linear = 0.5
        self.k_angular = 1.8

        # Speed limits
        self.max_linear_speed = 0.25
        self.max_angular_speed = 0.8

        # Tolerances
        self.distance_tolerance = 0.10
        self.heading_tolerance = 0.15

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False
        self.finished = False

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Waypoint follower started.")
        self.get_logger().info(f"Waypoints: {self.waypoints}")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.odom_received = True

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        if not self.odom_received:
            return

        if self.finished:
            self.publish_stop()
            return

        goal_x, goal_y = self.waypoints[self.current_waypoint_index]

        dx = goal_x - self.x
        dy = goal_y - self.y

        distance_error = math.sqrt(dx * dx + dy * dy)
        desired_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(desired_heading - self.yaw)

        if distance_error < self.distance_tolerance:
            self.get_logger().info(
                f"Reached waypoint {self.current_waypoint_index + 1}: "
                f"({goal_x:.2f}, {goal_y:.2f})"
            )

            self.current_waypoint_index += 1

            if self.current_waypoint_index >= len(self.waypoints):
                self.finished = True
                self.publish_stop()
                self.get_logger().info("All waypoints complete.")
                return

            return

        cmd = Twist()

        if abs(heading_error) > self.heading_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = self.k_angular * heading_error
        else:
            cmd.linear.x = self.k_linear * distance_error
            cmd.angular.z = self.k_angular * heading_error

        cmd.linear.x = self.clamp(
            cmd.linear.x,
            0.0,
            self.max_linear_speed
        )

        cmd.angular.z = self.clamp(
            cmd.angular.z,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"WP {self.current_waypoint_index + 1}/{len(self.waypoints)} | "
            f"pose=({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f}) | "
            f"goal=({goal_x:.2f}, {goal_y:.2f}) | "
            f"dist={distance_error:.2f}, heading_err={heading_error:.2f} | "
            f"cmd=({cmd.linear.x:.2f}, {cmd.angular.z:.2f})",
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.publish_stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()