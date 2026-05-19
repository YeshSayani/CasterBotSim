#!/usr/bin/env python3

import math
import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class StanleyController(Node):
    def __init__(self):
        super().__init__("stanley_controller")

        self.path = []
        self.closest_index = 0

        # Controller tuning
        self.k_cross_track = 1.2
        self.k_heading = 1.5

        # Speed tuning
        self.max_linear_speed = 0.22
        self.min_linear_speed = 0.08
        self.max_angular_speed = 1.0

        # Goal behavior
        self.goal_tolerance = 0.15
        self.slowdown_distance = 0.60
        self.goal_linear_speed = 0.10

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.odom_received = False
        self.path_received = False
        self.finished = False

        # CSV logging
        self.log_dir = os.path.expanduser("~/self_drive_ws/logs/controller_logs")
        os.makedirs(self.log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y_%m_%d_%H%M%S")
        self.log_path = os.path.join(
            self.log_dir,
            f"stanley_log_{timestamp}.csv"
        )

        self.log_file = open(self.log_path, "w", newline="")
        self.csv_writer = csv.writer(self.log_file)

        self.csv_writer.writerow([
            "time_sec",
            "controller",
            "x",
            "y",
            "yaw",
            "closest_x",
            "closest_y",
            "path_yaw",
            "heading_error",
            "cross_track_error",
            "goal_distance",
            "cmd_linear_x",
            "cmd_angular_z"
        ])

        self.get_logger().info(f"Logging Stanley data to: {self.log_path}")

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            "/planned_path",
            self.path_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.path_marker_pub = self.create_publisher(
            Marker,
            "/stanley_path",
            10
        )

        self.closest_marker_pub = self.create_publisher(
            Marker,
            "/stanley_closest_point",
            10
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Stanley controller started.")
        self.get_logger().info("Waiting for /planned_path and /odom...")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.odom_received = True

    def path_callback(self, msg):
        if len(msg.poses) < 2:
            self.get_logger().warn("Received path with fewer than 2 poses. Ignoring.")
            return

        self.path = []

        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.path.append((x, y))

        self.closest_index = 0
        self.path_received = True
        self.finished = False

        self.get_logger().info(f"Received new /planned_path with {len(self.path)} points.")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def distance(self, p1, p2):
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return math.sqrt(dx * dx + dy * dy)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def find_closest_index(self):
        robot_pos = (self.x, self.y)

        best_index = self.closest_index
        best_dist = float("inf")

        for i in range(self.closest_index, len(self.path)):
            d = self.distance(robot_pos, self.path[i])
            if d < best_dist:
                best_dist = d
                best_index = i

        self.closest_index = best_index
        return best_index, best_dist

    def get_path_heading(self, index):
        if index >= len(self.path) - 1:
            index = len(self.path) - 2

        x1, y1 = self.path[index]
        x2, y2 = self.path[index + 1]

        return math.atan2(y2 - y1, x2 - x1)

    def compute_signed_cross_track_error(self, index):
        """
        Signed cross-track error:
        positive if robot is to the left of the path direction,
        negative if robot is to the right of the path direction.
        """
        if index >= len(self.path) - 1:
            index = len(self.path) - 2

        path_x, path_y = self.path[index]
        path_yaw = self.get_path_heading(index)

        dx = self.x - path_x
        dy = self.y - path_y

        y_path = -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy

        return y_path

    def log_data(
        self,
        closest_point,
        path_heading,
        heading_error,
        cross_track_error,
        final_distance,
        cmd
    ):
        time_sec = self.get_clock().now().nanoseconds / 1e9

        self.csv_writer.writerow([
            f"{time_sec:.4f}",
            "stanley",
            f"{self.x:.4f}",
            f"{self.y:.4f}",
            f"{self.yaw:.4f}",
            f"{closest_point[0]:.4f}",
            f"{closest_point[1]:.4f}",
            f"{path_heading:.4f}",
            f"{heading_error:.4f}",
            f"{cross_track_error:.4f}",
            f"{final_distance:.4f}",
            f"{cmd.linear.x:.4f}",
            f"{cmd.angular.z:.4f}"
        ])

        self.log_file.flush()

    def publish_path_marker(self):
        if not self.path:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "stanley_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.color.a = 1.0
        marker.color.b = 1.0

        for x, y in self.path:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.06
            marker.points.append(p)

        self.path_marker_pub.publish(marker)

    def publish_closest_marker(self, point):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "stanley_closest_point"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.10
        marker.scale.x = 0.16
        marker.scale.y = 0.16
        marker.scale.z = 0.16
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.7

        self.closest_marker_pub.publish(marker)

    def control_loop(self):
        self.publish_path_marker()

        if not self.odom_received or not self.path_received:
            return

        if self.finished:
            self.publish_stop()
            return

        final_goal = self.path[-1]
        final_distance = self.distance((self.x, self.y), final_goal)

        if final_distance < self.goal_tolerance and self.closest_index >= len(self.path) - 2:
            self.finished = True
            self.publish_stop()
            self.get_logger().info("Stanley path complete.")
            return

        closest_index, cross_track_abs = self.find_closest_index()
        closest_point = self.path[closest_index]
        self.publish_closest_marker(closest_point)

        path_heading = self.get_path_heading(closest_index)
        heading_error = self.normalize_angle(path_heading - self.yaw)
        cross_track_error = self.compute_signed_cross_track_error(closest_index)

        speed = self.max_linear_speed
        if final_distance < self.slowdown_distance:
            speed = self.goal_linear_speed

        cte_term = math.atan2(
            self.k_cross_track * cross_track_error,
            speed + 0.05
        )

        angular_cmd = self.k_heading * heading_error + cte_term

        cmd = Twist()
        cmd.linear.x = self.clamp(
            speed,
            self.min_linear_speed,
            self.max_linear_speed
        )

        cmd.angular.z = self.clamp(
            angular_cmd,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        self.cmd_pub.publish(cmd)

        self.log_data(
            closest_point,
            path_heading,
            heading_error,
            cross_track_error,
            final_distance,
            cmd
        )

        self.get_logger().info(
            f"idx={closest_index}/{len(self.path)} | "
            f"pose=({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f}) | "
            f"path_yaw={path_heading:.2f} | "
            f"heading_err={heading_error:.2f} | "
            f"cte={cross_track_error:.3f} m | "
            f"goal_dist={final_distance:.2f} m | "
            f"cmd=({cmd.linear.x:.2f}, {cmd.angular.z:.2f})",
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.publish_stop()

    if hasattr(node, "log_file"):
        node.log_file.close()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()