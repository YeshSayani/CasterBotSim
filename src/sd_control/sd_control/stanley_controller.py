#!/usr/bin/env python3

# This controller follows a Nav2-generated path from: /planned_path.
# Subscribes to /odom, /planned_path
# Publishes to /cmd_vel, /stanley_path, /stanley_closest_point.
# Stanley Controller basically says:
    # Find the closest point on the path.
    # Match the path heading.
    # Correct lateral cross-track error.
    # Basically, angular_cmd = heading correction + cross-track correction.

import math
import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point # Marker Points.

# Defines the stanley controller node.
class StanleyController(Node):
    def __init__(self):
        # Names the node Stanley Controller.
        super().__init__("stanley_controller")

        self.path = [] # Stores the path from /planned_path.
        self.closest_index = 0 # Stores the closest index, tracks progress along the path.

        # Controller tuning, main stanley gains.
        # Weights lateral error, higher value is more aggressive towards center line.  
        self.k_cross_track = 1.2 
        # Weights Heading error, higher value is more aggressive towards path direction.  
        self.k_heading = 1.5

        # Speed tuning and limits
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

        # Sets these flags to false intially, so the controller does nothing until /odom and /planned_path have been received.
        self.odom_received = False
        self.path_received = False
        self.finished = False

        # CSV logging
        # Creates the log directory if needed.
        self.log_dir = os.path.expanduser("~/self_drive_ws/logs/controller_logs")
        os.makedirs(self.log_dir, exist_ok=True)
        # Creates a timestamped log file such as: stanley_log_2026_06_14_153020.csv
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

        # Reads robot pose from /odom
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        # Reads the Nav2 path from /planned_path
        self.path_sub = self.create_subscription(
            Path,
            "/planned_path",
            self.path_callback,
            10
        )

        # Publishes velocity commands to the robot.
        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )
        # Publisher for the path marker for Rviz.
        self.path_marker_pub = self.create_publisher(
            Marker,
            "/stanley_path",
            10
        )

        # Publishes a sphere marker at the closest point.
        self.closest_marker_pub = self.create_publisher(
            Marker,
            "/stanley_closest_point",
            10
        )

        # Runs the controller at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Stanley controller started.")
        self.get_logger().info("Waiting for /planned_path and /odom...")

    def odom_callback(self, msg):
        # Reads Robot position from /odom, converts quaternions into Yaw.
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.odom_received = True

    def path_callback(self, msg):
        if len(msg.poses) < 2:
            # Stanley needs atleast two points as it computes path heading using point i and i + 1.
            self.get_logger().warn("Received path with fewer than 2 poses. Ignoring.")
            return

        self.path = []
        # Converts the ROS path into a list of (x,y) points. 
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.path.append((x, y))

        # Resets tracking when a new path arrives.
        self.closest_index = 0
        self.path_received = True
        self.finished = False

        self.get_logger().info(f"Received new /planned_path with {len(self.path)} points.")

    def normalize_angle(self, angle):
        # Keeps angles between -pi and pi.
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def clamp(self, value, min_value, max_value):
        # Clamps the values between a minimum and maximum.
        return max(min(value, max_value), min_value)

    def distance(self, p1, p2):
        # Computes Euclidean distance between two points.
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return math.sqrt(dx * dx + dy * dy)

    def publish_stop(self):
        # Publishes stop commands or Zero velocity commands. 
        self.cmd_pub.publish(Twist())

    def find_closest_index(self):
        # Finds the closest path point to the robot.
        # Starts at self.closest_index. 
        robot_pos = (self.x, self.y)
        best_index = self.closest_index
        best_dist = float("inf")

        for i in range(self.closest_index, len(self.path)):
            d = self.distance(robot_pos, self.path[i])
            if d < best_dist:
                best_dist = d
                best_index = i

        self.closest_index = best_index
        # It returns: closest_index and absolute distance to closest point
        return best_index, best_dist

    def get_path_heading(self, index):
        # This computes the direction of the path segment at the closest point.
        # It uses the vector: path[index] -> path[index + 1]
        if index >= len(self.path) - 1: # Prevents indexing past the end.
            index = len(self.path) - 2

        x1, y1 = self.path[index]
        x2, y2 = self.path[index + 1]

        return math.atan2(y2 - y1, x2 - x1)

    def compute_signed_cross_track_error(self, index):
        """
        Signed cross-track error:
        positive if robot is to the left of the path direction,
        negative if robot is to the right of the path direction.
        Stanley does not just need “distance from path”; it needs to know which direction to steer.
        """
        if index >= len(self.path) - 1:
            index = len(self.path) - 2

        # This creates the vector from path point to robot.
        path_x, path_y = self.path[index]
        path_yaw = self.get_path_heading(index)

        dx = self.x - path_x
        dy = self.y - path_y

        # This rotates the robot-position error into the path frame.
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
        # Continuously publishes the path marker.
        self.publish_path_marker()

        # Waits until both odometry and path are ready.
        if not self.odom_received or not self.path_received:
            return

        # If path is complete, keep publishing stop messages.
        if self.finished:
            self.publish_stop()
            return

        # Stops if the robot is near the final path point and has progressed to the end of the path.
        final_goal = self.path[-1]
        final_distance = self.distance((self.x, self.y), final_goal)

        if final_distance < self.goal_tolerance and self.closest_index >= len(self.path) - 2:
            self.finished = True
            self.publish_stop()
            self.get_logger().info("Stanley path complete.")
            return
        
        # Finds and visualizes the closest path point.
        closest_index, cross_track_abs = self.find_closest_index()
        closest_point = self.path[closest_index]
        self.publish_closest_marker(closest_point)

        # These are the two core Stanley errors:
        # heading_error: How much the robot heading differs from path heading.
        # cross_track_error: How far left/right the robot is from the path.
        path_heading = self.get_path_heading(closest_index)
        heading_error = self.normalize_angle(path_heading - self.yaw)
        cross_track_error = self.compute_signed_cross_track_error(closest_index)

        # Regulates the speed near the final goal.
        speed = self.max_linear_speed
        if final_distance < self.slowdown_distance:
            speed = self.goal_linear_speed
        
        # This is the Stanley cross-track correction. The denominator uses speed.
        # At low speed, the same cross-track error should produce stronger steering correction.
        # At higher speed, correction is smoother.
        # The + 0.05 prevents division by zero or extremely large corrections when speed is near zero.
        cte_term = math.atan2(
            self.k_cross_track * cross_track_error,
            speed + 0.05
        )

        # It combines: heading alignment + lateral path correction, limits it to ±1.0 rad/s.
        angular_cmd = self.k_heading * heading_error + cte_term

        cmd = Twist()
        # Sets forward speed.
        cmd.linear.x = self.clamp(
            speed,
            self.min_linear_speed,
            self.max_linear_speed
        )

        # Sets forward speed.
        cmd.angular.z = self.clamp(
            angular_cmd,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        # Publishes cmd with linear and angular speed.
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
            "\n"
            "================ Stanley Control DEBUG ================\n"
            f"{'Path Index':<18}: {closest_index}/{len(self.path)}\n"
            f"{'Robot Pose':<18}: x={self.x:>8.2f}, y={self.y:>8.2f}, yaw={self.yaw:>8.2f}\n" 
            f"{'Path Yaw':<18}: {path_heading:>8.2f}\n"
            f"{'Heading Error':<18}: {heading_error:>8.2f}\n"
            f"{'Cross Track Error':<18}: {cross_track_error:>8.3f}\n"
            f"{'Goal Distance':<18}: {final_distance:>8.2f}\n"
            f"{'Command':<18}: Linear.x:{cmd.linear.x:>8.2f}, Angular.z:{cmd.angular.z:>8.2f}\n"
            "====================================================\n\n",
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args) # Starts the node.
    node = StanleyController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Publish stop messages on shutdown.
    node.publish_stop()

    # Close the csv file
    if hasattr(node, "log_file"):
        node.log_file.close()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()