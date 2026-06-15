#!/usr/bin/env python3

# Subscribes to /odom and /planned_path and publishes to /pp_plan_path, /pp_plan_lookahead and /cmd_vel 

import math
import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# Defines the ROS2 node class 
class PurePursuitPlanFollower(Node):
    def __init__(self):
        super().__init__("pure_pursuit_plan_follower") # Names the node pure_pursuit_plan_follower

        # Helps prevent controller from jumping back to earlier path points.
        self.path = [] # Empty till /planned_path becomes active.
        self.closest_index = 0 # Tracks progress along path.

        # Robot chases a point 0.45 m ahead on the planned path.
        self.lookahead_distance = 0.45

        # Speed limiter settings
        self.max_linear_speed = 0.25
        self.min_linear_speed = 0.08
        self.max_angular_speed = 1.0

        # Adaptive speed tuning
        # Higher value = slows down more aggressively on curves
        self.curvature_speed_gain = 0.8

        # Goal behavior
        # Controller stops when the robot is within 15 cm of the goal.
        self.goal_tolerance = 0.15
        # Reduce speed when within 0.6 m of the final goal. 
        self.slowdown_distance = 0.60
        # Caps speed to 0.1 m/s when near the goal.
        self.goal_linear_speed = 0.10

        # Robot State Parameters.
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Frame setup.
        # /planned_path is in map frame, so robot pose must also be read in map frame.
        self.path_frame = "map"
        self.robot_frame = "base_footprint"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_received = False # Becomes true after /odom is received.
        self.path_received = False # Becomes true after /planned_path is received.
        self.finished = False # Becomes true after goal is reached.
        
        # CSV logging
        self.log_dir = os.path.expanduser("~/self_drive_ws/logs/controller_logs") # Creates a log directory if it does not exist
        os.makedirs(self.log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y_%m_%d_%H%M%S") # Creates a time stamped log file name
        self.log_path = os.path.join( 
            self.log_dir,
            f"pure_pursuit_log_{timestamp}.csv"
        )
        self.log_file = open(self.log_path, "w", newline="") # Opens the CSV file for writing 
        self.csv_writer = csv.writer(self.log_file) 
        self.csv_writer.writerow([ # Writes the header row
            "time_sec",
            "controller",
            "x",
            "y",
            "yaw",
            "lookahead_x",
            "lookahead_y",
            "cross_track_error",
            "goal_distance",
            "curvature",
            "cmd_linear_x",
            "cmd_angular_z"
        ])
        self.get_logger().info(f"Logging Pure Pursuit data to: {self.log_path}")
        # Subscribes to odometry 
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )
        # Subscribes to the nav2 path generated and published by nav2_plan_client.py
        self.path_sub = self.create_subscription(
            Path,
            "/planned_path",
            self.path_callback,
            10
        )

        # Publishes velocity commands to the robot
        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # Publishes the planned path as an Rviz marker
        self.path_marker_pub = self.create_publisher(
            Marker,
            "/pp_plan_path",
            10
        )

        # Publishes the current lookahead point as an RViz marker.
        self.lookahead_marker_pub = self.create_publisher(
            Marker,
            "/pp_plan_lookahead",
            10
        )
        # Runs control at 20 Hz 
        self.timer = self.create_timer(0.05, self.control_loop)
        # Print Statements
        self.get_logger().info("Pure Pursuit /plan follower started.")
        self.get_logger().info("Waiting for /planned_path and /odom...")

    # Writes one row of controller data to the CSV file.    
    def log_data(
        self,
        lookahead_point,
        cross_track_error,
        final_distance,
        curvature,
        cmd
    ):
        # Gets ROS time in seconds
        time_sec = self.get_clock().now().nanoseconds / 1e9

        self.csv_writer.writerow([
            f"{time_sec:.4f}",
            "pure_pursuit",
            f"{self.x:.4f}",
            f"{self.y:.4f}",
            f"{self.yaw:.4f}",
            f"{lookahead_point[0]:.4f}",
            f"{lookahead_point[1]:.4f}",
            f"{cross_track_error:.4f}",
            f"{final_distance:.4f}",
            f"{curvature:.4f}",
            f"{cmd.linear.x:.4f}",
            f"{cmd.angular.z:.4f}"
        ])
        # Forces data to be written to disk every control cycle.
        self.log_file.flush()

    def odom_callback(self, msg):
        # This function reads robot position and converts quaternion to yaw.
        # self.x = msg.pose.pose.position.x
        # self.y = msg.pose.pose.position.y

        # q = msg.pose.pose.orientation

        # siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        # cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        # self.yaw = math.atan2(siny_cosp, cosy_cosp)

        # Marks odom as received or available.
        self.odom_received = True
    
    def update_robot_pose_from_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.path_frame,      # target frame: map
                self.robot_frame,     # source frame: base_footprint
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().warn(
                f"Could not get transform {self.path_frame} -> {self.robot_frame}: {ex}",
                throttle_duration_sec=1.0
            )
            return False

        t = transform.transform.translation
        q = transform.transform.rotation

        self.x = t.x
        self.y = t.y

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
    
        return True

    def path_callback(self, msg):
        # If an empty path arrives, ignore it. 
        if len(msg.poses) == 0:
            return

        # Clears the previous path.
        self.path = []

        # Converts nav_msgs/Path into a Python list (x,y) tuples.
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.path.append((x, y))

        # Restarts path tracking from the beginning.
        self.closest_index = 0
        # Marks the path as received.
        self.path_received = True
        # Clears the finished flag.
        self.finished = False
        
        self.get_logger().info(f"Received new /planned_path with {len(self.path)} points.")

    def clamp(self, value, min_value, max_value):
        # Clamps the value between a minium and maximum.
        return max(min(value, max_value), min_value)

    def distance(self, p1, p2):
        # Computes Euclidean distance between 2 points. 
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return math.sqrt(dx * dx + dy * dy)
    
    def compute_cross_track_error(self):
    #"""Cross-track error is the distance from the robot to the nearest point on the planned path.Smaller is better."""
        if not self.path:
            return 0.0

        robot_pos = (self.x, self.y)
        min_dist = float("inf")

        for point in self.path:
            d = self.distance(robot_pos, point)
            if d < min_dist:
                min_dist = d

        return min_dist

    def find_closest_index(self):
        # Searches forward from the current closest index and finds the nearest path point.
        robot_pos = (self.x, self.y)

        best_index = self.closest_index
        best_dist = float("inf")

        for i in range(self.closest_index, len(self.path)):
            d = self.distance(robot_pos, self.path[i])
            if d < best_dist:
                best_dist = d
                best_index = i

        self.closest_index = best_index
        return best_index

    def find_lookahead_point(self):
        # Finds the first path point ahead whose distance from the robot is at least the lookahead distance.
        robot_pos = (self.x, self.y)
        closest = self.find_closest_index()

        for i in range(closest, len(self.path)):
            d = self.distance(robot_pos, self.path[i])
            if d >= self.lookahead_distance:
                return self.path[i], i

        return self.path[-1], len(self.path) - 1

    def transform_to_robot_frame(self, point):
        # Transforms the lookahead point from map/world coordinates into the robot coordinate frame.
        dx = point[0] - self.x
        dy = point[1] - self.y

        x_robot = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy
        y_robot = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy

        return x_robot, y_robot

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def publish_path_marker(self):
        if not self.path:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pp_plan_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.color.a = 1.0
        marker.color.g = 1.0

        for x, y in self.path:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.05
            marker.points.append(p)

        self.path_marker_pub.publish(marker)

    def publish_lookahead_marker(self, point):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pp_plan_lookahead"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.10
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2

        self.lookahead_marker_pub.publish(marker)

    def control_loop(self):
        self.publish_path_marker()

        #if not self.odom_received or not self.path_received:
        #    return
        if not self.path_received:
            return

        # Update robot pose in the same frame as /planned_path: map.
        if not self.update_robot_pose_from_tf():
            return

        if self.finished:
            self.publish_stop()
            return

        final_goal = self.path[-1]
        final_distance = self.distance((self.x, self.y), final_goal)

        if final_distance < self.goal_tolerance and self.closest_index >= len(self.path) - 2:
            self.finished = True
            self.publish_stop()
            self.get_logger().info("Finished following /plan.")
            return

        lookahead_point, lookahead_index = self.find_lookahead_point()
        self.publish_lookahead_marker(lookahead_point)

        x_robot, y_robot = self.transform_to_robot_frame(lookahead_point)

        cmd = Twist()

        if x_robot <= 0.05:
            cmd.linear.x = 0.0
            cmd.angular.z = self.max_angular_speed if y_robot > 0.0 else -self.max_angular_speed
            self.cmd_pub.publish(cmd)
            return

        curvature = 2.0 * y_robot / (self.lookahead_distance * self.lookahead_distance)
        
        # Adaptive speed:
        # When curvature is small, go faster.
        # When curvature is large, slow down.
        speed = self.max_linear_speed / (
            1.0 + self.curvature_speed_gain * abs(curvature)
        )
        
        speed = self.clamp(
            speed,
            self.min_linear_speed,
            self.max_linear_speed
        )
        
        # Slow down near the final goal
        if final_distance < self.slowdown_distance:
            speed = min(speed, self.goal_linear_speed)
        
        cmd.linear.x = speed
        cmd.angular.z = cmd.linear.x * curvature
        
        cmd.angular.z = self.clamp(
            cmd.angular.z,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        self.cmd_pub.publish(cmd)
        cross_track_error = self.compute_cross_track_error()
        self.log_data(
            lookahead_point,
            cross_track_error,
            final_distance,
            curvature,
            cmd
        )
        self.get_logger().info(
            "\n"
            "================ PURE PURSUIT DEBUG ================\n"
            f"{'Path Index':<18}: {lookahead_index}/{len(self.path)}\n"
            f"{'Robot Pose':<18}: x={self.x:>8.2f}, y={self.y:>8.2f}, yaw={self.yaw:>8.2f}\n"
            f"{'Lookahead Point':<18}: x={lookahead_point[0]:>8.2f}, y={lookahead_point[1]:>8.2f}\n"
            f"{'Robot Frame Point':<18}: x={x_robot:>8.2f}, y={y_robot:>8.2f}\n"
            f"{'Cross Track Error':<18}: {cross_track_error:>8.3f} m\n"
            f"{'Goal Distance':<18}: {final_distance:>8.2f} m\n"
            f"{'Curvature':<18}: {curvature:>8.2f}\n"
            f"{'Command':<18}: linear.x={cmd.linear.x:>8.2f}, angular.z={cmd.angular.z:>8.2f}\n"
            "====================================================\n\n",
            throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitPlanFollower()

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