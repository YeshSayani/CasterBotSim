#!/usr/bin/env python3
# Python Shebang, tells linux to execute the script with python3 if executed directly.

import math

import rclpy # ROS2 Python imports 
from rclpy.node import Node # Imports Node

from nav_msgs.msg import Odometry # Message type for /odom
from geometry_msgs.msg import Twist # Message type for /cmd_vel
from visualization_msgs.msg import Marker # Rviz Visualization
from geometry_msgs.msg import Point # Rviz Visualization


class PurePursuitController(Node): # Creates a ROS2 Node class.
    def __init__(self):
        super().__init__("pure_pursuit_controller") # Inherits from Node, so it can create subscribers, publishers, timers, and logs.

        # Path in odom frame.
        # This creates a smooth-ish loop around the map.
        # Hardcoded sparse list of waypoints. Pure pursuit will chase points from this list.
        self.path = [
            (0.0, 0.0),
            (0.5, 0.0),
            (0.9, 0.0),
            (1.2, -0.3),
            (1.2, -0.8),
            (0.8, -1.0),
            (0.3, -1.0),
        ]
        # Critical pure pursuit parameter, robot chases a point approximately 0.45 m ahead of it. 
        # Smaller lookahead: tighter tracking
        # more aggressive steering
        # more oscillation risk
    
        # Larger lookahead: smoother tracking
        # Less accurate around sharp turns
        self.lookahead_distance = 0.45

        # Controller uses a constant forward speed. 
        self.linear_speed = 0.22
        # Clamp on the maximum angular speed.
        self.max_angular_speed = 1.0

        # Tolerance - if the robot gets within 12 cm of the goal, it stops. 
        self.goal_tolerance = 0.12

        # Store the robot's current state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False
        self.finished = False

        # Set the initial closest index to 0. 
        # The controller only searches forward from the current closest index, preventing going backward along the same path.
        self.closest_index = 0

        # Odometry subscriber, subscribes to /odom.
        self.odom_sub = self.create_subscription(
            Odometry, # Message Type
            "/odom", # Topic to listen to
            self.odom_callback, # Odom callback function
            10 # Queue Size
        )

        # cmd_vel publisher
        self.cmd_pub = self.create_publisher(
            Twist, # Message type
            "/cmd_vel", # Topic to publish on
            10 # Queue Size
        )

        # Publisher for path marker (Visualization)
        self.path_marker_pub = self.create_publisher(
            Marker, # Message Type
            "/pure_pursuit_path", # Topic to publish on
            10 # Queue size
        )

        # Publisher for a sphere marker for the lookahead point.
        self.lookahead_marker_pub = self.create_publisher(
            Marker, # Message Type
            "/pure_pursuit_lookahead", # Topic to publish on
            10 # Queue size 
        )

        # Runs the control loop every 0.05 seconds.
        self.timer = self.create_timer(0.05, self.control_loop)

        # Print out statements for when the controller started and the lookahead distance
        self.get_logger().info("Pure Pursuit controller started.")
        self.get_logger().info(f"Lookahead distance: {self.lookahead_distance:.2f} m")

    def odom_callback(self, msg):
        # Gets the current position of the robot.
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Gets the robot orientation quaternion 
        q = msg.pose.pose.orientation

        # Converts quaternion into yaw angle.
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        # Marks True if odometry is received.
        self.odom_received = True

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def distance(self, p1, p2):
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return math.sqrt(dx * dx + dy * dy)

    def find_closest_index(self):
        robot_pos = (self.x, self.y)

        best_index = self.closest_index
        best_dist = float("inf")

        # Search forward from current closest index.
        for i in range(self.closest_index, len(self.path)):
            d = self.distance(robot_pos, self.path[i])
            if d < best_dist:
                best_dist = d
                best_index = i

        self.closest_index = best_index
        return best_index

    def find_lookahead_point(self):
        robot_pos = (self.x, self.y)

        closest = self.find_closest_index()

        for i in range(closest, len(self.path)):
            d = self.distance(robot_pos, self.path[i])
            if d >= self.lookahead_distance:
                return self.path[i], i

        return self.path[-1], len(self.path) - 1

    def transform_to_robot_frame(self, point):
        dx = point[0] - self.x
        dy = point[1] - self.y

        # Rotate world-frame error into robot frame.
        x_robot = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy
        y_robot = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy

        return x_robot, y_robot

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def publish_path_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pure_pursuit_path"
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
            p.z = 0.03
            marker.points.append(p)

        self.path_marker_pub.publish(marker)

    def publish_lookahead_marker(self, point):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pure_pursuit_lookahead"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.08
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

        if not self.odom_received:
            return

        if self.finished:
            self.publish_stop()
            return

        final_goal = self.path[-1]
        final_distance = self.distance((self.x, self.y), final_goal)

        if final_distance < self.goal_tolerance and self.closest_index >= len(self.path) - 2:
            self.finished = True
            self.publish_stop()
            self.get_logger().info("Pure Pursuit path complete.")
            return

        lookahead_point, lookahead_index = self.find_lookahead_point()
        self.publish_lookahead_marker(lookahead_point)

        x_robot, y_robot = self.transform_to_robot_frame(lookahead_point)

        cmd = Twist()

        # If lookahead point is behind robot, rotate in place.
        if x_robot <= 0.05:
            cmd.linear.x = 0.0
            cmd.angular.z = self.max_angular_speed if y_robot > 0 else -self.max_angular_speed
            self.cmd_pub.publish(cmd)
            return

        # Pure pursuit curvature formula for 2D path tracking:
        # curvature = 2*y / Ld^2
        curvature = 2.0 * y_robot / (self.lookahead_distance * self.lookahead_distance)

        cmd.linear.x = self.linear_speed
        cmd.angular.z = cmd.linear.x * curvature

        cmd.angular.z = self.clamp(
            cmd.angular.z,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        self.cmd_pub.publish(cmd)
        
        self.get_logger().info(
            "\n"
            "================ Pure Pursuit Control DEBUG ================\n"
            f"{'Path Index':<18}: {lookahead_index}\n"
            f"{'Robot Pose':<18}: x={self.x:>8.2f}, y={self.y:>8.2f}, yaw={self.yaw:>8.2f}\n"
            f"{'Lookahead point':<18}: x0 = {lookahead_point[0]:>8.2f}, y0 = {lookahead_point[1]:>8.2f}\n"
            f"{'robot_frame':<18}: {x_robot:>8.2f}, {y_robot:>8.2f}\n"
            f"{'Curvature':<18}: {curvature:>8.2f}\n"
            f"{'Cmd':<18}: Linear.x = {cmd.linear.x:>8.2f}, Angular.z = {cmd.angular.z:>8.2f}\n"
            "====================================================\n\n",
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.publish_stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()