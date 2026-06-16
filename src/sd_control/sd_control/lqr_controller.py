#!/usr/bin/env python3

"""
LQR Path Tracking Controller for a differential-drive robot.

Architecture:

    /planned_path + /odom
            ↓
      LQR controller
            ↓
         /cmd_vel
            ↓
    Gazebo diff_drive_controller

This controller does NOT do global planning.
It assumes Nav2 planner has already generated a path and published it on:

    /planned_path

The controller's job is only to track that path.
"""

import math
import csv
import os
from datetime import datetime

import numpy as np

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


class LQRController(Node):
    def __init__(self):
        """
        Constructor.

        This runs once when the node starts.
        Here we initialize parameters, subscribers, publishers, logging,
        and the timer that repeatedly runs the control loop.
        """

        super().__init__("lqr_controller")

        # Path storage
        # self.path will store the planned path as a list of (x, y) tuples.
        # self.closest_index remembers where we are on the path.
        # We search forward from this index so the controller does not
        # jump backward to earlier path points.
        self.path = []
        self.closest_index = 0

        # Control loop timing
        # The controller runs at 20 Hz
        # This matches a reasonable controller frequency for this robot.
        self.dt = 0.05

        # Speed limits
        self.max_linear_speed = 0.22
        self.min_linear_speed = 0.08
        self.goal_linear_speed = 0.10
        self.max_angular_speed = 1.0

        #
        # Goal behavior
        #
        # goal_tolerance:
        #   if robot is within this distance of final path point,
        #   it stops
        #
        # slowdown_distance:
        #   when robot is this close to final goal, reduce speed
        self.goal_tolerance = 0.15
        self.slowdown_distance = 0.60

        #
        # LQR weights
        #
        # LQR minimizes a cost function:
        #
        #   cost = sum(x.T Q x + u.T R u)
        #
        # State:
        #   x = [cross_track_error, heading_error]
        #
        # Control:
        #   u = angular velocity command
        #
        # Q tells LQR how much we care about state errors.
        # Larger Q values make the controller correct errors more strongly.
        #
        # R tells LQR how much we penalize aggressive control effort.
        # Larger R makes angular velocity smoother/less aggressive.
        #
        # Here:
        #   Q[0,0] = 4.0  -> care about cross-track error
        #   Q[1,1] = 2.0  -> care about heading error
        #   R      = 0.8  -> avoid excessive angular velocity
        self.Q = np.diag([4.0, 2.0])
        self.R = np.array([[0.8]])

        #
        # Robot state from odometry
        #
        # These are updated every time we receive /odom.
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Frame setup.
        # /planned_path is in map frame, so robot pose must also be in map frame.
        self.path_frame = "map"
        self.robot_frame = "base_footprint"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Flags to make sure the controller does not run before it has data.
        self.odom_received = False
        self.path_received = False
        self.finished = False

        #
        # CSV logging setup
        #
        # Logs are saved here:
        #
        #   ~/self_drive_ws/logs/controller_logs/
        #
        # Each run creates a timestamped CSV file.
        # Later we can compare Pure Pursuit, Stanley, LQR, MPC, etc.
        self.log_dir = os.path.expanduser("~/self_drive_ws/logs/controller_logs")
        os.makedirs(self.log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y_%m_%d_%H%M%S")
        self.log_path = os.path.join(
            self.log_dir,
            f"lqr_log_{timestamp}.csv"
        )

        self.log_file = open(self.log_path, "w", newline="")
        self.csv_writer = csv.writer(self.log_file)

        # CSV header row.
        # Each control-loop step writes one row.
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
            "lqr_k_cte",
            "lqr_k_heading",
            "cmd_linear_x",
            "cmd_angular_z"
        ])

        self.get_logger().info(f"Logging LQR data to: {self.log_path}")

        # /odom gives the robot's current pose and orientation.
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        # /planned_path is produced by your nav2_plan_client.
        #
        # Flow:
        #   RViz 2D Goal Pose
        #       -> /goal_pose
        #       -> nav2_plan_client
        #       -> Nav2 planner_server
        #       -> /planned_path
        #
        # This controller subscribes to /planned_path and tracks it.
        self.path_sub = self.create_subscription(
            Path,
            "/planned_path",
            self.path_callback,
            10
        )

        #
        # Publisher
        #
        # /cmd_vel is what Gazebo's diff_drive_controller listens to.
        #
        # We publish:
        #   linear.x  = forward speed
        #   angular.z = yaw rate
        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        #
        # RViz visualization publishers
        #
        # These make debugging easier.
        #
        # /lqr_path:
        #   line strip showing the planned path
        #
        # /lqr_closest_point:
        #   sphere showing the closest path point currently being tracked
        self.path_marker_pub = self.create_publisher(
            Marker,
            "/lqr_path",
            10
        )

        self.closest_marker_pub = self.create_publisher(
            Marker,
            "/lqr_closest_point",
            10
        )

        #
        # Timer
        #
        # Runs control_loop() every self.dt seconds.
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("LQR controller started.")
        self.get_logger().info("Waiting for /planned_path and /odom...")

    #
    # Odometry callback
    #

    def odom_callback(self, msg):
        """
        Called every time a new /odom message arrives.

        Extracts:
            x position
            y position
            yaw angle

        ROS odometry orientation is a quaternion.
        Since this is a 2D ground robot, we only need yaw.
        """

        # self.x = msg.pose.pose.position.x
        # self.y = msg.pose.pose.position.y

        # q = msg.pose.pose.orientation

        # Quaternion to yaw conversion:
        #
        # yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
        #
        # This avoids needing tf_transformations.
        # siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        # cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        # self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.odom_received = True
    
    def update_robot_pose_from_tf(self):
        """
        Update robot pose using TF.

        This gets the pose of base_footprint expressed in map frame,
        so it matches the frame used by /planned_path.
        """

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

    # Path callback

    def path_callback(self, msg):
        """
        Called when a new /planned_path message arrives.

        Converts nav_msgs/Path into a plain Python list:
            [(x1, y1), (x2, y2), ...]

        Resets closest_index and finished flag so the new path can be tracked.
        """

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

    # Utility functions

    def normalize_angle(self, angle):
        """
        Wrap angle to [-pi, pi].

        This prevents issues where, for example:
            +179 degrees and -179 degrees
        should be treated as only 2 degrees apart, not 358 degrees apart.
        """

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def clamp(self, value, min_value, max_value):
        """
        Saturate value between min_value and max_value.
        """

        return max(min(value, max_value), min_value)

    def distance(self, p1, p2):
        """
        Euclidean distance between two 2D points.
        """

        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return math.sqrt(dx * dx + dy * dy)

    def publish_stop(self):
        """
        Publish zero velocity command.
        """

        self.cmd_pub.publish(Twist())

    # Path geometry functions

    def find_closest_index(self):
        """
        Find the closest path point to the robot.

        Important detail:
            We search from self.closest_index forward.

        Why?
            If the robot moves along the path, we don't want the closest point
            to jump backward to a point it already passed.
        """

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
        """
        Estimate path heading at a given path index.

        It uses the direction from:
            path[index] to path[index + 1]

        heading = atan2(dy, dx)
        """

        if index >= len(self.path) - 1:
            index = len(self.path) - 2

        x1, y1 = self.path[index]
        x2, y2 = self.path[index + 1]

        return math.atan2(y2 - y1, x2 - x1)

    def compute_signed_cross_track_error(self, index):
        """
        Compute signed cross-track error.

        Cross-track error means:
            lateral distance from robot to the path.

        Sign convention:
            positive if robot is to the left of the path direction
            negative if robot is to the right of the path direction

        This is done by transforming the robot position error
        into the path's local coordinate frame.
        """

        if index >= len(self.path) - 1:
            index = len(self.path) - 2

        path_x, path_y = self.path[index]
        path_yaw = self.get_path_heading(index)

        dx = self.x - path_x
        dy = self.y - path_y

        # Rotate world-frame error into path frame.
        # The path-frame y coordinate is lateral error.
        y_path = -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy

        return y_path

    #
    # LQR functions
    #

    def solve_discrete_lqr(self, A, B, Q, R):
        """
        Solve discrete-time LQR.

        System model:
            x[k+1] = A x[k] + B u[k]

        Cost:
            J = sum(x.T Q x + u.T R u)

        LQR produces:
            u = -Kx

        where:
            K is the optimal feedback gain matrix.

        This function solves the discrete algebraic Riccati equation
        iteratively.

        For this project, scipy may not always be installed, so we use
        a simple manual iterative solver with numpy.
        """

        P = Q.copy()

        for _ in range(100):
            BT_P = B.T @ P

            # K = (R + B.T P B)^-1 B.T P A
            K = np.linalg.inv(R + BT_P @ B) @ (BT_P @ A)

            # Riccati update
            P_next = Q + A.T @ P @ A - A.T @ P @ B @ K

            # Stop when solution stops changing significantly.
            if np.max(np.abs(P_next - P)) < 1e-6:
                P = P_next
                break

            P = P_next

        K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)

        return K

    def compute_lqr_gain(self, speed):
        """
        Compute LQR gain for a simplified lateral tracking model.

        State:
            e_y     = cross-track error
            e_theta = heading error

        Input:
            omega = angular velocity command

        Continuous intuition:
            e_y_dot     ≈ v * e_theta
            e_theta_dot = omega

        Discrete model:
            e_y[k+1]     = e_y[k] + v * dt * e_theta[k]
            e_theta[k+1] = e_theta[k] + dt * omega[k]

        Matrix form:
            x[k+1] = A x[k] + B u[k]

        where:
            x = [e_y, e_theta]
            u = omega
        """

        A = np.array([
            [1.0, speed * self.dt],
            [0.0, 1.0]
        ])

        B = np.array([
            [0.0],
            [self.dt]
        ])

        K = self.solve_discrete_lqr(A, B, self.Q, self.R)

        return K

    #
    # RViz marker functions
    #

    def publish_path_marker(self):
        """
        Publish the planned path as a line strip marker in RViz.
        """

        if not self.path:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lqr_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.04

        # Cyan-ish path color
        marker.color.a = 1.0
        marker.color.r = 0.2
        marker.color.g = 1.0
        marker.color.b = 1.0

        for x, y in self.path:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.08
            marker.points.append(p)

        self.path_marker_pub.publish(marker)

    def publish_closest_marker(self, point):
        """
        Publish the current closest path point as a sphere marker.
        """

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lqr_closest_point"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.12

        marker.scale.x = 0.16
        marker.scale.y = 0.16
        marker.scale.z = 0.16

        # Orange sphere
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.4
        marker.color.b = 0.0

        self.closest_marker_pub.publish(marker)

    #
    # CSV logging
    #

    def log_data(
        self,
        closest_point,
        path_heading,
        heading_error,
        cross_track_error,
        final_distance,
        K,
        cmd
    ):
        """
        Write one row of controller data to CSV.

        This is useful for comparing:
            Pure Pursuit vs Stanley vs LQR vs MPC
        """

        time_sec = self.get_clock().now().nanoseconds / 1e9

        self.csv_writer.writerow([
            f"{time_sec:.4f}",
            "lqr",
            f"{self.x:.4f}",
            f"{self.y:.4f}",
            f"{self.yaw:.4f}",
            f"{closest_point[0]:.4f}",
            f"{closest_point[1]:.4f}",
            f"{path_heading:.4f}",
            f"{heading_error:.4f}",
            f"{cross_track_error:.4f}",
            f"{final_distance:.4f}",
            f"{K[0, 0]:.4f}",
            f"{K[0, 1]:.4f}",
            f"{cmd.linear.x:.4f}",
            f"{cmd.angular.z:.4f}"
        ])

        # Flush every row so data is saved even if the node is stopped.
        self.log_file.flush()

    #
    # Main control loop
    #

    def control_loop(self):
        """
        Runs every self.dt seconds.

        Steps:
            1. Wait until odom and path are available
            2. Find closest point on path
            3. Compute path heading
            4. Compute cross-track error and heading error
            5. Build LQR state vector
            6. Compute LQR gain
            7. Compute angular velocity command
            8. Publish /cmd_vel
            9. Log data
        """

        # Keep publishing the path visualization.
        self.publish_path_marker()

        # Do nothing until both odometry and path are available.
        # if not self.odom_received or not self.path_received:
        #     return

        if not self.path_received:
            return
        
        if not self.update_robot_pose_from_tf():
            return

        # If controller has completed the path, keep robot stopped.
        if self.finished:
            self.publish_stop()
            return

        # Final goal is the last point in the planned path.
        final_goal = self.path[-1]
        final_distance = self.distance((self.x, self.y), final_goal)

        # Stop when close enough to the final goal.
        if final_distance < self.goal_tolerance and self.closest_index >= len(self.path) - 2:
            self.finished = True
            self.publish_stop()
            self.get_logger().info("LQR path complete.")
            return

        # Find path point nearest to robot.
        closest_index, _ = self.find_closest_index()
        closest_point = self.path[closest_index]

        # Show closest point in RViz.
        self.publish_closest_marker(closest_point)

        # Get path tangent direction.
        path_heading = self.get_path_heading(closest_index)

        # Compute errors.
        #
        # cross_track_error:
        #   lateral displacement from path
        #
        # heading_error:
        #   robot heading relative to path heading
        #
        # Here we use:
        #   heading_error = robot yaw - path yaw
        #
        # If the controller turns the wrong direction during testing,
        # this sign convention is the first place to revisit.
        cross_track_error = self.compute_signed_cross_track_error(closest_index)
        heading_error = self.normalize_angle(self.yaw - path_heading)

        # Choose forward speed.
        speed = self.max_linear_speed

        # Slow down near final goal.
        if final_distance < self.slowdown_distance:
            speed = self.goal_linear_speed

        speed = self.clamp(
            speed,
            self.min_linear_speed,
            self.max_linear_speed
        )

        # Build LQR state vector.
        #
        # state =
        #   [ cross_track_error ]
        #   [ heading_error      ]
        state = np.array([
            [cross_track_error],
            [heading_error]
        ])

        # Compute LQR feedback gain for current speed.
        K = self.compute_lqr_gain(speed)

        # LQR control law:
        #
        #   u = -Kx
        #
        # Here u is angular velocity command.
        angular_cmd = float((-K @ state)[0, 0])

        # Build Twist command.
        cmd = Twist()
        cmd.linear.x = speed

        # Saturate angular velocity.
        cmd.angular.z = self.clamp(
            angular_cmd,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        # Publish velocity command to Gazebo.
        self.cmd_pub.publish(cmd)

        # Save data to CSV.
        self.log_data(
            closest_point,
            path_heading,
            heading_error,
            cross_track_error,
            final_distance,
            K,
            cmd
        )

        # Human-readable terminal log.
        self.get_logger().info(
            "\n"
            "================ LQR Control DEBUG ================\n"
            f"{'Path Index':<18}: {closest_index}/{len(self.path)}\n"
            f"{'Robot Pose':<18}: x={self.x:>8.2f}, y={self.y:>8.2f}, yaw={self.yaw:>8.2f}\n" 
            f"{'Path Yaw':<18}: {path_heading:>8.2f}\n"
            f"{'Heading Error':<18}: {heading_error:>8.2f}\n"
            f"{'Cross Track Error':<18}: {cross_track_error:>8.3f}\n"
            f"{'K':<18}: {K[0,0]:>8.2f}, {K[0,1]:>8.2f}\n"
            f"{'Goal Distance':<18}: {final_distance:>8.2f}\n"
            f"{'Command':<18}: Linear.x:{cmd.linear.x:>8.2f}, Angular.z:{cmd.angular.z:>8.2f}\n"
            "====================================================\n\n",
            throttle_duration_sec=1.0
        )


def main(args=None):
    """
    ROS node entry point.
    """

    rclpy.init(args=args)
    node = LQRController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Stop robot when shutting down.
    node.publish_stop()

    # Close CSV log safely.
    if hasattr(node, "log_file"):
        node.log_file.close()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()