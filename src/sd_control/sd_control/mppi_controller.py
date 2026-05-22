#!/usr/bin/env python3

"""
MPPI Path Tracking Controller for a differential-drive robot.

Architecture:

    /planned_path + /odom
            ↓
       MPPI controller
            ↓
         /cmd_vel
            ↓
    Gazebo diff_drive_controller

MPPI = Model Predictive Path Integral control.

At each control step:
    1. Read current robot pose from /odom
    2. Read path from /planned_path
    3. Sample many possible future control sequences
    4. Simulate robot motion for each sequence
    5. Score each rollout by path tracking error, heading error, goal progress,
       and control effort
    6. Take a weighted average of sampled controls
    7. Publish the first control as /cmd_vel
"""

import math
import csv
import os
from datetime import datetime

import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class MPPIController(Node):
    def __init__(self):
        super().__init__("mppi_controller")

        # ============================================================
        # Path storage
        # ============================================================
        self.path = []
        self.closest_index = 0

        # ============================================================
        # Control timing
        # ============================================================
        # The controller runs at 10 Hz.
        # MPPI is more computationally expensive than Pure Pursuit/LQR,
        # so we start with 10 Hz instead of 20 Hz.
        self.dt = 0.10

        # ============================================================
        # MPPI parameters
        # ============================================================
        # horizon_steps:
        #   number of future steps simulated
        #
        # num_samples:
        #   number of candidate control sequences sampled each cycle
        #
        # lambda_:
        #   temperature parameter.
        #   lower = more strongly favor the best rollouts
        #   higher = more averaged / smoother
        #
        # noise_std_v / noise_std_w:
        #   exploration noise for linear and angular velocity
        self.horizon_steps = 20
        self.num_samples = 300
        self.lambda_ = 1.0

        self.noise_std_v = 0.08
        self.noise_std_w = 0.45

        # ============================================================
        # Speed limits
        # ============================================================
        self.min_linear_speed = 0.00
        self.max_linear_speed = 0.28
        self.max_angular_speed = 1.2

        # Nominal/default control sequence.
        # Shape:
        #   horizon_steps x 2
        # Each row:
        #   [v, w]
        self.u_nominal = np.zeros((self.horizon_steps, 2))
        self.u_nominal[:, 0] = 0.18
        self.u_nominal[:, 1] = 0.0

        # ============================================================
        # Cost weights
        # ============================================================
        # These are the tuning knobs.
        #
        # path_error_weight:
        #   penalizes distance from path
        #
        # heading_error_weight:
        #   penalizes robot heading not matching path heading
        #
        # goal_distance_weight:
        #   encourages progress toward final goal
        #
        # control_weight:
        #   penalizes large commands
        #
        # angular_rate_weight:
        #   discourages aggressive spinning
        #
        # terminal_goal_weight:
        #   extra final-state cost to end closer to goal
        self.path_error_weight = 8.0
        self.heading_error_weight = 1.5
        self.goal_distance_weight = 0.6
        self.control_weight = 0.1
        self.angular_rate_weight = 0.2
        self.terminal_goal_weight = 3.0

        # ============================================================
        # Goal behavior
        # ============================================================
        self.goal_tolerance = 0.18
        self.slowdown_distance = 0.70

        # ============================================================
        # Robot state from /odom
        # ============================================================
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.odom_received = False
        self.path_received = False
        self.finished = False

        # ============================================================
        # Random number generator
        # ============================================================
        self.rng = np.random.default_rng()

        # ============================================================
        # CSV logging
        # ============================================================
        self.log_dir = os.path.expanduser("~/self_drive_ws/logs/controller_logs")
        os.makedirs(self.log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y_%m_%d_%H%M%S")
        self.log_path = os.path.join(
            self.log_dir,
            f"mppi_log_{timestamp}.csv"
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
            "cross_track_error",
            "heading_error",
            "goal_distance",
            "best_cost",
            "mean_cost",
            "cmd_linear_x",
            "cmd_angular_z"
        ])

        self.get_logger().info(f"Logging MPPI data to: {self.log_path}")

        # ============================================================
        # Subscribers
        # ============================================================
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

        # ============================================================
        # Publisher
        # ============================================================
        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # ============================================================
        # RViz markers
        # ============================================================
        self.path_marker_pub = self.create_publisher(
            Marker,
            "/mppi_path",
            10
        )

        self.best_rollout_marker_pub = self.create_publisher(
            Marker,
            "/mppi_best_rollout",
            10
        )

        self.closest_marker_pub = self.create_publisher(
            Marker,
            "/mppi_closest_point",
            10
        )

        # ============================================================
        # Timer
        # ============================================================
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("MPPI controller started.")
        self.get_logger().info("Waiting for /planned_path and /odom...")

    # ================================================================
    # ROS callbacks
    # ================================================================

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

        # Reset nominal control sequence for the new path.
        self.u_nominal[:, 0] = 0.18
        self.u_nominal[:, 1] = 0.0

        self.get_logger().info(f"Received new /planned_path with {len(self.path)} points.")

    # ================================================================
    # Utility functions
    # ================================================================

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

    # ================================================================
    # Path geometry
    # ================================================================

    def find_closest_index_for_pose(self, x, y, start_index=0):
        robot_pos = (x, y)

        best_index = start_index
        best_dist = float("inf")

        for i in range(start_index, len(self.path)):
            d = self.distance(robot_pos, self.path[i])

            if d < best_dist:
                best_dist = d
                best_index = i

        return best_index, best_dist

    def find_closest_index(self):
        best_index, best_dist = self.find_closest_index_for_pose(
            self.x,
            self.y,
            self.closest_index
        )

        self.closest_index = best_index
        return best_index, best_dist

    def get_path_heading(self, index):
        if index >= len(self.path) - 1:
            index = len(self.path) - 2

        x1, y1 = self.path[index]
        x2, y2 = self.path[index + 1]

        return math.atan2(y2 - y1, x2 - x1)

    def compute_signed_cross_track_error_for_pose(self, x, y, index):
        if index >= len(self.path) - 1:
            index = len(self.path) - 2

        path_x, path_y = self.path[index]
        path_yaw = self.get_path_heading(index)

        dx = x - path_x
        dy = y - path_y

        y_path = -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy

        return y_path

    # ================================================================
    # Robot model rollout
    # ================================================================

    def rollout_dynamics(self, x, y, yaw, v, w):
        """
        Differential-drive / unicycle model:

            x_dot   = v cos(yaw)
            y_dot   = v sin(yaw)
            yaw_dot = w

        Discrete Euler integration:
            x[k+1]   = x[k] + dt * v cos(yaw)
            y[k+1]   = y[k] + dt * v sin(yaw)
            yaw[k+1] = yaw[k] + dt * w
        """

        x_next = x + self.dt * v * math.cos(yaw)
        y_next = y + self.dt * v * math.sin(yaw)
        yaw_next = self.normalize_angle(yaw + self.dt * w)

        return x_next, y_next, yaw_next

    def rollout_and_score(self, control_sequence):
        """
        Simulate one candidate control sequence and compute its total cost.

        control_sequence shape:
            horizon_steps x 2

        Returns:
            total_cost
            rollout_points
        """

        x = self.x
        y = self.y
        yaw = self.yaw

        total_cost = 0.0
        rollout_points = []

        current_search_index = self.closest_index
        final_goal = self.path[-1]

        for k in range(self.horizon_steps):
            v = control_sequence[k, 0]
            w = control_sequence[k, 1]

            x, y, yaw = self.rollout_dynamics(x, y, yaw, v, w)
            rollout_points.append((x, y))

            closest_index, path_dist = self.find_closest_index_for_pose(
                x,
                y,
                current_search_index
            )

            current_search_index = closest_index

            path_yaw = self.get_path_heading(closest_index)
            heading_error = self.normalize_angle(yaw - path_yaw)
            goal_distance = self.distance((x, y), final_goal)

            # Cost terms
            path_cost = self.path_error_weight * (path_dist ** 2)
            heading_cost = self.heading_error_weight * (heading_error ** 2)
            goal_cost = self.goal_distance_weight * goal_distance
            control_cost = self.control_weight * (v ** 2)
            angular_cost = self.angular_rate_weight * (w ** 2)

            total_cost += (
                path_cost
                + heading_cost
                + goal_cost
                + control_cost
                + angular_cost
            )

        # Terminal cost: encourage final rollout point closer to final goal.
        total_cost += self.terminal_goal_weight * self.distance((x, y), final_goal)

        return total_cost, rollout_points

    # ================================================================
    # MPPI optimization
    # ================================================================

    def compute_mppi_control(self):
        """
        Sample many control sequences, score them, and compute MPPI weighted
        average control sequence.
        """

        # Noise shape:
        #   num_samples x horizon_steps x 2
        noise = np.zeros((self.num_samples, self.horizon_steps, 2))

        noise[:, :, 0] = self.rng.normal(
            0.0,
            self.noise_std_v,
            size=(self.num_samples, self.horizon_steps)
        )

        noise[:, :, 1] = self.rng.normal(
            0.0,
            self.noise_std_w,
            size=(self.num_samples, self.horizon_steps)
        )

        # Candidate controls = nominal controls + noise
        candidate_controls = self.u_nominal[None, :, :] + noise

        # Saturate controls
        candidate_controls[:, :, 0] = np.clip(
            candidate_controls[:, :, 0],
            self.min_linear_speed,
            self.max_linear_speed
        )

        candidate_controls[:, :, 1] = np.clip(
            candidate_controls[:, :, 1],
            -self.max_angular_speed,
            self.max_angular_speed
        )

        costs = np.zeros(self.num_samples)
        rollouts = []

        best_cost = float("inf")
        best_rollout = []

        for i in range(self.num_samples):
            cost, rollout_points = self.rollout_and_score(candidate_controls[i])
            costs[i] = cost
            rollouts.append(rollout_points)

            if cost < best_cost:
                best_cost = cost
                best_rollout = rollout_points

        # MPPI weights:
        # subtract min cost for numerical stability
        beta = np.min(costs)
        weights = np.exp(-(costs - beta) / self.lambda_)
        weights_sum = np.sum(weights) + 1e-9
        weights = weights / weights_sum

        # Weighted average of candidate control sequences
        weighted_controls = np.zeros_like(self.u_nominal)

        for i in range(self.num_samples):
            weighted_controls += weights[i] * candidate_controls[i]

        # Smooth/update nominal sequence for next iteration
        self.u_nominal = weighted_controls.copy()

        # Shift sequence forward:
        # next cycle starts from the second control
        self.u_nominal[:-1] = self.u_nominal[1:]
        self.u_nominal[-1] = self.u_nominal[-2]

        # First control is applied now
        cmd_v = weighted_controls[0, 0]
        cmd_w = weighted_controls[0, 1]

        return cmd_v, cmd_w, best_cost, float(np.mean(costs)), best_rollout

    # ================================================================
    # RViz markers
    # ================================================================

    def publish_path_marker(self):
        if not self.path:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mppi_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.04

        marker.color.a = 1.0
        marker.color.r = 0.8
        marker.color.g = 0.2
        marker.color.b = 1.0

        for x, y in self.path:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.10
            marker.points.append(p)

        self.path_marker_pub.publish(marker)

    def publish_best_rollout_marker(self, rollout_points):
        if not rollout_points:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mppi_best_rollout"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.035

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.6
        marker.color.b = 0.0

        for x, y in rollout_points:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.15
            marker.points.append(p)

        self.best_rollout_marker_pub.publish(marker)

    def publish_closest_marker(self, point):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mppi_closest_point"
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.15

        marker.scale.x = 0.16
        marker.scale.y = 0.16
        marker.scale.z = 0.16

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        self.closest_marker_pub.publish(marker)

    # ================================================================
    # CSV logging
    # ================================================================

    def log_data(
        self,
        closest_point,
        path_heading,
        cross_track_error,
        heading_error,
        final_distance,
        best_cost,
        mean_cost,
        cmd
    ):
        time_sec = self.get_clock().now().nanoseconds / 1e9

        self.csv_writer.writerow([
            f"{time_sec:.4f}",
            "mppi",
            f"{self.x:.4f}",
            f"{self.y:.4f}",
            f"{self.yaw:.4f}",
            f"{closest_point[0]:.4f}",
            f"{closest_point[1]:.4f}",
            f"{path_heading:.4f}",
            f"{cross_track_error:.4f}",
            f"{heading_error:.4f}",
            f"{final_distance:.4f}",
            f"{best_cost:.4f}",
            f"{mean_cost:.4f}",
            f"{cmd.linear.x:.4f}",
            f"{cmd.angular.z:.4f}"
        ])

        self.log_file.flush()

    # ================================================================
    # Main control loop
    # ================================================================

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
            self.get_logger().info("MPPI path complete.")
            return

        closest_index, path_dist = self.find_closest_index()
        closest_point = self.path[closest_index]
        self.publish_closest_marker(closest_point)

        path_heading = self.get_path_heading(closest_index)
        heading_error = self.normalize_angle(self.yaw - path_heading)
        cross_track_error = self.compute_signed_cross_track_error_for_pose(
            self.x,
            self.y,
            closest_index
        )

        cmd_v, cmd_w, best_cost, mean_cost, best_rollout = self.compute_mppi_control()

        # Slow down near goal
        if final_distance < self.slowdown_distance:
            cmd_v = min(cmd_v, 0.10)

        cmd = Twist()
        cmd.linear.x = self.clamp(
            cmd_v,
            self.min_linear_speed,
            self.max_linear_speed
        )
        cmd.angular.z = self.clamp(
            cmd_w,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        self.cmd_pub.publish(cmd)

        self.publish_best_rollout_marker(best_rollout)

        self.log_data(
            closest_point,
            path_heading,
            cross_track_error,
            heading_error,
            final_distance,
            best_cost,
            mean_cost,
            cmd
        )

        self.get_logger().info(
            "\n"
            "================ MPPI Control DEBUG ================\n"
            f"{'Path Index':<18}: {closest_index}/{len(self.path)}\n"
            f"{'Robot Pose':<18}: x={self.x:>8.2f}, y={self.y:>8.2f}, yaw={self.yaw:>8.2f}\n" 
            f"{'Path Yaw':<18}: {path_heading:>8.2f}\n"
            f"{'Heading Error':<18}: {heading_error:>8.2f}\n"
            f"{'Cross Track Error':<18}: {cross_track_error:>8.3f}\n"
            f"{'Goal Distance':<18}: {final_distance:>8.2f}\n"
            f"{'Cost':<18}: Best Cost = {best_cost:>8.2f}, Mean Cost ={mean_cost:>8.2f}\n"
            f"{'Command':<18}: Linear.x:{cmd.linear.x:>8.2f}, Angular.z:{cmd.angular.z:>8.2f}\n"
            "====================================================\n\n",
            throttle_duration_sec=1.0
        )



def main(args=None):
    rclpy.init(args=args)
    node = MPPIController()

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