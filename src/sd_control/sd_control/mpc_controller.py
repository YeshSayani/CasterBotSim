#!/usr/bin/env python3

"""
MPC Path Tracking Controller for a differential-drive robot.

This controller is meant to work in your custom-controller architecture:

    RViz 2D Goal Pose
            ↓
        /goal_pose
            ↓
      nav2_plan_client
            ↓
      Nav2 planner_server
            ↓
       /planned_path
            ↓
       MPC controller
            ↓
         /cmd_vel
            ↓
    Gazebo diff_drive_controller

Important:
    This controller does NOT create the global path.
    Nav2 creates the global path.
    MPC only tracks the already-created /planned_path.

MPC means Model Predictive Control.

At every control cycle, this controller:
    1. Reads current robot pose from /odom.
    2. Reads planned path from /planned_path.
    3. Predicts the robot's future motion for many candidate commands.
    4. Scores those future motions using a cost function.
    5. Chooses the velocity sequence with the lowest cost.
    6. Applies only the first velocity command.
    7. Repeats the process at the next cycle.

Robot state:
    x, y, yaw

Control inputs:
    v = linear velocity
    w = angular velocity

Robot model:
    x[k+1]   = x[k] + dt * v[k] * cos(yaw[k])
    y[k+1]   = y[k] + dt * v[k] * sin(yaw[k])
    yaw[k+1] = yaw[k] + dt * w[k]
"""

# math gives us sin, cos, atan2, pi, sqrt, etc.
import math

# csv is used to log controller performance data to a CSV file.
import csv

# os is used for creating folders and expanding ~/ paths.
import os

# datetime is used to create timestamped log filenames.
from datetime import datetime

# numpy is used for arrays, optimization variables, and numeric operations.
import numpy as np

# scipy.optimize.minimize is the nonlinear optimizer used by this MPC.
from scipy.optimize import minimize

# rclpy is the Python ROS 2 client library.
import rclpy

# Node is the base class for creating a ROS 2 node.
from rclpy.node import Node

# Odometry gives current robot pose and velocity.
# Path is the message type for /planned_path.
from nav_msgs.msg import Odometry, Path

# Twist is the message type for /cmd_vel.
from geometry_msgs.msg import Twist

# Marker lets us draw lines/spheres in RViz for debugging.
from visualization_msgs.msg import Marker

# Point is used inside Marker messages.
from geometry_msgs.msg import Point


class MPCController(Node):
    """
    ROS 2 node that implements MPC path tracking.

    Subscribes:
        /odom
        /planned_path

    Publishes:
        /cmd_vel
        /mpc_path
        /mpc_prediction
        /mpc_closest_point
    """

    def __init__(self):
        """
        Constructor.

        This runs once when the node starts.
        It initializes:
            - controller parameters
            - robot state variables
            - CSV logging
            - subscribers
            - publishers
            - timer loop
        """

        # Initialize the ROS 2 node with name "mpc_controller".
        super().__init__("mpc_controller")

        # ============================================================
        # Path storage
        # ============================================================

        # This list stores the current planned path as plain (x, y) tuples.
        # Example:
        #   self.path = [(0.0, 0.0), (0.1, 0.0), (0.2, 0.05), ...]
        self.path = []

        # closest_index stores the index of the closest path point.
        # We use it to avoid searching backward on the path every cycle.
        self.closest_index = 0

        # ============================================================
        # MPC timing and prediction horizon
        # ============================================================

        # dt is the time step used for both:
        #   1. the controller timer period
        #   2. the internal MPC prediction model
        #
        # dt = 0.10 means the controller runs at 10 Hz.
        self.dt = 0.10

        # Number of future steps the MPC predicts.
        #
        # horizon_steps = 12 and dt = 0.10 means:
        #   prediction horizon = 12 * 0.10 = 1.2 seconds
        self.horizon_steps = 12

        # ============================================================
        # Velocity limits
        # ============================================================

        # Minimum forward speed.
        # We allow zero because MPC may need to stop near the goal.
        self.min_linear_speed = 0.00

        # Maximum forward speed.
        self.max_linear_speed = 0.25

        # Maximum absolute angular speed.
        # The controller clamps angular velocity to [-1.0, 1.0].
        self.max_angular_speed = 1.0

        # ============================================================
        # Goal behavior
        # ============================================================

        # If the robot is within this distance of the final path point,
        # we consider the goal reached.
        self.goal_tolerance = 0.18

        # If the robot is closer than this to the final goal,
        # reduce speed.
        self.slowdown_distance = 0.70

        # Speed cap near the goal.
        self.goal_linear_speed = 0.10

        # ============================================================
        # MPC cost weights
        # ============================================================

        # Penalizes distance from the path.
        # Higher value = robot tries harder to stay close to the path.
        self.path_error_weight = 10.0

        # Penalizes yaw misalignment with the path direction.
        # Higher value = robot tries harder to face along the path.
        self.heading_error_weight = 2.0

        # Penalizes being far from the final goal.
        # This encourages progress along the path.
        self.goal_distance_weight = 0.8

        # Penalizes large v and w commands.
        # Higher value = more conservative control.
        self.control_effort_weight = 0.1

        # Penalizes sudden changes in v and w from one future step to the next.
        # Higher value = smoother commands.
        self.control_smoothness_weight = 0.3

        # Extra cost on the final predicted state being far from the final goal.
        self.terminal_goal_weight = 5.0

        # ============================================================
        # Robot state from /odom
        # ============================================================

        # Robot x position in odom/map-like coordinate frame.
        self.x = 0.0

        # Robot y position.
        self.y = 0.0

        # Robot yaw angle in radians.
        self.yaw = 0.0

        # Becomes True after first /odom message is received.
        self.odom_received = False

        # Becomes True after first /planned_path is received.
        self.path_received = False

        # Becomes True after the controller reaches the goal.
        self.finished = False

        # ============================================================
        # Warm-start control guess
        # ============================================================

        # The MPC optimizer chooses a sequence:
        #
        #   [v0, w0, v1, w1, v2, w2, ..., vN-1, wN-1]
        #
        # There are 2 values per horizon step:
        #   v = linear velocity
        #   w = angular velocity
        #
        # So total optimization variables = 2 * horizon_steps.
        self.u_prev = np.zeros(2 * self.horizon_steps)

        # Initialize the warm-start guess.
        # Start with constant forward speed and zero turn rate.
        for k in range(self.horizon_steps):
            # Linear velocity guess at step k.
            self.u_prev[2 * k] = 0.15

            # Angular velocity guess at step k.
            self.u_prev[2 * k + 1] = 0.0

        # ============================================================
        # CSV logging
        # ============================================================

        # Folder where controller logs will be saved.
        self.log_dir = os.path.expanduser("~/self_drive_ws/logs/controller_logs")

        # Create the log folder if it does not exist.
        os.makedirs(self.log_dir, exist_ok=True)

        # Create a timestamp like: 2026_05_24_153012
        timestamp = datetime.now().strftime("%Y_%m_%d_%H%M%S")

        # Full path to the CSV log file.
        self.log_path = os.path.join(
            self.log_dir,
            f"mpc_log_{timestamp}.csv"
        )

        # Open the CSV file for writing.
        self.log_file = open(self.log_path, "w", newline="")

        # Create a CSV writer object.
        self.csv_writer = csv.writer(self.log_file)

        # Write CSV header row.
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
            "mpc_cost",
            "cmd_linear_x",
            "cmd_angular_z",
            "optimizer_success"
        ])

        # Print where the log is being saved.
        self.get_logger().info(f"Logging MPC data to: {self.log_path}")

        # ============================================================
        # Subscribers
        # ============================================================

        # Subscribe to /odom.
        # Every new odom message calls self.odom_callback().
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        # Subscribe to /planned_path.
        # Every new planned path calls self.path_callback().
        self.path_sub = self.create_subscription(
            Path,
            "/planned_path",
            self.path_callback,
            10
        )

        # ============================================================
        # Publisher
        # ============================================================

        # Publish velocity commands to /cmd_vel.
        # Gazebo's diff_drive_controller subscribes to this topic.
        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # ============================================================
        # RViz marker publishers
        # ============================================================

        # Publishes the planned path as a line in RViz.
        self.path_marker_pub = self.create_publisher(
            Marker,
            "/mpc_path",
            10
        )

        # Publishes the MPC predicted future trajectory as a line in RViz.
        self.prediction_marker_pub = self.create_publisher(
            Marker,
            "/mpc_prediction",
            10
        )

        # Publishes the closest path point as a sphere in RViz.
        self.closest_marker_pub = self.create_publisher(
            Marker,
            "/mpc_closest_point",
            10
        )

        # ============================================================
        # Timer
        # ============================================================

        # Call self.control_loop() every self.dt seconds.
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Startup messages.
        self.get_logger().info("MPC controller started.")
        self.get_logger().info("Waiting for /planned_path and /odom...")

    # ================================================================
    # ROS callbacks
    # ================================================================

    def odom_callback(self, msg):
        """
        Runs whenever a new /odom message is received.

        Extracts:
            x position
            y position
            yaw angle

        The orientation in ROS is a quaternion.
        For a 2D ground robot, we only need yaw.
        """

        # Extract x position from odometry.
        self.x = msg.pose.pose.position.x

        # Extract y position from odometry.
        self.y = msg.pose.pose.position.y

        # Extract quaternion orientation.
        q = msg.pose.pose.orientation

        # Convert quaternion to yaw.
        #
        # Formula:
        #   yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        # Mark that odom is available.
        self.odom_received = True

    def path_callback(self, msg):
        """
        Runs whenever a new /planned_path message is received.

        Converts nav_msgs/Path into a simple list of (x, y) points.
        """

        # Ignore invalid paths.
        if len(msg.poses) < 2:
            self.get_logger().warn("Received path with fewer than 2 poses. Ignoring.")
            return

        # Clear old path.
        self.path = []

        # Convert each PoseStamped in nav_msgs/Path into an (x, y) tuple.
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.path.append((x, y))

        # Reset progress along the path.
        self.closest_index = 0

        # Mark that path is available.
        self.path_received = True

        # New path means controller is no longer finished.
        self.finished = False

        # Reset warm-start guess for new path.
        for k in range(self.horizon_steps):
            self.u_prev[2 * k] = 0.15
            self.u_prev[2 * k + 1] = 0.0

        self.get_logger().info(f"Received new /planned_path with {len(self.path)} points.")

    # ================================================================
    # Utility functions
    # ================================================================

    def normalize_angle(self, angle):
        """
        Wrap an angle into [-pi, pi].

        This prevents discontinuities such as:
            +179 degrees and -179 degrees
        being treated as 358 degrees apart.
        """

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def clamp(self, value, min_value, max_value):
        """
        Clamp value between min_value and max_value.
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

    # ================================================================
    # Path geometry functions
    # ================================================================

    def find_closest_index_for_pose(self, x, y, start_index=0):
        """
        Find the closest path point to an arbitrary pose (x, y).

        start_index lets us search forward from a known point on the path.
        This prevents the controller from jumping backward.
        """

        robot_pos = (x, y)

        # Initialize best index and distance.
        best_index = start_index
        best_dist = float("inf")

        # Search from start_index to the end of the path.
        for i in range(start_index, len(self.path)):
            d = self.distance(robot_pos, self.path[i])

            # If this point is closer, save it.
            if d < best_dist:
                best_dist = d
                best_index = i

        return best_index, best_dist

    def find_closest_index(self):
        """
        Find closest path point to the actual robot pose.

        Updates self.closest_index.
        """

        best_index, best_dist = self.find_closest_index_for_pose(
            self.x,
            self.y,
            self.closest_index
        )

        self.closest_index = best_index

        return best_index, best_dist

    def get_path_heading(self, index):
        """
        Estimate path direction at a given path index.

        Uses the vector:
            path[index] → path[index + 1]
        """

        # If index is at the last point, use the previous segment.
        if index >= len(self.path) - 1:
            index = len(self.path) - 2

        # Current path point.
        x1, y1 = self.path[index]

        # Next path point.
        x2, y2 = self.path[index + 1]

        # atan2 gives heading angle of the path segment.
        return math.atan2(y2 - y1, x2 - x1)

    def compute_signed_cross_track_error_for_pose(self, x, y, index):
        """
        Compute signed cross-track error for pose (x, y).

        Cross-track error is lateral distance from path.

        Positive/negative sign tells which side of the path the robot is on.
        """

        if index >= len(self.path) - 1:
            index = len(self.path) - 2

        # Path reference point.
        path_x, path_y = self.path[index]

        # Path heading at that point.
        path_yaw = self.get_path_heading(index)

        # Error from path point to robot/predicted pose.
        dx = x - path_x
        dy = y - path_y

        # Rotate error into path frame.
        # y_path is lateral error.
        y_path = -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy

        return y_path

    # ================================================================
    # Robot model
    # ================================================================

    def simulate_step(self, x, y, yaw, v, w):
        """
        Simulate one step of differential-drive/unicycle motion.

        Inputs:
            x, y, yaw = current predicted state
            v         = linear velocity
            w         = angular velocity

        Returns:
            x_next, y_next, yaw_next
        """

        # Forward Euler integration.
        x_next = x + self.dt * v * math.cos(yaw)
        y_next = y + self.dt * v * math.sin(yaw)
        yaw_next = self.normalize_angle(yaw + self.dt * w)

        return x_next, y_next, yaw_next

    # ================================================================
    # MPC cost function
    # ================================================================

    def mpc_cost_function(self, u_flat):
        """
        This is the function scipy.optimize.minimize tries to minimize.

        u_flat is the candidate future control sequence:

            [v0, w0, v1, w1, ..., vN-1, wN-1]

        The function:
            1. Simulates the robot forward using u_flat.
            2. Computes cost at each predicted step.
            3. Returns one scalar total_cost.

        Lower cost = better future command sequence.
        """

        # Start prediction from current real robot state.
        x = self.x
        y = self.y
        yaw = self.yaw

        # Accumulated cost over prediction horizon.
        total_cost = 0.0

        # Start path search from current closest index.
        current_search_index = self.closest_index

        # Final goal is last path point.
        final_goal = self.path[-1]

        # Used to compute smoothness cost.
        previous_v = None
        previous_w = None

        # Loop through each future step.
        for k in range(self.horizon_steps):

            # Extract v and w for this step from the flat optimization vector.
            v = u_flat[2 * k]
            w = u_flat[2 * k + 1]

            # Simulate robot one step forward.
            x, y, yaw = self.simulate_step(x, y, yaw, v, w)

            # Find nearest path point to predicted pose.
            closest_index, path_distance = self.find_closest_index_for_pose(
                x,
                y,
                current_search_index
            )

            # Update search index so future steps search forward.
            current_search_index = closest_index

            # Get direction of path at closest point.
            path_yaw = self.get_path_heading(closest_index)

            # Heading error between predicted yaw and path yaw.
            heading_error = self.normalize_angle(yaw - path_yaw)

            # Distance from predicted pose to final goal.
            goal_distance = self.distance((x, y), final_goal)

            # Penalize distance from path.
            path_cost = self.path_error_weight * (path_distance ** 2)

            # Penalize heading misalignment.
            heading_cost = self.heading_error_weight * (heading_error ** 2)

            # Penalize being far from final goal.
            goal_cost = self.goal_distance_weight * goal_distance

            # Penalize large control inputs.
            control_cost = self.control_effort_weight * ((v ** 2) + (w ** 2))

            # Default smoothness cost is zero for first step.
            smoothness_cost = 0.0

            # From second step onward, penalize sudden changes in commands.
            if previous_v is not None:
                dv = v - previous_v
                dw = w - previous_w
                smoothness_cost = self.control_smoothness_weight * ((dv ** 2) + (dw ** 2))

            # Add all cost terms.
            total_cost += (
                path_cost
                + heading_cost
                + goal_cost
                + control_cost
                + smoothness_cost
            )

            # Store current controls for next step's smoothness cost.
            previous_v = v
            previous_w = w

        # Terminal cost encourages final predicted state to be near final goal.
        total_cost += self.terminal_goal_weight * self.distance((x, y), final_goal)

        return total_cost

    def solve_mpc(self):
        """
        Solve the MPC optimization problem.

        Returns:
            cmd_v
            cmd_w
            mpc_cost
            optimizer_success
            predicted_points
        """

        # Create bounds for every v and w in the horizon.
        bounds = []

        for _ in range(self.horizon_steps):
            # Bound for v.
            bounds.append((self.min_linear_speed, self.max_linear_speed))

            # Bound for w.
            bounds.append((-self.max_angular_speed, self.max_angular_speed))

        # Run nonlinear constrained optimization.
        result = minimize(
            self.mpc_cost_function,   # function to minimize
            self.u_prev,             # initial guess
            method="SLSQP",          # optimizer method
            bounds=bounds,           # velocity constraints
            options={
                "maxiter": 40,       # max optimizer iterations
                "ftol": 1e-3,        # convergence tolerance
                "disp": False        # don't print scipy optimizer output
            }
        )

        # If optimizer succeeded, use its solution.
        if result.success:
            u_solution = result.x

        # If optimizer failed, use previous guess as fallback.
        else:
            self.get_logger().warn(
                f"MPC optimization failed: {result.message}. Using previous command guess.",
                throttle_duration_sec=1.0
            )
            u_solution = self.u_prev

        # First control in optimized sequence is applied now.
        cmd_v = u_solution[0]
        cmd_w = u_solution[1]

        # Warm start next optimization by shifting solution forward.
        shifted = np.zeros_like(u_solution)

        # Drop first [v, w], shift rest forward.
        shifted[:-2] = u_solution[2:]

        # Repeat final control at end.
        shifted[-2:] = u_solution[-2:]

        # Store warm-start guess for next cycle.
        self.u_prev = shifted

        # Simulate predicted trajectory for RViz visualization.
        predicted_points = self.rollout_prediction(u_solution)

        return cmd_v, cmd_w, float(result.fun), bool(result.success), predicted_points

    def rollout_prediction(self, u_flat):
        """
        Simulate the optimized control sequence so we can visualize
        the predicted future trajectory in RViz.
        """

        # Start from current robot state.
        x = self.x
        y = self.y
        yaw = self.yaw

        predicted_points = []

        for k in range(self.horizon_steps):
            v = u_flat[2 * k]
            w = u_flat[2 * k + 1]

            x, y, yaw = self.simulate_step(x, y, yaw, v, w)

            predicted_points.append((x, y))

        return predicted_points

    # ================================================================
    # RViz markers
    # ================================================================

    def publish_path_marker(self):
        """
        Publish the planned path as a cyan line in RViz.
        """

        if not self.path:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mpc_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.04

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0

        for x, y in self.path:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.12
            marker.points.append(p)

        self.path_marker_pub.publish(marker)

    def publish_prediction_marker(self, predicted_points):
        """
        Publish the MPC predicted trajectory as an orange line in RViz.
        """

        if not predicted_points:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mpc_prediction"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.035

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.0

        for x, y in predicted_points:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.18
            marker.points.append(p)

        self.prediction_marker_pub.publish(marker)

    def publish_closest_marker(self, point):
        """
        Publish the closest path point as a yellow sphere in RViz.
        """

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mpc_closest_point"
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.16

        marker.scale.x = 0.16
        marker.scale.y = 0.16
        marker.scale.z = 0.16

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

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
        mpc_cost,
        cmd,
        optimizer_success
    ):
        """
        Log one row of MPC data to CSV.
        """

        time_sec = self.get_clock().now().nanoseconds / 1e9

        self.csv_writer.writerow([
            f"{time_sec:.4f}",
            "mpc",
            f"{self.x:.4f}",
            f"{self.y:.4f}",
            f"{self.yaw:.4f}",
            f"{closest_point[0]:.4f}",
            f"{closest_point[1]:.4f}",
            f"{path_heading:.4f}",
            f"{cross_track_error:.4f}",
            f"{heading_error:.4f}",
            f"{final_distance:.4f}",
            f"{mpc_cost:.4f}",
            f"{cmd.linear.x:.4f}",
            f"{cmd.angular.z:.4f}",
            str(optimizer_success)
        ])

        self.log_file.flush()

    # ================================================================
    # Main control loop
    # ================================================================

    def control_loop(self):
        """
        Main MPC loop.

        Runs every dt seconds.

        Steps:
            1. Publish path marker.
            2. Wait for odom and path.
            3. Check if goal is reached.
            4. Find closest path point.
            5. Compute tracking errors.
            6. Solve MPC optimization.
            7. Publish /cmd_vel.
            8. Publish prediction marker.
            9. Log data.
        """

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
            self.get_logger().info("MPC path complete.")
            return

        closest_index, _ = self.find_closest_index()
        closest_point = self.path[closest_index]
        self.publish_closest_marker(closest_point)

        path_heading = self.get_path_heading(closest_index)

        heading_error = self.normalize_angle(self.yaw - path_heading)

        cross_track_error = self.compute_signed_cross_track_error_for_pose(
            self.x,
            self.y,
            closest_index
        )

        cmd_v, cmd_w, mpc_cost, optimizer_success, predicted_points = self.solve_mpc()

        if final_distance < self.slowdown_distance:
            cmd_v = min(cmd_v, self.goal_linear_speed)

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

        self.publish_prediction_marker(predicted_points)

        self.log_data(
            closest_point,
            path_heading,
            cross_track_error,
            heading_error,
            final_distance,
            mpc_cost,
            cmd,
            optimizer_success
        )

        self.get_logger().info(
            f"idx={closest_index}/{len(self.path)} | "
            f"pose=({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f}) | "
            f"path_yaw={path_heading:.2f} | "
            f"cte={cross_track_error:.3f} m | "
            f"heading_err={heading_error:.2f} | "
            f"goal_dist={final_distance:.2f} m | "
            f"cost={mpc_cost:.2f} | "
            f"success={optimizer_success} | "
            f"cmd=({cmd.linear.x:.2f}, {cmd.angular.z:.2f})",
            throttle_duration_sec=1.0
        )


def main(args=None):
    """
    ROS 2 entry point.
    """

    rclpy.init(args=args)

    node = MPCController()

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