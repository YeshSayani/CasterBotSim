#!/usr/bin/env python3 
# Tells Linux to run the file using Python 3 if you execute it directly.

import math # Imports math functions.

import rclpy # rclpy initializes and runs the ROS 2 node. 
from rclpy.node import Node # Node is the base class controller inherits from.

from nav_msgs.msg import Odometry # Import ROS Message types, Odometry is used for /odom.
from geometry_msgs.msg import Twist # Imports ROS Message Twist, used in /cmd_vel.


class GoToGoalController(Node): # Defines Controller Node Class. means it can create ROS publishers, subscribers, timers, and logs.
    def __init__(self):
        super().__init__("go_to_goal_controller") # This initializes the ROS node and names it: /go_to_goal_controller

        # Fixed test goal in odom frame
        self.goal_x = 1.5
        self.goal_y = 0.0

        # Controller gains
        self.k_linear = 0.5
        self.k_angular = 1.5

        # Speed limits
        self.max_linear_speed = 0.25
        self.max_angular_speed = 0.8

        # Stop conditions
        self.distance_tolerance = 0.08
        self.heading_tolerance = 0.15

        # Robot state variables from odom
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False
        self.goal_reached = False

        self.odom_sub = self.create_subscription(
            Odometry, # Message Type
            "/odom", # Topic to subscribe to 
            self.odom_callback, # Call back function    
            10 # Queue Size
        )

        self.cmd_pub = self.create_publisher( 
            Twist, # Twist Message Type
            "/cmd_vel", # Topic to publish to
            10 # Queue Size
        )

        self.timer = self.create_timer(0.05, self.control_loop) # Runs the control loop every 0.05

        # Prints start up information and goal information
        self.get_logger().info("Go-to-goal controller started.")
        self.get_logger().info(f"Goal: x={self.goal_x:.2f}, y={self.goal_y:.2f}")

    def odom_callback(self, msg): # function runs whenever a new /odom message arrives.
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        #ROS does not usually store orientation directly as yaw angle. It stores it as a quaternion:
        q = msg.pose.pose.orientation # This gets the robot orientation quaternion. 

        # Convert quaternion to yaw manually.
        # yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.odom_received = True

    def normalize_angle(self, angle): # forces an angle into the range: -pi to +pi
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def clamp(self, value, min_value, max_value): # Limits a value between a minimum and maximum.
        return max(min(value, max_value), min_value)

    def publish_stop(self): # Publishes Empty Twist message to stop the robot.
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def control_loop(self): # runs every 0.05 seconds.
        if not self.odom_received: # If no odometry has been received yet, do nothing.
            return

        if self.goal_reached: # If the robot already reached the goal, keep publishing stop commands.
            self.publish_stop()
            return

        # This computes the vector from the robot to the goal.
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        distance_error = math.sqrt(dx * dx + dy * dy) # This computes Euclidean distance to the goal.
        desired_heading = math.atan2(dy, dx) # Computes the angle from the robot to the goal.
        # Computes how far the robot must rotate to face the goal.
        heading_error = self.normalize_angle(desired_heading - self.yaw) # Normalizes the result to [-pi, pi].

        cmd = Twist() # Creates a new velocity command

        # If the robot is within 8 cm of the goal, stop.
        # Sets the self.goal_reached = True
        # Then publishes zero velocity and prints final pose.
        # Controller does not care about final yaw. It only cares about reaching the position.
        if distance_error < self.distance_tolerance: 
            self.goal_reached = True
            self.publish_stop()
            self.get_logger().info(
                f"Goal reached. Final pose: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}"
            )
            return

        # First rotate toward the goal if heading error is large.
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
            f"x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}, "
            f"dist_err={distance_error:.2f}, heading_err={heading_error:.2f}, "
            f"cmd_v={cmd.linear.x:.2f}, cmd_w={cmd.angular.z:.2f}",
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoalController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.publish_stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()