#!/usr/bin/env python3
# Tells Linux to run the file using Python 3 if executed directly.

import math

import rclpy # Imports ROS 2 Python tools.
from rclpy.node import Node

from nav_msgs.msg import Odometry # Imports Odometry message
from geometry_msgs.msg import Twist # Imports Twist message


class WaypointFollower(Node): # Defines ROS2 node class type called WaypointFollower, Inherits from Node
    def __init__(self): # Initializes ROS2 Node and names it /waypoint_follower_controller.
        super().__init__("waypoint_follower_controller") 

        # Waypoints in odom frame, list of target points.
        self.waypoints = [
            (-1.0, 0.0),
            (-1.0, -1.0),
            (0.0, -1.0),
            (0.0, 0.0),
        ]

        self.current_waypoint_index = 0 # Tracks which waypoint the robot is following currently. Initially 0, the active waypoint is self.waypoints[0].

        # Controller gains
        self.k_linear = 0.5
        # linear.x = k_linear × distance_error
        self.k_angular = 1.8
        # angular.z = k_angular × heading_error

        # Speed limits
        self.max_linear_speed = 0.25
        self.max_angular_speed = 0.8

        # Tolerances.
        self.distance_tolerance = 0.10
        self.heading_tolerance = 0.15

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False
        self.finished = False

        # Odometry subscriber.
        self.odom_sub = self.create_subscription( # Subscriber
            Odometry, # Message Type.
            "/odom", # Topic it subscribes to. 
            self.odom_callback, # Callback function. 
            10 # Queue Size.
        )
        
        # Velocity publisher.
        self.cmd_pub = self.create_publisher( # Publisher
            Twist, # Message Type.
            "/cmd_vel", # Topic to publish on.
            10 # Queue Size.
        )
        
        # Runs the control loop every 0.05 seconds.
        self.timer = self.create_timer(0.05, self.control_loop) 

        # Prints controller startup information.
        self.get_logger().info("Waypoint follower started.")
        self.get_logger().info(f"Waypoints: {self.waypoints}")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x # Robot position x
        self.y = msg.pose.pose.position.y # RObot position y

        q = msg.pose.pose.orientation # Obtains robot orientation quaternion

        # Converts quaternion into yaw angle, the robot's heading in radians.
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y) 
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.odom_received = True # Marks odometry as valid. 

    def normalize_angle(self, angle):
        # Keeps the angles within the range -pi to pi
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def clamp(self, value, min_value, max_value):
        # Limits velocity command values to within a minimum and maximum
        return max(min(value, max_value), min_value)

    def publish_stop(self):
        # Publishes zero velocity to stop the robot.
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        # Main controller, runs every 0.05 seconds.
        if not self.odom_received: # Checks if odometry is received, if not, does nothing 
            return

        if self.finished: # If all waypoints are completed, begin publishing stop commands.  
            self.publish_stop()
            return

        # Selects the current waypoint
        goal_x, goal_y = self.waypoints[self.current_waypoint_index] 

        # Vector from the current robot position to the current waypoint.
        dx = goal_x - self.x
        dy = goal_y - self.y
        
        # Distance from robot to current waypoint.
        distance_error = math.sqrt(dx * dx + dy * dy)
        # Angle required from robot to the waypoint.
        desired_heading = math.atan2(dy, dx)
        # How much the robot needs to turn to face the waypoint. 
        heading_error = self.normalize_angle(desired_heading - self.yaw)

        # If the distance error, i.e if the robot is within 10 cm of the waypoint, consider it reached. 
        if distance_error < self.distance_tolerance:
            self.get_logger().info( # Logs the message that it has reached the waypoint
                f"Reached waypoint {self.current_waypoint_index + 1}: "
                f"({goal_x:.2f}, {goal_y:.2f})"
            )
        
        # Updates the waypoint, moves to the next waypoint.
            self.current_waypoint_index += 1 

        # If the current waypoint index is greater than the length of the waypoints, publish stop messages and log a completed message.
            if self.current_waypoint_index >= len(self.waypoints):
                self.finished = True
                self.publish_stop()
                self.get_logger().info("All waypoints complete.")
                return

            return # Return after reaching waypoint. 
        
        # 
        cmd = Twist() # Creates a new velocity command

        if abs(heading_error) > self.heading_tolerance:
        # If the robot is not facing the waypoint, rotate in place. 
            cmd.linear.x = 0.0
            cmd.angular.z = self.k_angular * heading_error
        else:
        # If the robot is facing the waypoint, move forward while correcting for the heading.
            cmd.linear.x = self.k_linear * distance_error
            cmd.angular.z = self.k_angular * heading_error

        # Clamps linear speed.
        cmd.linear.x = self.clamp(
            cmd.linear.x,
            0.0,
            self.max_linear_speed
        )

        # Clamps angular speed.
        cmd.angular.z = self.clamp(
            cmd.angular.z,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        # Publishes the velocity command
        self.cmd_pub.publish(cmd)
        
        self.get_logger().info(
            "\n"
            "================ Way Point Follower DEBUG ================\n"
            f"{'Way Point':<18}: {self.current_waypoint_index + 1}/{len(self.waypoints)}\n"
            f"{'Robot Pose':<18}: x={self.x:>8.2f}, y={self.y:>8.2f}, yaw={self.yaw:>8.2f}\n"
            f"{'Goal Distance':<18}: x={goal_x:>8.2f}, y={goal_y:>8.2f}\n"
            f"{'Heading Error':<18}: {heading_error:>8.3f} m\n"
            f"{'Distance Error':<18}: {distance_error:>8.2f} m\n"
            f"{'Command':<18}: linear.x={cmd.linear.x:>8.2f}, angular.z={cmd.angular.z:>8.2f}\n"
            "====================================================\n\n",
            throttle_duration_sec=1.0
            )


def main(args=None): # Entry point for the node.
    rclpy.init(args=args) # Initializes ROS 2.
    node = WaypointFollower() # Creates Node.

    try: # Keep the node running. 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Once the node is shutdown
    # Publish stop
    # Destroy node
    # Shutdowm
    node.publish_stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()