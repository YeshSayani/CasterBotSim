#!/usr/bin/env python3

# Bridge between Rviz goal clicks and Nav2 global panning
# RViz 2D Goal Pose -> publishes /goal_pose -> nav2_plan_client.py receives goal -> calls Nav2 /compute_path_to_pose action -> 
# -> planner_server computes path -> nav2_plan_client.py publishes path on /planned_path -> custom controller follows /planned_path

import rclpy # Imports the ROS 2 Python client library. Needs rclpy to create ROS2 nodes, publishers, subscribers, action clients, timers, logging, spinning etc.
from rclpy.node import Node # Imports the base Node class.
from rclpy.action import ActionClient # Imports the ROS 2 action client class.

from geometry_msgs.msg import PoseStamped # Imports PoseStamped, meaning A pose with a coordinate frame and timestamp.
from nav_msgs.msg import Path # Imports the Path message, basically a list of poses. 
from nav2_msgs.action import ComputePathToPose # Imports the Nav2 action type for computing a path to a goal pose.


class Nav2PlanClient(Node): # Custom ROS2 node class that inherits from Node, so has access to ROS2 node features such as publishers, subscribers, timers, parameters etc.
    def __init__(self): # Constructor, runs once when node is created. 
        super().__init__("nav2_plan_client") # Calls the constructor of the parent class, which is of type Node. 
        # Initializes the object as a real ROS 2 node and gives it the node name "nav2_plan_client".

        self.goal_sub = self.create_subscription(  # Goal subscriber
            PoseStamped, # Message type
            "/goal_pose", # Listens to /goal_pose 
            self.goal_callback, #call back function 
            10 # Queue Size
        )

        self.path_pub = self.create_publisher( # Creates a publisher
            Path, # Message type
            "/planned_path", # Topic to publish on 
            10 # Queue Size
        )

        self.action_client = ActionClient( # Creates a client for the action (Nav2 planner)
            # Connects to the action server /compute_path_to_pose
            # The action type is: nav2_msgs/action/ComputePathToPose
            # This action server is provided by: /planner_server
            self,
            ComputePathToPose,
            "/compute_path_to_pose"
        )

        # These print information to the terminal.
        self.get_logger().info("Nav2 planner client started.")
        self.get_logger().info("Use RViz 2D Goal Pose to publish /goal_pose.")
        self.get_logger().info("Waiting for /compute_path_to_pose action server...")

    def goal_callback(self, goal_msg): # Function runs whenever a message arrives on: /goal_pose, incoming message is stored in goal_msg.
        self.get_logger().info( # This logs the received goal.
            f"Received goal: x={goal_msg.pose.position.x:.2f}, "
            f"y={goal_msg.pose.position.y:.2f}, frame={goal_msg.header.frame_id}"
        )

        if not self.action_client.wait_for_server(timeout_sec=5.0): # Before sending a planning request, the node waits up to 5 seconds for: /compute_path_to_pose
            self.get_logger().error("Planner action server /compute_path_to_pose not available.") # If the action server is not available, it logs an error and exits the callback.
            return

        goal = ComputePathToPose.Goal() # This creates a new planning request object.
        goal.goal = goal_msg # Sets the target pose 
        #Do not use a manually supplied start pose.
        #Use the robot's current pose.
        goal.use_start = False # ComputePathToPose can accept either: a provided start pose or the robot's current pose from TF

        self.get_logger().info("Sending goal to Nav2 planner...") # Logs that it is sending the request.

        send_future = self.action_client.send_goal_async(goal) # This sends the planning request asynchronously.
        send_future.add_done_callback(self.goal_response_callback) # When Nav2 responds saying whether it accepted/rejected the goal, call goal_response_callback.

    def goal_response_callback(self, future): # This function runs when the planner action server responds to the goal request.
        goal_handle = future.result() # This extracts the goal handle. The goal handle indicates whether the action server accepted the goal.

        if not goal_handle.accepted: # If the planner rejects the goal, log an error and stop.
            self.get_logger().error("Planner rejected the goal.")
            return

        self.get_logger().info("Planner accepted goal. Waiting for path result...") # If accepted, output the planner is now computing the path.

        result_future = goal_handle.get_result_async() # This asks for the final action result asynchronously.
        result_future.add_done_callback(self.result_callback) # When the final result is ready, call: self.result_callback

    def result_callback(self, future): # Runs when Nav2 finishes computing the path.
        result = future.result().result # Extracts the action result.
        path = result.path # This gets the computed path from the Nav2 result.

        if len(path.poses) == 0: # If the planner returns no poses, there is no valid path.
            self.get_logger().error("Planner returned empty path.")
            return

        self.path_pub.publish(path) # This publishes the path on: /planned_path

        self.get_logger().info(
            f"Published planned path with {len(path.poses)} poses on /planned_path." # Logs how many poses are in the path.
        )


def main(args=None): # This is the function called when the executable runs.
    rclpy.init(args=args) # Initializes ROS 2 Python.
    node = Nav2PlanClient() # Creates the node

    try:
        rclpy.spin(node) # Keep processing callbacks until shutdown.
    except KeyboardInterrupt: # The KeyboardInterrupt catches Ctrl+C so the node can shut down cleanly.
        pass

    node.destroy_node() # Clean shutdown. Destroy the node and shut down ROS 2 Python.
    rclpy.shutdown()


if __name__ == "__main__": # This allows the file to be run directly with Python: python3 nav2_plan_client.py
    main()