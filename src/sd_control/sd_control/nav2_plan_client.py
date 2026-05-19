#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose


class Nav2PlanClient(Node):
    def __init__(self):
        super().__init__("nav2_plan_client")

        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_callback,
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            "/planned_path",
            10
        )

        self.action_client = ActionClient(
            self,
            ComputePathToPose,
            "/compute_path_to_pose"
        )

        self.get_logger().info("Nav2 planner client started.")
        self.get_logger().info("Use RViz 2D Goal Pose to publish /goal_pose.")
        self.get_logger().info("Waiting for /compute_path_to_pose action server...")

    def goal_callback(self, goal_msg):
        self.get_logger().info(
            f"Received goal: x={goal_msg.pose.position.x:.2f}, "
            f"y={goal_msg.pose.position.y:.2f}, frame={goal_msg.header.frame_id}"
        )

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Planner action server /compute_path_to_pose not available.")
            return

        goal = ComputePathToPose.Goal()
        goal.goal = goal_msg
        goal.use_start = False

        self.get_logger().info("Sending goal to Nav2 planner...")

        send_future = self.action_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Planner rejected the goal.")
            return

        self.get_logger().info("Planner accepted goal. Waiting for path result...")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        path = result.path

        if len(path.poses) == 0:
            self.get_logger().error("Planner returned empty path.")
            return

        self.path_pub.publish(path)

        self.get_logger().info(
            f"Published planned path with {len(path.poses)} poses on /planned_path."
        )


def main(args=None):
    rclpy.init(args=args)
    node = Nav2PlanClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()