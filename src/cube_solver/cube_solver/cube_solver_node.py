#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from cube_perception.action import ScanCube
# todo: from cube_motion_interfaces.action import ExecuteMove

class CubeSolver(Node):
    def __init__(self):
        super().__init__('cube_solver')
        self.perception_client = ActionClient(self, ScanCube, 'scan_cube')
#        self.motion_client = self.create_client(ExecuteMove, 'execute_cube_move')

    def run(self):
        self.get_logger().info("Main started...")
        # wait for action server (blocking)
        if not self.perception_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Perception action server not available")
            return

        goal = ScanCube.Goal()
        #goal.camera_index = 1
        send_goal_future = self.perception_client.send_goal_async(goal)
        # block until send_goal completes
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if goal_handle is None or not getattr(goal_handle, "accepted", False):
            self.get_logger().error("Perception goal was rejected or no goal handle received")
            return

        self.get_logger().info("Perception goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result()
        if res is None:
            self.get_logger().error("Failed to get result from perception server")
            return

        solution = res.result.solution.split()
        for move in solution:
            # todo: self.motion_client ...
            self.get_logger().info(move)

        self.get_logger().info("Cube solved!")

def main(args=None):
    rclpy.init(args=args)
    node = CubeSolver()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()