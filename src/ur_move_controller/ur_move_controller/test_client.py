#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ur_move_controller.action import MoveRobot


class TestClient(Node):
    def __init__(self):
        super().__init__('test_client')
        self.client = ActionClient(self, MoveRobot, 'move_robot')
    
    def send(self, ns, move_type, frame, pos, orient, vel):
        goal = MoveRobot.Goal()
        goal.robot_namespace = ns
        goal.movement_type = move_type
        goal.reference_frame = frame
        goal.position = pos
        goal.orientation = orient
        goal.velocity = vel
        
        self.client.wait_for_server()
        self.get_logger().info(f'Sending: {move_type} to {ns}')
        
        future = self.client.send_goal_async(goal, feedback_callback=self.feedback)
        future.add_done_callback(self.response)
    
    def feedback(self, msg):
        fb = msg.feedback
        self.get_logger().info(f'{fb.status}: {fb.progress:.0f}%')
    
    def response(self, future):
        handle = future.result()
        if handle.accepted:
            self.get_logger().info('Accepted')
            handle.get_result_async().add_done_callback(self.result)
        else:
            self.get_logger().error('Rejected')
            rclpy.shutdown()
    
    def result(self, future):
        res = future.result().result
        if res.success:
            self.get_logger().info(f'SUCCESS in {res.time:.2f}s')
        else:
            self.get_logger().error(f'FAILED: {res.message}')
        rclpy.shutdown()


def main():
    rclpy.init()
    client = TestClient()
    
    # Test: Robot1 MoveJ
    client.send(
        ns="robot1",
        move_type="moveJ",
        frame="base_link",
        pos=[0.3, 0.2, 0.4],
        orient=[0.0, 0.0, 0.0],
        vel=500.0
    )
    
    rclpy.spin(client)


if __name__ == '__main__':
    main()