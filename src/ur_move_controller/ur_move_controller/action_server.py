#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped, Pose
from ur_move_controller.action import MoveRobot
from scipy.spatial.transform import Rotation
from rcl_interfaces.srv import GetParameters
import time


class URMoveController(Node):
    def __init__(self):
        super().__init__('ur_move_controller')
        
        self.get_logger().info("Loading robot_description from parameter server...")
        
        # Hole robot_description von robot_state_publisher
        robot_desc = self.get_robot_description_from_param()
        
        if robot_desc is None:
            self.get_logger().error("Failed to get robot_description!")
            raise RuntimeError("No robot_description available")
        
        self.get_logger().info(f"✓ robot_description loaded ({len(robot_desc)} chars)")
        
        # Declare robot_description parameter für MoveIt
        self.declare_parameter('robot_description', robot_desc)
        
        # MoveItPy initialisieren
        try:
            self.moveit = MoveItPy(node_name="moveit_node")
            self.robot_model = self.moveit.get_robot_model()
            self.planning_components = {}
            
            # Namespace Mapping
            self.ns_map = {"robot1": "ur3e_1", "robot2": "ur3e_2"}
            
            # Action Server
            self._action_server = ActionServer(
                self,
                MoveRobot,
                'move_robot',
                execute_callback=self.execute,
                callback_group=ReentrantCallbackGroup()
            )
            
            self.get_logger().info("✓ UR Move Controller ready!")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MoveIt: {e}")
            import traceback
            traceback.print_exc()
            raise
    
    def get_robot_description_from_param(self):
        """Hole robot_description von robot_state_publisher Parameter"""
        # Versuche verschiedene mögliche Quellen
        sources = [
            '/ur3e_1/robot_state_publisher',
            '/robot_state_publisher',
            '/ur3e_1',
        ]
        
        for source in sources:
            self.get_logger().info(f"Trying to get robot_description from {source}...")
            
            try:
                # Erstelle Client für GetParameters Service
                client = self.create_client(GetParameters, f'{source}/get_parameters')
                
                if not client.wait_for_service(timeout_sec=5.0):
                    self.get_logger().warn(f"Service {source}/get_parameters not available")
                    continue
                
                # Request erstellen
                request = GetParameters.Request()
                request.names = ['robot_description']
                
                # Call service
                future = client.call_async(request)
                
                # Warte auf Antwort
                timeout = 10.0
                start = time.time()
                while not future.done() and (time.time() - start) < timeout:
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                if future.done():
                    response = future.result()
                    if response and len(response.values) > 0:
                        robot_desc = response.values[0].string_value
                        if robot_desc and len(robot_desc) > 100:
                            self.get_logger().info(f"✓ Found robot_description at {source}")
                            return robot_desc
                
            except Exception as e:
                self.get_logger().warn(f"Failed to get from {source}: {e}")
                continue
        
        return None
    
    def get_prefix(self, ns):
        """Konvertiere robot1/robot2 zu ur3e_1/ur3e_2"""
        return self.ns_map.get(ns, ns)
    
    def get_planner(self, ns):
        """Hole Planning Component"""
        prefix = self.get_prefix(ns)
        group = f"{prefix}_arm"
        
        if group not in self.planning_components:
            self.planning_components[group] = \
                self.moveit.get_planning_component(group)
        
        return self.planning_components[group]
    
    def get_frame(self, ref, ns):
        """Verarbeite Reference Frame"""
        if ref == "world":
            return "world"
        elif ref == "base_link":
            return f"{self.get_prefix(ns)}/base_link"
        else:
            return ref
    
    def to_quat(self, rpy):
        """Euler zu Quaternion"""
        r = Rotation.from_euler('xyz', rpy)
        return r.as_quat()
    
    def execute(self, goal_handle):
        """Action Execute Callback"""
        req = goal_handle.request
        self.get_logger().info(
            f"{req.movement_type}: {req.robot_namespace} -> "
            f"{req.position} @ {req.velocity}mm/s"
        )
        
        # Feedback
        feedback = MoveRobot.Feedback()
        feedback.status = "planning"
        feedback.progress = 0.0
        goal_handle.publish_feedback(feedback)
        
        # Setup
        planner = self.get_planner(req.robot_namespace)
        frame = self.get_frame(req.reference_frame, req.robot_namespace)
        prefix = self.get_prefix(req.robot_namespace)
        
        # Pose erstellen
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = req.position
        quat = self.to_quat(req.orientation)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
        
        # Velocity Scaling
        vel_scale = min(req.velocity / 1000.0, 1.0)
        
        start_time = time.time()
        success = False
        
        try:
            planner.set_start_state_to_current_state()
            
            if req.movement_type.lower() == "movej":
                # MoveJ
                pose_goal = PoseStamped()
                pose_goal.header.frame_id = frame
                pose_goal.pose = pose
                planner.set_goal_state(
                    pose_stamped_msg=pose_goal,
                    pose_link=f"{prefix}/tool0"
                )
                
                planner.set_plan_request_parameters(
                    {
                        'planning_attempts': 10,
                        'planning_time': 5.0,
                        'max_velocity_scaling_factor': vel_scale,
                        'max_acceleration_scaling_factor': vel_scale * 0.8
                    }
                )
                
                feedback.status = "planning"
                goal_handle.publish_feedback(feedback)
                
                plan = planner.plan()
                if plan:
                    feedback.status = "executing"
                    feedback.progress = 50.0
                    goal_handle.publish_feedback(feedback)
                    
                    self.moveit.execute(plan.trajectory, controllers=[])
                    success = True
            
            elif req.movement_type.lower() == "movel":
                # MoveL
                robot_state = self.moveit.get_planning_scene_monitor().get_current_state()
                
                fraction, traj = robot_state.computeCartesianPath(
                    f"{prefix}_arm",
                    f"{prefix}/tool0",
                    [pose],
                    max_step=0.005,
                    jump_threshold=0.0,
                    reference_frame=frame
                )
                
                if fraction >= 0.95:
                    feedback.status = "executing"
                    feedback.progress = 50.0
                    goal_handle.publish_feedback(feedback)
                    
                    from moveit.core.robot_trajectory import RobotTrajectory
                    from moveit.planning import IterativeParabolicTimeParameterization
                    
                    rt = RobotTrajectory(self.robot_model, f"{prefix}_arm")
                    rt.set_robot_trajectory_msg(robot_state, traj)
                    
                    time_param = IterativeParabolicTimeParameterization()
                    time_param.compute_time_stamps(rt, vel_scale, vel_scale * 0.8)
                    
                    self.moveit.execute(rt.get_robot_trajectory_msg(), controllers=[])
                    success = True
                else:
                    self.get_logger().error(f"Cartesian path only {fraction*100:.1f}%")
        
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            import traceback
            traceback.print_exc()
        
        # Result
        result = MoveRobot.Result()
        result.success = success
        result.time = time.time() - start_time
        result.message = "Success" if success else "Failed"
        
        if success:
            feedback.status = "completed"
            feedback.progress = 100.0
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
        else:
            goal_handle.abort()
        
        return result


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = URMoveController()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()