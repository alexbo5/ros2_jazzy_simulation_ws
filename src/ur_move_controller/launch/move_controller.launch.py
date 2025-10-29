from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
import subprocess
import time


def generate_launch_description():
    
    # Warte-Aktion: Gibt der Simulation Zeit zu starten
    wait_action = ExecuteProcess(
        cmd=['sleep', '2'],
        output='screen'
    )
    
    # Action Server mit robot_description vom Topic
    # Nutze ros2 param um den Wert zu holen und zu setzen
    action_server_node = Node(
        package='ur_move_controller',
        executable='action_server',
        name='ur_move_controller',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description_topic': '/ur3e_1/robot_description'},
        ],
    )
    
    return LaunchDescription([
        wait_action,
        action_server_node,
    ])