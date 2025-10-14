from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    # Hardcoded values for Robot 1
    ur_type_1 = "ur3e"
    safety_limits_1 = "true"
    safety_pos_margin_1 = "0.15"
    safety_k_position_1 = "20"
    tf_prefix_1 = "ur_robot1_"              # MIT Underscore für TF-Frames
    robot_name_1 = "ur_robot1"               # OHNE Underscore für Namespace
    initial_joint_controller_1 = "joint_trajectory_controller"
    
    # Hardcoded values for Robot 2
    ur_type_2 = "ur3e"
    safety_limits_2 = "true"
    safety_pos_margin_2 = "0.15"
    safety_k_position_2 = "20"
    tf_prefix_2 = "ur_robot2_"              # MIT Underscore für TF-Frames
    robot_name_2 = "ur_robot2"               # OHNE Underscore für Namespace
    initial_joint_controller_2 = "joint_trajectory_controller"
    
    # Common values
    world_file = "empty.sdf"


    # Dynamic paths (not hardcoded)
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("ur_simulation_gz"), "config", "ur_controllers.yaml"]
    )
    description_file = PathJoinSubstitution(
        [FindPackageShare("ur_simulation_gz"), "urdf", "ur_gz.urdf.xacro"]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
    )


    # ========== Robot 1 ==========
    robot_description_content_1 = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            "safety_limits:=",
            safety_limits_1,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin_1,
            " ",
            "safety_k_position:=",
            safety_k_position_1,
            " ",
            "name:=",
            robot_name_1,
            " ",
            "ur_type:=",
            ur_type_1,
            " ",
            "tf_prefix:=",
            tf_prefix_1,
            " ",
            "simulation_controllers:=",
            controllers_file,
            " ",
            "ros_namespace:=",
            robot_name_1,
        ]
    )
    robot_description_1 = {"robot_description": robot_description_content_1}


    robot_state_publisher_node_1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True, "frame_prefix": tf_prefix_1}, robot_description_1],
        namespace=robot_name_1,
    )


    joint_state_broadcaster_spawner_1 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            f"/{robot_name_1}/controller_manager"
        ],
    )


    initial_joint_controller_spawner_1 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller_1,
            "-c",
            f"/{robot_name_1}/controller_manager"
        ],
    )


    gz_spawn_entity_1 = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content_1,
            "-name",
            robot_name_1,
            "-allow_renaming",
            "true",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
    )


    # ========== Robot 2 ==========
    robot_description_content_2 = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            "safety_limits:=",
            safety_limits_2,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin_2,
            " ",
            "safety_k_position:=",
            safety_k_position_2,
            " ",
            "name:=",
            robot_name_2,
            " ",
            "ur_type:=",
            ur_type_2,
            " ",
            "tf_prefix:=",
            tf_prefix_2,
            " ",
            "simulation_controllers:=",
            controllers_file,
            " ",
            "ros_namespace:=",
            robot_name_2,
        ]
    )
    robot_description_2 = {"robot_description": robot_description_content_2}


    robot_state_publisher_node_2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True, "frame_prefix": tf_prefix_2}, robot_description_2],
        namespace=robot_name_2,
    )


    joint_state_broadcaster_spawner_2 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            f"/{robot_name_2}/controller_manager"
        ],
    )


    initial_joint_controller_spawner_2 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller_2,
            "-c",
            f"/{robot_name_2}/controller_manager"
        ],
    )


    gz_spawn_entity_2 = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content_2,
            "-name",
            robot_name_2,
            "-allow_renaming",
            "true",
            "-x", "2.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
    )


    # ========== Common Nodes ==========
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )


    # Delay rviz start after both joint_state_broadcasters
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_2,
            on_exit=[rviz_node],
        ),
    )


    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": " -r -v 4 " + world_file
        }.items(),
    )


    # Make the /clock topic available in ROS
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )


    nodes_to_start = [
        gz_launch_description,
        gz_sim_bridge,
        # Robot 1
        robot_state_publisher_node_1,
        gz_spawn_entity_1,
        joint_state_broadcaster_spawner_1,
        initial_joint_controller_spawner_1,
        # Robot 2
        robot_state_publisher_node_2,
        gz_spawn_entity_2,
        joint_state_broadcaster_spawner_2,
        initial_joint_controller_spawner_2,
        # Common
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]


    return LaunchDescription(nodes_to_start)