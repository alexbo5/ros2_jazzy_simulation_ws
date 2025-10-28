from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler, GroupAction
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    
    # Roboter-Konfigurationen
    robots = [
        {
            'name': 'ur3e_1', 
            'type': 'ur3e', 
            'origin_xyz': '0.0 0.0 0.0',
            'origin_rpy': '0 0 0'
        },
        {
            'name': 'ur3e_2', 
            'type': 'ur3e', 
            'origin_xyz': '0.5 0.5 0.0',
            'origin_rpy': '0 0 3.14'
        },
    ]
    
    # Package-Pfade
    dual_ur_simulation_package = FindPackageShare('dual_ur_simulation')
    urdf_file = PathJoinSubstitution([
        dual_ur_simulation_package,
        'urdf',
        'ur.urdf.xacro'
    ])
    
    # Load controllers configuration
    controllers_config = PathJoinSubstitution([
        dual_ur_simulation_package,
        'config',
        'controllers.yaml'
    ])

    # Gazebo Simulation mit Bullet-Featherstone Physics Engine starten
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': '-r empty.sdf --physics-engine gz-physics-bullet-featherstone-plugin'
        }.items()
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    # RViz2 starten
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            dual_ur_simulation_package,
            'rviz',
            'dual_ur.rviz'
        ])],
    )

    # Nodes für jeden Roboter erstellen
    robot_nodes = []
    for robot in robots:
        robot_name = robot['name']
        ur_type = robot['type']
        origin_xyz = robot['origin_xyz']
        origin_rpy = robot['origin_rpy']
        
        # Robot State Publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_name,
            parameters=[{
                'robot_description': Command([
                    FindExecutable(name='xacro'), ' ',
                    urdf_file, ' ',
                    'name:=', robot_name, ' ',
                    'ur_type:=', ur_type, ' ',
                    'tf_prefix:=', robot_name, '/', ' ',
                    'origin_xyz:="', origin_xyz, '" ',
                    'origin_rpy:="', origin_rpy, '" ',
                    'simulation_controllers:=', controllers_config, ' ',
                    'ros_namespace:=', robot_name
                ])
            }],
            output='screen'
        )

        # Spawn Entity
        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{robot_name}',
            arguments=[
                '-name', robot_name,
                '-topic', f'/{robot_name}/robot_description'
            ],
            output='screen'
        )

        # Joint State Broadcaster
        joint_state_broadcaster = Node(
            package='controller_manager',
            executable='spawner',
            name=f'joint_state_broadcaster_spawner_{robot_name}',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', 
                f'/{robot_name}/controller_manager',
                '--param-file', controllers_config
            ],
            output='screen'
        )

        # Trajectory Controller (verzögert nach Joint State Broadcaster)
        delay_joint_trajectory_controller = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        name=f'joint_trajectory_controller_spawner_{robot_name}',
                        arguments=[
                            'joint_trajectory_controller',
                            '--controller-manager', 
                            f'/{robot_name}/controller_manager',
                            '--param-file', controllers_config,
                        ],
                        output='screen'
                    )
                ]
            )
        )


        # Greifer Controller spawnen (verzögert nach Trajectory Controller)
        delay_gripper_controller = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        name=f'gripper_controller_spawner_{robot_name}',
                        arguments=[
                            'robotiq_gripper_controller',
                            '--controller-manager', 
                            f'/{robot_name}/controller_manager',
                            '--param-file', controllers_config,
                        ],
                        output='screen'
                    )
                ]
            )
        )


        # Gruppiere Nodes pro Roboter
        robot_group = GroupAction([
            robot_state_publisher,
            spawn_entity,
            joint_state_broadcaster,
            delay_joint_trajectory_controller,
            delay_gripper_controller  # Greifer Controller hinzugefügt
        ])
        
        robot_nodes.append(robot_group)

    return LaunchDescription([
        gz_sim,
        gz_sim_bridge,
        rviz_node,
        *robot_nodes
    ])
