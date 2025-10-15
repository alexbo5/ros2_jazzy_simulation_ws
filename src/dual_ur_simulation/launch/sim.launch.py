from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Launch-Argumente
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='ur3e_1',
        description='Name des Roboters (für Namespace und Frame-Prefix)'
    )
    
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur3e',
        description='UR-Roboter Typ (ur3, ur5, ur10, ur3e, ur5e, ur10e, ur16e, etc.)'
    )
    
    robot_x_arg = DeclareLaunchArgument(
        'robot_x',
        default_value='0.0',
        description='X-Position des Roboters'
    )
    
    robot_y_arg = DeclareLaunchArgument(
        'robot_y',
        default_value='0.0',
        description='Y-Position des Roboters'
    )
    
    # Launch-Konfigurationen
    robot_name = LaunchConfiguration('robot_name')
    ur_type = LaunchConfiguration('ur_type')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    
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

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        arguments=[
            '-name', robot_name,
            '-topic', [robot_name, '/robot_description'],
            '-x', robot_x,
            '-y', robot_y,
            '-z', '0.0'
        ],
        output='screen'
    )
    
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
                'simulation_controllers:=', controllers_config, ' ',
                'ros_namespace:=', robot_name
            ])
        }],
        output='screen'
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', 
            ['/', robot_name, '/controller_manager'],
            '--param-file', controllers_config
        ],
        output='screen'
    )

    # Delay trajectory controller until joint_state_broadcaster is loaded
    delay_joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'joint_trajectory_controller',
                        '--controller-manager', 
                        ['/', robot_name, '/controller_manager'],
                        '--param-file', controllers_config,
                    ],
                    output='screen'
                )
            ]
        )
    )

    return LaunchDescription([
        robot_name_arg,
        ur_type_arg,
        robot_x_arg,
        robot_y_arg,
        gz_sim,
        gz_sim_bridge,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster,
        delay_joint_trajectory_controller
    ])