import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    pkg_name = 'simple_diff_drive_sim'

    # Get package share directory
    pkg_share = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'test_world.sdf')

    # Get launch configurations
    enable_obstacle = LaunchConfiguration('enable_obstacle').perform(context)
    obstacle_seed = LaunchConfiguration('obstacle_seed').perform(context)

    # Robot State Publisher
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Gazebo Sim launch
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': '-r ' + world_file}.items(),
    )

    # Spawn robot entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_bot', '-x', '-4.0', '-z', '0.3'],
        output='screen',
    )

    # ROS - Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/depth_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        output='screen',
    )

    nodes = [
        node_robot_state_publisher,
        launch_gazebo,
        spawn_entity,
        bridge,
    ]

    # Add obstacle if enabled
    if enable_obstacle.lower() == 'true':
        obstacle_model_path = os.path.join(
            pkg_share, 'models', 'moving_obstacle', 'model.sdf'
        )

        # Spawn obstacle model
        spawn_obstacle = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file',
                obstacle_model_path,
                '-name',
                'moving_obstacle',
                '-x',
                '-2.0',
                '-y',
                '0.0',
                '-z',
                '0.0',
            ],
            output='screen',
        )

        # Random walk controller node
        random_walk_node = Node(
            package='simple_diff_drive_sim',
            executable='random_walk_obstacle',
            parameters=[
                {
                    'seed': int(obstacle_seed),
                    'max_linear_velocity': 0.3,
                    'max_angular_velocity': 0.8,
                    'direction_change_interval': 2.5,
                }
            ],
            output='screen',
        )

        # Bridge for obstacle cmd_vel
        obstacle_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/obstacle/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            ],
            output='screen',
        )

        nodes.extend([spawn_obstacle, random_walk_node, obstacle_bridge])

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'enable_obstacle',
                default_value='false',
                description='Enable random walking obstacle for testing',
            ),
            DeclareLaunchArgument(
                'obstacle_seed',
                default_value='42',
                description='Random seed for reproducible obstacle movement',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
