import os
import random
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


def generate_obstacle_sdf(model_name: str, cmd_vel_topic: str) -> str:
    """Generate SDF string for moving obstacle with unique cmd_vel topic."""
    return f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>false</static>

    <!-- Body (cylinder) -->
    <link name="base_link">
      <pose>0 0 0.75 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>10.0</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>10.0</iyy>
          <iyz>0</iyz>
          <izz>0.5</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.2</radius>
            <length>1.5</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.0</mu>
              <mu2>0.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.2</radius>
            <length>1.5</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>0.8 0.2 0.2 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>

    <!-- Left wheel -->
    <link name="left_wheel">
      <pose>0 0.25 0.05 -1.5707 0 0</pose>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
        </material>
      </visual>
    </link>

    <!-- Right wheel -->
    <link name="right_wheel">
      <pose>0 -0.25 0.05 -1.5707 0 0</pose>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
        </material>
      </visual>
    </link>

    <!-- Caster (ball) for stability -->
    <link name="caster">
      <pose>-0.15 0 0.025 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0001</iyy>
          <iyz>0</iyz>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.025</radius>
          </sphere>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.0</mu>
              <mu2>0.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.025</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
        </material>
      </visual>
    </link>

    <!-- Joints -->
    <joint name="left_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>left_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <joint name="right_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>right_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <joint name="caster_joint" type="ball">
      <parent>base_link</parent>
      <child>caster</child>
    </joint>

    <!-- Diff drive plugin -->
    <plugin filename="gz-sim-diff-drive-system"
            name="gz::sim::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.5</wheel_separation>
      <wheel_radius>0.05</wheel_radius>
      <topic>{cmd_vel_topic}</topic>
      <odom_publish_frequency>0</odom_publish_frequency>
    </plugin>
  </model>
</sdf>'''


def launch_setup(context, *args, **kwargs):
    pkg_name = 'simple_diff_drive_sim'

    # Get package share directory
    pkg_share = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'test_world.sdf')

    # Get launch configurations
    enable_obstacle = LaunchConfiguration('enable_obstacle').perform(context)
    obstacle_seed = LaunchConfiguration('obstacle_seed').perform(context)
    num_obstacles = int(LaunchConfiguration('num_obstacles').perform(context))
    spawn_x_min = float(LaunchConfiguration('spawn_x_min').perform(context))
    spawn_x_max = float(LaunchConfiguration('spawn_x_max').perform(context))
    spawn_y_min = float(LaunchConfiguration('spawn_y_min').perform(context))
    spawn_y_max = float(LaunchConfiguration('spawn_y_max').perform(context))

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

    # Add obstacles if enabled
    if enable_obstacle.lower() == 'true':
        # Initialize RNG with seed for reproducible spawn positions
        random.seed(int(obstacle_seed))

        # Spawn multiple obstacles
        for i in range(num_obstacles):
            obstacle_name = f'moving_obstacle_{i}'
            cmd_vel_topic = f'/obstacle_{i}/cmd_vel'

            # Random spawn position within specified range
            x_pos = random.uniform(spawn_x_min, spawn_x_max)
            y_pos = random.uniform(spawn_y_min, spawn_y_max)

            # Generate SDF with unique cmd_vel topic
            obstacle_sdf = generate_obstacle_sdf(obstacle_name, cmd_vel_topic)

            # Spawn obstacle model with inline SDF
            spawn_obstacle = Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-string',
                    obstacle_sdf,
                    '-name',
                    obstacle_name,
                    '-x',
                    str(x_pos),
                    '-y',
                    str(y_pos),
                    '-z',
                    '0.0',
                ],
                output='screen',
            )

            # Random walk controller node with unique seed
            random_walk_node = Node(
                package='simple_diff_drive_sim',
                executable='random_walk_obstacle',
                name=f'random_walk_obstacle_{i}',
                parameters=[
                    {
                        'seed': int(obstacle_seed) + i,  # Unique seed per obstacle
                        'max_linear_velocity': 0.3,
                        'max_angular_velocity': 0.8,
                        'direction_change_interval': 2.5,
                        'cmd_vel_topic': cmd_vel_topic,
                    }
                ],
                output='screen',
            )

            # Bridge for obstacle cmd_vel
            obstacle_bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    f'{cmd_vel_topic}@geometry_msgs/msg/Twist@gz.msgs.Twist',
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
                description='Enable random walking obstacles for testing',
            ),
            DeclareLaunchArgument(
                'obstacle_seed',
                default_value='42',
                description='Random seed for reproducible obstacle movement and spawn positions',
            ),
            DeclareLaunchArgument(
                'num_obstacles',
                default_value='3',
                description='Number of dynamic obstacles to spawn',
            ),
            DeclareLaunchArgument(
                'spawn_x_min',
                default_value='-3.0',
                description='Minimum X coordinate for obstacle spawn area',
            ),
            DeclareLaunchArgument(
                'spawn_x_max',
                default_value='3.0',
                description='Maximum X coordinate for obstacle spawn area',
            ),
            DeclareLaunchArgument(
                'spawn_y_min',
                default_value='-3.0',
                description='Minimum Y coordinate for obstacle spawn area',
            ),
            DeclareLaunchArgument(
                'spawn_y_max',
                default_value='3.0',
                description='Maximum Y coordinate for obstacle spawn area',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
