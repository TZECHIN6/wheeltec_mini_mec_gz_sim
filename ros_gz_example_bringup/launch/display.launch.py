import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ros_gz_example_description')
    config_pkg_share = get_package_share_directory('ros_gz_example_bringup')
    project_gazebo_share = get_package_share_directory('ros_gz_example_gazebo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    default_model_path = os.path.join(pkg_share, 'models', 'wheeltec_mini_mec', 'model_test.sdf')
    default_rviz_config_path = os.path.join(config_pkg_share, 'config', 'display_config.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            project_gazebo_share,
            'worlds',
            'empty_world.sdf'
        ])}.items(),
    )
    # Bridge ROS topics and Gazebo messages for establishing communication
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(config_pkg_share, 'config', 'bridge_config.yaml'),
        }],
        output='screen'
    )
    # Bridge camera topic from Gazebo to ROS
    camera_bridge_image = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='bridge_gz_ros_camera_image',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['/camera/image_raw'],
    )
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "wheeltec_mini_mec",
            '-file', default_model_path,
            '-x', "0.0",
            '-y', "0.0",
            '-z', "0.1"
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        robot_state_publisher_node,
        rviz_node,
        gz_sim,
        spawn_entity,
        ros_gz_bridge,
        camera_bridge_image,
    ])