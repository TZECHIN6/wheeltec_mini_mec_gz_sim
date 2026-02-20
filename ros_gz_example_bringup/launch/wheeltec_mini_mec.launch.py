import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer

def generate_launch_description():
    pkg_share = get_package_share_directory('ros_gz_example_description')
    config_pkg_share = get_package_share_directory('ros_gz_example_bringup')
    project_gazebo_share = get_package_share_directory('ros_gz_example_gazebo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_spawn_model_launch_source = os.path.join(ros_gz_sim_share, "launch", "gz_spawn_model.launch.py")
    default_model_path = os.path.join(pkg_share, 'models', 'wheeltec_mini_mec', 'model.sdf')
    default_world_path = os.path.join(project_gazebo_share, 'worlds', 'wheeltec_mini_mec_world.sdf')
    default_bridge_config_path = os.path.join(config_pkg_share, 'config', 'wheeltec_mini_mec_bridge.yaml')
    default_rviz_config_path = os.path.join(config_pkg_share, 'config', 'wheeltec_mini_mec.rviz')

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
    gz_server = GzServer(
        world_sdf_file=default_world_path,
        container_name='ros_gz_container',
        create_own_container='True',
        use_composition='True',
    )
    # Bridge ROS topics and Gazebo messages for establishing communication
    ros_gz_bridge = RosGzBridge(
        bridge_name = "ros_gz_bridge",
        config_file = default_bridge_config_path,
        container_name = "ros_gz_container",
        create_own_container = False,
        use_composition = True,
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
    # Spawn the robot entity in Gazebo
    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            'world': 'wheeltec_mini_mec_world',
            'topic': '/robot_description',
            'entity_name': 'wheeltec_mini_mec',
            'z': '0.1',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'),
        robot_state_publisher_node,
        rviz_node,
        gz_server,
        ros_gz_bridge,
        camera_bridge_image,
        ros_gz_bridge,
        spawn_entity,
    ])
