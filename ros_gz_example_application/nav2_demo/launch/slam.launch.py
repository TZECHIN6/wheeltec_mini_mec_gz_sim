from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    ),
    DeclareLaunchArgument(
        "slam_params_file",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("nav2_demo"),
                "params",
                "mapper_params_online_async.yaml",
            ]
        ),
        description="Full path to the ROS2 parameters file to use for SLAM",
    ),
]


def generate_launch_description():
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")

    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_toolbox_dir, "launch", "online_async_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
        }.items(),
    )

    # Combine everything
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(slam_toolbox_cmd)

    return ld
