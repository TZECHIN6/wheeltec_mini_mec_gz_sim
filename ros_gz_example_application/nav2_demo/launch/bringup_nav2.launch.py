from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    ),
    DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [get_package_share_directory("nav2_demo"), "params", "nav2_params_sim.yaml"]
        ),
        description="Full path to the ROS2 parameters file",
    ),
    DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution(
            [get_package_share_directory("nav2_demo"), "maps", "map_office.yaml"]
        ),
        description="Full path to map YAML file",
    ),
    DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    ),
    DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to launch RViz",
    ),
    DeclareLaunchArgument(
        "rviz_config_file",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("nav2_bringup"),
                "rviz",
                "nav2_default_view.rviz",
            ]
        ),
        description="Full path to the RViz config file to use",
    ),
    # Arguments for costmap filter
    DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    ),
    DeclareLaunchArgument(
        "mask",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("nav2_demo"),
                "maps",
                "map_office_keepout.yaml",
            ]
        ),
        description="Full path to filter mask yaml file to load",
    ),
    DeclareLaunchArgument(
        "params_file_keepout",
        default_value=PathJoinSubstitution(
            [get_package_share_directory("nav2_demo"), "params", "keepout_params.yaml"]
        ),
        description="Full path to the ROS2 parameters file for keepout",
    ),
]


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    my_nav2_dir = get_package_share_directory("nav2_demo")
    docking_demo_dir = get_package_share_directory("docking_demo")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_yaml_file = LaunchConfiguration("map")
    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    use_composition = LaunchConfiguration("use_composition")
    mask_yaml_file = LaunchConfiguration("mask")
    params_file_keepout = LaunchConfiguration("params_file_keepout")

    # Include Nav2 bringup
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, "launch", "bringup_launch.py"])
        ),
        launch_arguments={
            "map": map_yaml_file,
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "namespace": namespace,
        }.items(),
    )

    # Include costmap filter
    costmap_filter_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [my_nav2_dir, "launch", "costmap_filter_info.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_composition": use_composition,
            "mask": mask_yaml_file,
            "params_file": params_file_keepout,
        }.items(),
    )

    # Include apriltag detection
    apriltag_detection_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [docking_demo_dir, "launch", "apriltag_detection.launch.py"]
            )
        ),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, "launch", "rviz_launch.py"])
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    # Combine everything
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(costmap_filter_cmd)
    ld.add_action(apriltag_detection_cmd)
    ld.add_action(rviz_cmd)

    return ld
