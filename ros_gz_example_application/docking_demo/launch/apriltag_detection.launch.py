from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share_docking_demo = get_package_share_directory("docking_demo")
    apriltag_params_file = PathJoinSubstitution(
        [pkg_share_docking_demo, "config", "apriltag_params.yaml"]
    )

    rectify_node = ComposableNode(
        package="image_proc",
        plugin="image_proc::RectifyNode",
        name="rectify_node",
        remappings=[
            ("image", "/camera/image_raw"),
            ("image_rect", "/camera/image_rect"),
            ("camera_info", "/camera/camera_info"),
        ],
        parameters=[{"use_sim_time": True}],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    apriltag_node = ComposableNode(
        package="apriltag_ros",
        plugin="AprilTagNode",
        name="apriltag_node",
        remappings=[
            ("camera_info", "/camera/camera_info"),
            ("image_rect", "/camera/image_rect"),
        ],
        parameters=[apriltag_params_file, {"use_sim_time": True}],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    start_apriltag_detection_container = ComposableNodeContainer(
        name="apriltag_detection_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[rectify_node, apriltag_node],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(start_apriltag_detection_container)

    return ld
