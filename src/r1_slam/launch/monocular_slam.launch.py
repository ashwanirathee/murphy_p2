from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_topic = LaunchConfiguration("camera_topic")
    focal_length = LaunchConfiguration("focal_length")
    principal_point_x = LaunchConfiguration("principal_point_x")
    principal_point_y = LaunchConfiguration("principal_point_y")
    publish_debug_image = LaunchConfiguration("publish_debug_image")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_topic",
                default_value="/camera/uid_0/image_raw",
                description="Monocular image topic to track.",
            ),
            DeclareLaunchArgument(
                "focal_length",
                default_value="320.0",
                description="Approximate focal length in pixels.",
            ),
            DeclareLaunchArgument(
                "principal_point_x",
                default_value="160.0",
                description="Camera principal point x in pixels.",
            ),
            DeclareLaunchArgument(
                "principal_point_y",
                default_value="120.0",
                description="Camera principal point y in pixels.",
            ),
            DeclareLaunchArgument(
                "publish_debug_image",
                default_value="true",
                description="Publish tracked feature overlays.",
            ),
            Node(
                package="r1_slam",
                executable="monocular_slam_node",
                name="monocular_slam_node",
                output="screen",
                parameters=[
                    {
                        "camera_topic": camera_topic,
                        "focal_length": focal_length,
                        "principal_point_x": principal_point_x,
                        "principal_point_y": principal_point_y,
                        "publish_debug_image": publish_debug_image,
                    }
                ],
            ),
        ]
    )
