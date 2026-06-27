from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    camera_uids = LaunchConfiguration("camera_uids")
    camera_labels = LaunchConfiguration("camera_labels")
    latest_image_path = LaunchConfiguration("latest_image_path")
    ollama_url = LaunchConfiguration("ollama_url")
    vlm_model = LaunchConfiguration("vlm_model")
    use_espeak = LaunchConfiguration("use_espeak")
    enable_2dobd = LaunchConfiguration("enable_2dobd")
    enable_3dobd = LaunchConfiguration("enable_3dobd")
    yolo_camera_uid = LaunchConfiguration("yolo_camera_uid")
    enable_ear = LaunchConfiguration("enable_ear")
    enable_vlm = LaunchConfiguration("enable_vlm")
    enable_slam = LaunchConfiguration("enable_slam")
    event_min_interval_sec = LaunchConfiguration("event_min_interval_sec")
    event_max_silence_sec = LaunchConfiguration("event_max_silence_sec")
    slam_camera_topic = LaunchConfiguration("slam_camera_topic")
    slam_focal_length = LaunchConfiguration("slam_focal_length")
    slam_principal_point_x = LaunchConfiguration("slam_principal_point_x")
    slam_principal_point_y = LaunchConfiguration("slam_principal_point_y")
    slam_publish_debug_image = LaunchConfiguration("slam_publish_debug_image")
    enable_any_obd = PythonExpression(
        ["'", enable_2dobd, "' == 'true' or '", enable_3dobd, "' == 'true'"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_uids",
                default_value="[0, 1]",
                description="Camera device ids for the cameras node and visual processor.",
            ),
            DeclareLaunchArgument(
                "camera_labels",
                default_value='["left", "right"]',
                description="Labels matching camera_uids.",
            ),
            DeclareLaunchArgument(
                "latest_image_path",
                default_value="/home/ubuntu/r1/latest_frame.jpg",
                description="Path shared by the visual processor and brain/VLM nodes.",
            ),
            DeclareLaunchArgument(
                "ollama_url",
                default_value="http://192.168.68.59:11434",
                description="Base URL for the Ollama server used by vlm_node.",
            ),
            DeclareLaunchArgument(
                "vlm_model",
                default_value="moondream",
                description="Vision-language model name for vlm_node.",
            ),
            DeclareLaunchArgument(
                "use_espeak",
                default_value="true",
                description="Whether audio_node should speak with espeak-ng.",
            ),
            DeclareLaunchArgument(
                "enable_2dobd",
                default_value="true",
                description="Start the 2D obstacle detection visual processor node.",
            ),
            DeclareLaunchArgument(
                "enable_3dobd",
                default_value="false",
                description="Enable 3D obstacle detection mode in visual_processor_node.",
            ),
            DeclareLaunchArgument(
                "yolo_camera_uid",
                default_value="0",
                description="Camera uid whose image stream should be used by YOLO.",
            ),
            DeclareLaunchArgument(
                "event_min_interval_sec",
                default_value="0.5",
                description="Minimum time between changed visual events from the same camera.",
            ),
            DeclareLaunchArgument(
                "event_max_silence_sec",
                default_value="2.0",
                description="Maximum silence before republishing a visual heartbeat event.",
            ),
            DeclareLaunchArgument(
                "enable_ear",
                default_value="true",
                description="Start ear_node for terminal text input.",
            ),
            DeclareLaunchArgument(
                "enable_vlm",
                default_value="true",
                description="Start vlm_node for image question answering.",
            ),
            DeclareLaunchArgument(
                "enable_slam",
                default_value="false",
                description="Start the experimental monocular SLAM node.",
            ),
            DeclareLaunchArgument(
                "slam_camera_topic",
                default_value="/camera/uid_0/image_raw",
                description="Image topic for the monocular SLAM node.",
            ),
            DeclareLaunchArgument(
                "slam_focal_length",
                default_value="320.0",
                description="Approximate focal length in pixels for monocular SLAM.",
            ),
            DeclareLaunchArgument(
                "slam_principal_point_x",
                default_value="160.0",
                description="Principal point x in pixels for monocular SLAM.",
            ),
            DeclareLaunchArgument(
                "slam_principal_point_y",
                default_value="120.0",
                description="Principal point y in pixels for monocular SLAM.",
            ),
            DeclareLaunchArgument(
                "slam_publish_debug_image",
                default_value="true",
                description="Publish a debug image with tracked features for monocular SLAM.",
            ),
            Node(
                package="r1",
                executable="cameras_node",
                name="cameras_node",
                output="screen",
                parameters=[
                    {
                        "camera_uids": camera_uids,
                        "camera_labels": camera_labels,
                    }
                ],
            ),
            Node(
                package="r1",
                executable="visual_processor_node",
                name="visual_processor_node",
                output="screen",
                parameters=[
                    {
                        "camera_uids": camera_uids,
                        "camera_labels": camera_labels,
                        "event_min_interval_sec": event_min_interval_sec,
                        "event_max_silence_sec": event_max_silence_sec,
                        "enable_2dobd": enable_2dobd,
                        "enable_3dobd": enable_3dobd,
                        "yolo_camera_uid": yolo_camera_uid,
                    }
                ],
                condition=IfCondition(enable_any_obd),
            ),
            Node(
                package="r1",
                executable="brain_node",
                name="brain_node",
                output="screen",
                parameters=[{"latest_image_path": latest_image_path}],
            ),
            Node(
                package="r1",
                executable="audio_node",
                name="audio_node",
                output="screen",
                parameters=[{"use_espeak": use_espeak}],
            ),
            Node(
                package="r1",
                executable="action_node",
                name="action_node",
                output="screen",
            ),
            Node(
                package="r1",
                executable="ear_node",
                name="ear_node",
                output="screen",
                condition=IfCondition(enable_ear),
            ),
            Node(
                package="r1_slam",
                executable="monocular_slam_node",
                name="monocular_slam_node",
                output="screen",
                parameters=[
                    {
                        "camera_topic": slam_camera_topic,
                        "focal_length": slam_focal_length,
                        "principal_point_x": slam_principal_point_x,
                        "principal_point_y": slam_principal_point_y,
                        "publish_debug_image": slam_publish_debug_image,
                    }
                ],
                condition=IfCondition(enable_slam),
            ),
            Node(
                package="r1",
                executable="vlm_node",
                name="vlm_node",
                output="screen",
                parameters=[
                    {
                        "ollama_url": ollama_url,
                        "model": vlm_model,
                    }
                ],
                condition=IfCondition(enable_vlm),
            ),
        ]
    )
