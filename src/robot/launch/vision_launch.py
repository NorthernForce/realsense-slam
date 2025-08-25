from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robot",
            executable="robot_bridge_node",
            name="robot_bridge",
            namespace="bridge",
            remappings=[("video", "/camera/D435i/color/image_raw")]
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ]),
            launch_arguments={
                "camera_name": "D435i",
                "enable_sync": "true",
                "enable_rgbd": "true",
                "align_depth.enable": "true",
                "pointcloud.enable": "true"
            }.items()
        )
        # TODO: realsense2 node, robot_localization node, + remappings
    ])
