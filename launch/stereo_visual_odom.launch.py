from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros
import os


def generate_launch_description():
    # Launch configuration variables
    node_name = LaunchConfiguration(
        "stereo_visual_odometry_node", default="stereo_visual_odometry_node"
    )

    # Node for publishing Eigen::Vector3d to PointCloud2
    stereo_visual_odom_node = Node(
        package="stereo_visual_odometry",
        executable="stereo_visual_odometry_node",
        name=node_name,
        output="screen",
    )

    # Node for broadcasting a static transform for camera_optical_link
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "-1.570796",
            "0",
            "-1.570796",
            "odom",
            "camera_optical_link",
        ],
        name="static_tf_publisher",
    )
    odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "map",
            "odom",
        ],
        name="static_tf_publisher",
    )

    return LaunchDescription(
        [
            stereo_visual_odom_node,
            odom_tf,
            camera_tf,
        ]
    )
