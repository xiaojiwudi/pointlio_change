from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory("odom_pointlio_wheel")
    params = os.path.join(pkg, "config", "fusion_params.yaml")

    return LaunchDescription([
        Node(
            package="odom_pointlio_wheel",
            executable="odom_pointlio_wheel_node",
            name="odom_pointlio_wheel_node",
            output="screen",
            parameters=[params],
            remappings=[
                # 如需重映射话题，在此修改
                ("/Odometry", "/aft_mapped_to_init"),
                ("/wheel_odom", "/wheel_topic"),
            ],
        )
    ])
