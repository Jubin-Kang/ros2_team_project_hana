from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("pub_rate", default_value="100"),
            DeclareLaunchArgument("output_topic", default_value="/imu_raw"),  # remap용

            Node(
                package="ros2_icm20948",
                executable="icm20948_node",
                name="icm20948_node",
                parameters=[
                    {
                        "i2c_address": LaunchConfiguration("i2c_address"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "pub_rate": LaunchConfiguration("pub_rate"),
                    },
                ],
                remappings=[
                    ("imu", LaunchConfiguration("output_topic")),  # remap 처리
                ],
            )
        ]
    )

