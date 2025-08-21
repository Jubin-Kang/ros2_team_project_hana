from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define the path to the EKF configuration file
    ekf_config_path = os.path.join(
        get_package_share_directory('hana_fusion'),
        'config',
        'ekf_pos.yaml' # ekf has some error
    )

    # Create the EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[
            ('/odometry/filtered', '/odom')
        ]
    )

    return LaunchDescription([ekf_node])