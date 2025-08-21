import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # RF2O Laser Odometry Node
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom_rf2o',  # EKF에서 사용할 토픽명
                'publish_tf': False,  # EKF가 TF를 퍼블리시하므로 False
                'base_frame_id': 'base_footprint',
                'odom_frame_id': 'odom',  # odom_raw 대신 odom 사용
                'init_pose_from_topic': '',
                'freq': 10.0,  # 주파수를 20Hz로 증가
                
                # 추가 파라미터들 (지원하는 경우)
                'verbose': False,
                'recover_from_pause': True,
                
                # 모션 모델 파라미터
                'linear_variation': 0.0,
                'angular_variation': 0.0,
                
                # 범위 설정
                'min_laser_range': 0.1,
                'max_laser_range': 30.0,
                
                # 공분산 설정 (x, y, z, roll, pitch, yaw)
                # rf2o는 2D이므로 x, y, yaw만 유효
                'pose_covariance_diagonal': [0.01, 0.01, 99999.0, 99999.0, 99999.0, 0.05],
                'twist_covariance_diagonal': [0.05, 0.05, 99999.0, 99999.0, 99999.0, 0.1]
            }],
            remappings=[
                # 필요시 리맵핑 추가
                # ('/tf', '/tf'),
                # ('/tf_static', '/tf_static')
            ]
        ),
    ])