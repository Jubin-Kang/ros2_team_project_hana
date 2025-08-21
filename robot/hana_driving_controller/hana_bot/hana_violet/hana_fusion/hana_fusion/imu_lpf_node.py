#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuLpfNode(Node):
    """
    IMU 데이터를 구독하여 저역 통과 필터를 적용하고 다시 발행하는 노드.
    """
    def __init__(self):
        super().__init__('imu_lpf_node')
        
        # 파라미터 선언 (alpha, input_topic, output_topic)
        self.declare_parameter('alpha', 0.1)
        self.declare_parameter('input_topic', '/imu_raw')
        self.declare_parameter('output_topic', '/imu')
        
        # 파라미터 값 가져오기
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        
        # 필터링된 값을 저장할 변수 초기화
        self.filtered_acc_x = None
        self.filtered_acc_y = None
        self.filtered_acc_z = None
        self.filtered_gyro_x = None
        self.filtered_gyro_y = None
        self.filtered_gyro_z = None
        
        # 퍼블리셔와 서브스크라이버 생성
        self.publisher_ = self.create_publisher(Imu, output_topic, 10)
        self.subscription = self.create_subscription(
            Imu,
            input_topic,
            self.imu_callback,
            10)
        
        self.get_logger().info(
            f"IMU LPF 노드 시작. alpha: {self.alpha}, "
            f"입력 토픽: '{input_topic}', 출력 토픽: '{output_topic}'"
        )
    
    def imu_callback(self, msg):
        """
        IMU 메시지를 받을 때마다 호출되는 콜백 함수.
        """
        # 첫 메시지인 경우, 필터 초기값을 현재 값으로 설정
        if self.filtered_acc_x is None:
            self.filtered_acc_x = msg.linear_acceleration.x
            self.filtered_acc_y = msg.linear_acceleration.y
            self.filtered_acc_z = msg.linear_acceleration.z
            self.filtered_gyro_x = msg.angular_velocity.x
            self.filtered_gyro_y = msg.angular_velocity.y
            self.filtered_gyro_z = msg.angular_velocity.z
        else:
            # 저역 통과 필터(LPF) 적용
            # filtered = alpha * new_raw + (1 - alpha) * previous_filtered
            self.filtered_acc_x = self.alpha * msg.linear_acceleration.x + (1.0 - self.alpha) * self.filtered_acc_x
            self.filtered_acc_y = self.alpha * msg.linear_acceleration.y + (1.0 - self.alpha) * self.filtered_acc_y
            self.filtered_acc_z = self.alpha * msg.linear_acceleration.z + (1.0 - self.alpha) * self.filtered_acc_z
            
            self.filtered_gyro_x = self.alpha * msg.angular_velocity.x + (1.0 - self.alpha) * self.filtered_gyro_x
            self.filtered_gyro_y = self.alpha * msg.angular_velocity.y + (1.0 - self.alpha) * self.filtered_gyro_y
            self.filtered_gyro_z = self.alpha * msg.angular_velocity.z + (1.0 - self.alpha) * self.filtered_gyro_z
        
        # 필터링된 데이터로 새 Imu 메시지 생성
        lpf_imu_msg = Imu()
        lpf_imu_msg.header = msg.header  # 헤더는 원본 메시지와 동일하게 유지
        
        # 필터링된 값 할당
        lpf_imu_msg.linear_acceleration.x = self.filtered_acc_x
        lpf_imu_msg.linear_acceleration.y = self.filtered_acc_y
        lpf_imu_msg.linear_acceleration.z = self.filtered_acc_z
        lpf_imu_msg.angular_velocity.x = self.filtered_gyro_x
        lpf_imu_msg.angular_velocity.y = self.filtered_gyro_y
        lpf_imu_msg.angular_velocity.z = self.filtered_gyro_z
        
        # 필터링하지 않은 값들은 그대로 복사
        lpf_imu_msg.orientation = msg.orientation
        lpf_imu_msg.orientation_covariance = msg.orientation_covariance
        lpf_imu_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        lpf_imu_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        
        # 필터링된 메시지 발행
        self.publisher_.publish(lpf_imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuLpfNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('키보드 인터럽트로 노드 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()