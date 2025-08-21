
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import math

def euler_from_quaternion(quaternion):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

class TfToOdom(Node):
    def __init__(self):
        super().__init__('tf_to_odom_publisher')

        # 노드 파라미터 선언
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')

        # 파라미터 값 가져오기
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value

        # TF 리스너 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Odometry 퍼블리셔 설정
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # 속도 계산을 위한 이전 상태 저장 변수
        self.last_transform = None
        self.last_time = None

        # 0.1초마다 콜백 함수 실행
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(
            f"TF to Odometry publisher started. Publishing odom for frames: "
            f"'{self.odom_frame}' -> '{self.child_frame}'"
        )

    def timer_callback(self):
        try:
            # odom -> base_link 간의 최신 transform 조회
            now = Time()
            transform_stamped = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.child_frame,
                now
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get transform: {e}')
            return

        current_time = self.get_clock().now()
        
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.child_frame

        # Pose 정보 채우기
        odom_msg.pose.pose.position.x = transform_stamped.transform.translation.x
        odom_msg.pose.pose.position.y = transform_stamped.transform.translation.y
        odom_msg.pose.pose.position.z = transform_stamped.transform.translation.z
        odom_msg.pose.pose.orientation = transform_stamped.transform.rotation
        
        # Twist (속도) 정보 계산 및 채우기
        if self.last_transform and self.last_time:
            dt = (current_time - self.last_time).nanoseconds / 1e9

            if dt > 0:
                # 선형 속도 (Linear velocity)
                dx = transform_stamped.transform.translation.x - self.last_transform.transform.translation.x
                dy = transform_stamped.transform.translation.y - self.last_transform.transform.translation.y
                odom_msg.twist.twist.linear.x = dx / dt
                odom_msg.twist.twist.linear.y = dy / dt

                # 각속도 (Angular velocity)
                _, _, last_yaw = euler_from_quaternion(self.last_transform.transform.rotation)
                _, _, current_yaw = euler_from_quaternion(transform_stamped.transform.rotation)
                dyaw = current_yaw - last_yaw
                
                # Yaw 값의 불연속성 처리 (e.g., from +pi to -pi)
                if dyaw > math.pi:
                    dyaw -= 2 * math.pi
                if dyaw < -math.pi:
                    dyaw += 2 * math.pi

                odom_msg.twist.twist.angular.z = dyaw / dt

        # Odometry 메시지 퍼블리시
        self.odom_pub.publish(odom_msg)

        # 현재 상태를 다음 계산을 위해 저장
        self.last_transform = transform_stamped
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = TfToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
