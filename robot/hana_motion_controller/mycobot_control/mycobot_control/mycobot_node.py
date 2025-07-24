import rclpy
from rclpy.node import Node
from pymycobot.mycobot import MyCobot
from std_msgs.msg import Float64MultiArray

class MyCobotNode(Node):
    def __init__(self):
        super().__init__('mycobot_node')

        # JetCobot 연결
        self.mc = MyCobot('/dev/ttyUSB0', 1000000)
        self.get_logger().info("JetCobot 연결됨")

        # 토픽 구독 시작
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/set_joint_angles',
            self.joint_callback,
            10
        )

    def joint_callback(self, msg):
        angles = list(msg.data)
        if len(angles) != 6:
            self.get_logger().warn("관절 각도는 6개여야 합니다.")
            return
        self.get_logger().info(f"이동 → {angles}")
        self.mc.send_angles(angles, 50)

def main(args=None):
    rclpy.init(args=args)
    node = MyCobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
