import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import time

class ArucoLauncher(Node):
    def __init__(self):
        super().__init__('aruco_launcher')
        self.subscription = self.create_subscription(
            String,
            '/aruco_start',
            self.callback,
            10
        )
        self.get_logger().info("ArUco 런처 노드 활성화됨")

    def callback(self, msg):
        if msg.data == "start":
            self.get_logger().info("ArUco 처리 시작")

            # video_stream_aruco_pose.py 실행 (백그라운드로)
            subprocess.Popen(
                ["python3", "/home/jetcobot/dev_ws/src/video_stream_aruco_pose.py"]
            )
                        
            time.sleep(2)
            
            # aruco_robot_centering.py 실행 (동기 실행)
            subprocess.Popen(
                ["python3", "/home/jetcobot/dev_ws/src/aruco_robot_centering.py"]
            )

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLauncher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
