import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
import numpy as np
import os
import cv2
from picamera2 import Picamera2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.image_publisher = self.create_publisher(Image, '/image_raw', 10)
        self.info_publisher = self.create_publisher(CameraInfo, '/camera_info', 10)
        self.timer = self.create_timer(0.1, self.publish_image)
        self.info_timer = self.create_timer(1.0, self.publish_camera_info)
        
        self.camera_info = self.load_camera_info()
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(
            main={"format": 'RGB888', "size": (640, 480)}))
        self.picam2.start()
        
        if self.camera_info:
            self.get_logger().info("Camera Info Publisher Node Started")
        else:
            self.get_logger().warn("No camera info loaded, continuing without publishing.")

    def load_camera_info(self):
        npz_file = '/home/hana/calib/camera_calibration.npz'  # 수정 필요
        if not os.path.exists(npz_file):
            self.get_logger().warn(f"NPZ file {npz_file} not found, skipping camera info loading.")
            return None
        
        try:
            data = np.load(npz_file)
            camera_info = CameraInfo()
            camera_info.width = 640  # 기본값 설정
            camera_info.height = 480  # 기본값 설정
            camera_info.k = data['camera_matrix'].flatten().tolist()
            camera_info.d = data['distortion_coefficients'].flatten().tolist()
            camera_info.distortion_model = 'plumb_bob'
            
            # rectification_matrix 기본값 (3x3 단위 행렬)
            camera_info.r = np.eye(3).flatten().tolist()
            
            # projection_matrix 기본값 (camera_matrix에 4번째 열 추가)
            projection_matrix = np.hstack((data['camera_matrix'], np.zeros((3, 1))))
            camera_info.p = projection_matrix.flatten().tolist()
            
            return camera_info
        except Exception as e:
            self.get_logger().error(f"Failed to load camera info: {e}")
            return None

    def publish_camera_info(self):
        if self.camera_info:
            self.camera_info.header.stamp = self.get_clock().now().to_msg()
            self.camera_info.header.frame_id = 'camera'
            self.info_publisher.publish(self.camera_info)

    def publish_image(self):
        frame = self.picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        
        self.image_publisher.publish(msg)

    def __del__(self):
        if hasattr(self, 'picam2'):
            self.picam2.close()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()