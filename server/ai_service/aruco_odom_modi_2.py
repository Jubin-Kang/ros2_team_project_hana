import cv2
import numpy as np
import socket
import pickle
import math
from typing import Tuple
from scipy.spatial.transform import Rotation as SciR

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# =================================================================================
# UDP Multicast Configuration
# =================================================================================
MCAST_GRP = '224.1.1.1'
MCAST_PORT = 5000
BUFFER_SIZE = 65536

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', MCAST_PORT))
mreq = socket.inet_aton(MCAST_GRP) + socket.inet_aton('0.0.0.0')
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

# =================================================================================
# 2D Pose Estimation Parameters (from aruco_git_odom_bin.py)
# =================================================================================
SIMILARITY_S = 1.1731463819258798
SIMILARITY_R = np.array([[ 0.02926172, -0.99957178],
                         [-0.99957178, -0.02926172]])
SIMILARITY_T = np.array([-0.00178071,  0.01315473])
THETA_RAD    = math.radians(-88.32318748498734)
DET_R_SIGN   = -1
ARUCO_XY_AXES = (0, 1)

# =================================================================================
# Utility Functions (from aruco_git_odom_bin.py)
# =================================================================================
def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi

def euler_from_quat_xyzw(q: np.ndarray) -> Tuple[float, float, float]:
    r = SciR.from_quat(q)
    roll, pitch, yaw = r.as_euler('xyz', degrees=False)
    return roll, pitch, yaw

def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n <= 1e-12:
        return np.array([0,0,0,1.0], dtype=float)
    return q / n

def convert_position(ax: float, ay: float) -> Tuple[float, float]:
    p = np.array([ax, ay])
    mp = (SIMILARITY_R @ p) * SIMILARITY_S + SIMILARITY_T
    return float(mp[0]), float(mp[1])

def convert_yaw(yaw_cam_rad: float) -> float:
    if DET_R_SIGN >= 0:
        return wrap(yaw_cam_rad + THETA_RAD)
    else:
        return wrap(-yaw_cam_rad + THETA_RAD)

# =================================================================================
# ROS2 Node for ArUco Detection
# =================================================================================
class ArucoReceiverNode(Node):
    def __init__(self):
        super().__init__('aruco_receiver_node')

        # --- Parameters for publishing ---
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('publish_odom', True)
        self.publish_tf   = bool(self.get_parameter('publish_tf').value)
        self.publish_odom = bool(self.get_parameter('publish_odom').value)

        # --- ROS2 Publishers and Broadcasters ---
        self.odom_pub = self.create_publisher(Odometry, '/DP_09/odom_aruco', qos_profile_sensor_data)
        self.tf_brd   = TransformBroadcaster(self)

        # --- State for velocity calculation ---
        self.prev_stamp = None
        self.prev_pose  = None  # (x, y, yaw)

        # --- Hardcoded Camera Parameters ---
        self.camera_matrix = np.array([[1185.96684, 0, 999.31995],
                                       [0, 890.7003, 569.28861],
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([-9.413361e-02, -8.374589e-04, 3.176887e-04, -3.987077e-04, 3.289896e-03, 0.0, 0.0, 0.0], dtype=np.float32)

        # --- Perspective Transform Parameters ---
        self.src_pts_original = np.float32([[432, 200], [390, 772], [1580, 780], [1548, 217]])
        self.crop_offsets = np.float32([390, 200])
        src_pts_relative = self.src_pts_original - self.crop_offsets
        width_top = np.linalg.norm(src_pts_relative[3] - src_pts_relative[0])
        width_bottom = np.linalg.norm(src_pts_relative[2] - src_pts_relative[1])
        height_left = np.linalg.norm(src_pts_relative[1] - src_pts_relative[0])
        height_right = np.linalg.norm(src_pts_relative[2] - src_pts_relative[3])
        self.dst_width = int(max(width_top, width_bottom))
        self.dst_height = int(max(height_left, height_right))
        self.dst_pts = np.float32([[0, 0], [0, self.dst_height], [self.dst_width, self.dst_height], [self.dst_width, 0]])
        self.M = cv2.getPerspectiveTransform(src_pts_relative, self.dst_pts)

        # --- ArUco Detector Initialization ---
        self.get_logger().info("Initializing ArUco detector...")
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        self.aruco_params.cornerRefinementWinSize = 30
        self.aruco_params.cornerRefinementMaxIterations = 70
        self.aruco_params.cornerRefinementMinAccuracy = 0.001
        self.marker_length = 0.1 # 10cm
        self.get_logger().info("ArUco detector initialized.")

        # --- Main processing loop via timer ---
        self.create_timer(0.001, self.receive_and_process)
        self.get_logger().info(f"Listening for frames on {MCAST_GRP}:{MCAST_PORT}")

    def _publish_tf_and_odom(self, stamp, x, y, yaw):
        q = SciR.from_euler('z', yaw).as_quat()  # [x,y,z,w]

        # TF: odom_aruco -> base_footprint
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'odom_aruco'
            t.child_frame_id  = 'base_footprint'
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = 0.0
            t.transform.rotation.x = float(q[0])
            t.transform.rotation.y = float(q[1])
            t.transform.rotation.z = float(q[2])
            t.transform.rotation.w = float(q[3])
            self.tf_brd.sendTransform(t)

        # /odom_aruco
        if self.publish_odom:
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id  = 'base_footprint'
            odom.pose.pose.position.x = float(x)
            odom.pose.pose.position.y = float(y)
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = float(q[0])
            odom.pose.pose.orientation.y = float(q[1])
            odom.pose.pose.orientation.z = float(q[2])
            odom.pose.pose.orientation.w = float(q[3])

            # Velocity calculation
            if self.prev_stamp is not None and self.prev_pose is not None:
                dt = (stamp.sec + stamp.nanosec*1e-9) - (self.prev_stamp.sec + self.prev_stamp.nanosec*1e-9)
                if dt > 1e-4:
                    vx = (x - self.prev_pose[0]) / dt
                    vy = (y - self.prev_pose[1]) / dt
                    wyaw = wrap(yaw - self.prev_pose[2]) / dt
                    odom.twist.twist.linear.x  = float(vx)
                    odom.twist.twist.linear.y  = float(vy)
                    odom.twist.twist.angular.z = float(wyaw)

            # Covariance
            odom.pose.covariance[0]  = 0.01
            odom.pose.covariance[7]  = 0.01
            odom.pose.covariance[35] = 0.02
            odom.twist.covariance[0] = 0.04
            odom.twist.covariance[7] = 0.04
            odom.twist.covariance[35]= 0.05

            self.odom_pub.publish(odom)
        
        # Store current state for next velocity calculation
        self.prev_stamp = stamp
        self.prev_pose = (x, y, yaw)

    def receive_and_process(self):
        try:
            # 1. Receive and decode frame
            packed_data, _ = sock.recvfrom(BUFFER_SIZE)
            data = pickle.loads(packed_data)
            encoded_frame = data['frame']
            cropped_frame = cv2.imdecode(encoded_frame, cv2.IMREAD_COLOR)
            stamp = self.get_clock().now().to_msg()

            # 2. Apply Perspective Transform for visualization
            transformed_frame = cv2.warpPerspective(cropped_frame, self.M, (self.dst_width, self.dst_height))
            
            # 3. Pre-process for ArUco detection
            gray_frame = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=8.0, tileGridSize=(12,12))
            gray_frame = clahe.apply(gray_frame)
            gray_frame = cv2.bilateralFilter(gray_frame, 9, 100, 50)
            
            # 4. Detect ArUco markers
            aruco_corners_cropped, aruco_ids, _ = self.aruco_detector.detectMarkers(gray_frame) # Use gray_frame 
           
            display_frame = transformed_frame.copy()

            pose_updated = False
            if aruco_ids is not None:
                # Find marker with ID 1
                marker_id_to_find = 2
                try:
                    marker_index = np.where(aruco_ids.flatten() == marker_id_to_find)[0][0]
                except IndexError:
                    marker_index = -1 # Not found

                # Draw all detected markers for visualization
                transformed_corners_all = [cv2.perspectiveTransform(c.reshape(-1, 1, 2), self.M) for c in aruco_corners_cropped]
                cv2.aruco.drawDetectedMarkers(display_frame, transformed_corners_all, aruco_ids)

                if marker_index != -1:
                    # 5. Estimate 3D pose using solvePnP for the specific marker
                    crop_x_min, crop_y_min = data['offsets']
                    
                    # Define 3D object points for the marker
                    obj_points = np.array([
                        [-self.marker_length/2,  self.marker_length/2, 0],
                        [ self.marker_length/2,  self.marker_length/2, 0],
                        [ self.marker_length/2, -self.marker_length/2, 0],
                        [-self.marker_length/2, -self.marker_length/2, 0]
                    ], dtype=np.float32)

                    # Get corresponding 2D image points in full frame coordinates
                    corners_cropped = aruco_corners_cropped[marker_index]
                    corners_full_frame = corners_cropped + np.array([crop_x_min, crop_y_min], dtype=np.float32)
                    
                    # Use solvePnP with IPPE_SQUARE for robust pose estimation
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points,
                        corners_full_frame.reshape(-1, 2),
                        self.camera_matrix,
                        self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )

                    # Z-axis flip check (marker must be in front of the camera)
                    if success and tvec[2][0] > 0:
                        # 6. Calculate 2D Pose
                        # Position from tvec
                        tvec_flat = tvec.flatten().astype(float)
                        ax = float(tvec_flat[ARUCO_XY_AXES[0]])
                        ay = float(tvec_flat[ARUCO_XY_AXES[1]])
                        x, y = convert_position(ax, ay)

                        # Yaw from 2D corner points (top-left -> top-right)
                        direction_vec = corners_cropped.reshape((4, 2))[1] - corners_cropped.reshape((4, 2))[0]
                        yaw_cam = math.atan2(direction_vec[1], direction_vec[0])
                        yaw = convert_yaw(yaw_cam)
                        
                        # 7. Publish TF and Odometry
                        self._publish_tf_and_odom(stamp, x, y, yaw)
                        pose_updated = True

                        # 8. Visualization
                        # Draw axis for the specific marker (ID 1)
                        axis_len = self.marker_length * 0.5
                        axis_points_3d = np.float32([[0,0,0], [axis_len,0,0], [0,axis_len,0], [0,0,axis_len]]).reshape(-1, 3)
                        
                        img_pts, _ = cv2.projectPoints(axis_points_3d, rvec, tvec, self.camera_matrix, self.dist_coeffs)
                        img_pts_relative = img_pts.reshape(-1, 2) - np.array([crop_x_min, crop_y_min], dtype=np.float32)
                        transformed_axis_pts = cv2.perspectiveTransform(img_pts_relative.reshape(-1, 1, 2), self.M)
                        pts = transformed_axis_pts.reshape(-1, 2).astype(int)
                        cv2.line(display_frame, tuple(pts[0]), tuple(pts[1]), (0,0,255), 2) # X-axis (Red)
                        cv2.line(display_frame, tuple(pts[0]), tuple(pts[2]), (0,255,0), 2) # Y-axis (Green)
                        # cv2.line(display_frame, tuple(pts[0]), tuple(pts[3]), (255,0,0), 2) # Z-axis (Blue)

                        pose_text = f"Pose: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f} deg"
                        cv2.putText(display_frame, pose_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            if not pose_updated:
                # If no valid marker detected, republish the last known pose
                if self.prev_pose is not None:
                    x, y, yaw = self.prev_pose
                    self._publish_tf_and_odom(stamp, x, y, yaw)

            cv2.imshow('ArUco Receiver', display_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                raise KeyboardInterrupt

        except socket.timeout:
            self.get_logger().warn("Socket timeout, no data received.", throttle_duration_sec=5)
        except Exception as e:
            self.get_logger().error(f"Error in ArUco processing: {e}")

def main(args=None):
    rclpy.init(args=args)
    aruco_receiver_node = ArucoReceiverNode()
    try:
        rclpy.spin(aruco_receiver_node)
    except KeyboardInterrupt:
        print("Shutting down ArUco receiver.")
    finally:
        aruco_receiver_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        sock.close()

if __name__ == '__main__':
    main()
