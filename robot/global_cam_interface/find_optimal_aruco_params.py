import cv2
import numpy as np
import socket
import pickle
import math
import itertools
import sys
from datetime import datetime

# --- UDP Multicast Configuration ---
MCAST_GRP = '224.1.1.1'
MCAST_PORT = 5000
BUFFER_SIZE = 65536

# --- Utility Functions (from original script) ---
def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi

def convert_position(ax: float, ay: float, SIMILARITY_S, SIMILARITY_R, SIMILARITY_T) -> tuple[float, float]:
    p = np.array([ax, ay])
    mp = (SIMILARITY_R @ p) * SIMILARITY_S + SIMILARITY_T
    return float(mp[0]), float(mp[1])

def convert_yaw(yaw_cam_rad: float, THETA_RAD, DET_R_SIGN) -> float:
    if DET_R_SIGN >= 0:
        return wrap(yaw_cam_rad + THETA_RAD)
    else:
        return wrap(-yaw_cam_rad + THETA_RAD)

def find_optimal_params():
    """
    Receives frames and tests different ArUco detector parameters
    to find the set that minimizes pose jitter on the raw grayscale image.
    """
    # --- LOGGING SETUP ---
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"log_aruco_params_raw_{timestamp}.txt"
    original_stdout = sys.stdout
    log_file = open(log_filename, 'w')
    sys.stdout = log_file

    print(f"--- Starting ArUco Parameter Optimization (Raw Image) ---")
    print(f"Log file: {log_filename}")
    print(f"Timestamp: {timestamp}\n")

    try:
        # --- Setup socket ---
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', MCAST_PORT))
        mreq = socket.inet_aton(MCAST_GRP) + socket.inet_aton('0.0.0.0')
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        print(f"Listening for frames on {MCAST_GRP}:{MCAST_PORT}")

        # --- Hardcoded parameters from the original script ---
        SIMILARITY_S = 1.1731463819258798
        SIMILARITY_R = np.array([[ 0.02926172, -0.99957178],
                                 [-0.99957178, -0.02926172]])
        SIMILARITY_T = np.array([-0.00178071,  0.01315473])
        THETA_RAD    = math.radians(-88.32318748498734)
        DET_R_SIGN   = -1
        ARUCO_XY_AXES = (0, 1)
        camera_matrix = np.array([[1185.96684, 0, 999.31995],
                                  [0, 890.7003, 569.28861],
                                  [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.array([-9.413361e-02, -8.374589e-04, 3.176887e-04, -3.987077e-04, 3.289896e-03, 0.0, 0.0, 0.0], dtype=np.float32)
        marker_length = 0.1

        # --- ARUCO PARAMETER SEARCH RANGES ---
        method_range = {
            "SUBPIX": cv2.aruco.CORNER_REFINE_SUBPIX,
            "CONTOUR": cv2.aruco.CORNER_REFINE_CONTOUR,
            "APRILTAG": cv2.aruco.CORNER_REFINE_APRILTAG
        }
        win_size_range = [10, 20, 30]
        max_iter_range = [30, 50, 70]
        min_acc_range = [0.1, 0.01, 0.001]

        # --- Experiment settings ---
        FRAMES_PER_TEST = 100
        best_params = {}
        lowest_jitter_score = float('inf')

        param_combinations = list(itertools.product(method_range.items(), win_size_range, max_iter_range, min_acc_range))
        print(f"Starting ArUco parameter search. Testing {len(param_combinations)} combinations.")
        print("IMPORTANT: For best results, keep the camera and ArUco marker STATIONARY during the test.")

        for i, ((method_name, method_val), win_size, max_iter, min_acc) in enumerate(param_combinations):
            print(f"\n--- Testing combination {i+1}/{len(param_combinations)}: Method={method_name}, WinSize={win_size}, MaxIter={max_iter}, MinAcc={min_acc} ---")

            # Configure ArUco Detector for this test
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_250)
            aruco_params = cv2.aruco.DetectorParameters()
            aruco_params.cornerRefinementMethod = method_val
            aruco_params.cornerRefinementWinSize = win_size
            aruco_params.cornerRefinementMaxIterations = max_iter
            aruco_params.cornerRefinementMinAccuracy = min_acc
            aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

            poses = []
            frames_collected = 0

            while frames_collected < FRAMES_PER_TEST:
                try:
                    sock.settimeout(5.0)
                    packed_data, _ = sock.recvfrom(BUFFER_SIZE)
                    data = pickle.loads(packed_data)
                    cropped_frame = cv2.imdecode(data['frame'], cv2.IMREAD_COLOR)
                    crop_x_min, crop_y_min = data['offsets']

                    gray_frame = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)

                    aruco_corners_cropped, aruco_ids, _ = aruco_detector.detectMarkers(gray_frame)

                    if aruco_ids is not None:
                        try:
                            marker_index = np.where(aruco_ids.flatten() == 1)[0][0]
                        except IndexError:
                            continue

                        obj_points = np.array([
                            [-marker_length/2,  marker_length/2, 0], [marker_length/2,  marker_length/2, 0],
                            [marker_length/2, -marker_length/2, 0], [-marker_length/2, -marker_length/2, 0]
                        ], dtype=np.float32)
                        
                        corners_cropped = aruco_corners_cropped[marker_index]
                        corners_full_frame = corners_cropped + np.array([crop_x_min, crop_y_min], dtype=np.float32)
                        
                        success, rvec, tvec = cv2.solvePnP(
                            obj_points, corners_full_frame.reshape(-1, 2),
                            camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
                        )

                        if success and tvec[2][0] > 0:
                            tvec_flat = tvec.flatten().astype(float)
                            ax = float(tvec_flat[ARUCO_XY_AXES[0]])
                            ay = float(tvec_flat[ARUCO_XY_AXES[1]])
                            x, y = convert_position(ax, ay, SIMILARITY_S, SIMILARITY_R, SIMILARITY_T)

                            direction_vec = corners_cropped.reshape((4, 2))[1] - corners_cropped.reshape((4, 2))[0]
                            yaw_cam = math.atan2(direction_vec[1], direction_vec[0])
                            yaw = convert_yaw(yaw_cam, THETA_RAD, DET_R_SIGN)
                            
                            poses.append([x, y, yaw])
                            frames_collected += 1
                            print(f"\rCollected frames: {frames_collected}/{FRAMES_PER_TEST}", end="")

                except socket.timeout:
                    print("\nSocket timeout. Make sure the stream is active.")
                    break
                except Exception as e:
                    print(f"\nAn error occurred: {e}")
                    continue
            
            if len(poses) > 10:
                poses_arr = np.array(poses)
                std_dev = np.std(poses_arr, axis=0)
                jitter_score = np.sum(std_dev)
                print(f"\nJitter Score: {jitter_score:.6f} (x: {std_dev[0]:.4f}, y: {std_dev[1]:.4f}, yaw: {std_dev[2]:.4f})")

                if jitter_score < lowest_jitter_score:
                    lowest_jitter_score = jitter_score
                    best_params = {
                        'method': method_name,
                        'win_size': win_size,
                        'max_iter': max_iter,
                        'min_acc': min_acc
                    }
                    print(f"*** New best parameters found! ***")
            else:
                print("\nNot enough data collected for this combination.")

        print("\n\n===================================================")
        print("Parameter search complete.")
        if best_params:
            print(f"Lowest jitter score: {lowest_jitter_score:.6f}")
            print(f"Optimal parameters found: {best_params}")
        else:
            print("Could not determine optimal parameters.")
        print("===================================================")
        
        sock.close()

    finally:
        # --- CLEANUP ---
        print("\nRestoring stdout and closing log file.")
        sys.stdout = original_stdout
        log_file.close()

if __name__ == '__main__':
    find_optimal_params()