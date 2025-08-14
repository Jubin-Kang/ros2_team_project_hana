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

# --- 2D Pose Estimation Parameters ---
SIMILARITY_S = 1.1731463819258798
SIMILARITY_R = np.array([[ 0.02926172, -0.99957178],
                         [-0.99957178, -0.02926172]])
SIMILARITY_T = np.array([-0.00178071,  0.01315473])
THETA_RAD    = math.radians(-88.32318748498734)
DET_R_SIGN   = -1
ARUCO_XY_AXES = (0, 1)

# --- Utility Functions ---
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

# --- Quality Evaluation Functions (from original clahe script) ---
def _evaluate_image_quality(img):
    """이미지 품질 평가 (대비도, 선명도)"""
    # 대비도 계산
    contrast = img.std() / 255.0
    
    # 라플라시안을 사용한 선명도 계산
    laplacian_var = cv2.Laplacian(img, cv2.CV_64F).var() / 10000.0
    
    # 히스토그램 분산 (동적 범위)
    hist = cv2.calcHist([img], [0], None, [256], [0, 256])
    hist_quality = np.std(hist) / np.mean(hist) if np.mean(hist) > 0 else 0
    
    return min(1.0, (contrast + min(laplacian_var, 1.0) + min(hist_quality/100, 0.5)) / 2.5)

def _evaluate_corner_accuracy(corners, img):
    """코너점 정확성 평가"""
    quality_score = 0.0
    
    for corner in corners:
        x, y = int(corner[0]), int(corner[1])
        
        # 경계 체크
        if x < 5 or y < 5 or x >= img.shape[1]-5 or y >= img.shape[0]-5:
            continue
            
        # 코너 주변의 그래디언트 강도 계산
        roi = img[y-2:y+3, x-2:x+3]
        if roi.size > 0:
            grad_x = cv2.Sobel(roi, cv2.CV_64F, 1, 0, ksize=3)
            grad_y = cv2.Sobel(roi, cv2.CV_64F, 0, 1, ksize=3)
            gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2).mean()
            quality_score += min(1.0, gradient_magnitude / 100.0)
            
    return quality_score / len(corners)

def _evaluate_geometric_consistency(corners):
    """기하학적 일관성 평가 (정사각형 모양 유지)"""
    if len(corners) != 4:
        return 0.0
        
    # 변의 길이 계산
    sides = []
    for i in range(4):
        p1 = corners[i]
        p2 = corners[(i+1) % 4]
        side_length = np.linalg.norm(p2 - p1)
        sides.append(side_length)
    
    # 변의 길이 일관성
    side_consistency = 1.0 - (np.std(sides) / np.mean(sides)) if np.mean(sides) > 0 else 0.0
    
    # 대각선 길이 비율 (정사각형이면 대각선이 같아야 함)
    diag1 = np.linalg.norm(corners[2] - corners[0])
    diag2 = np.linalg.norm(corners[3] - corners[1])
    diag_ratio = min(diag1, diag2) / max(diag1, diag2) if max(diag1, diag2) > 0 else 0.0
    
    # 내각 평가 (정사각형이면 모두 90도)
    angles = []
    for i in range(4):
        p1 = corners[(i-1) % 4]
        p2 = corners[i]
        p3 = corners[(i+1) % 4]
        
        v1 = p1 - p2
        v2 = p3 - p2
        
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        cos_angle = np.clip(cos_angle, -1, 1)
        angle = np.arccos(cos_angle)
        angles.append(abs(angle - np.pi/2))  # 90도에서 벗어난 정도
    
    angle_consistency = 1.0 - (np.mean(angles) / (np.pi/4))  # 정규화
    
    return (side_consistency + diag_ratio + max(0, angle_consistency)) / 3.0

def _evaluate_marker_size(corners):
    """마커 크기의 적절성 평가"""
    # 마커 영역 계산
    area = cv2.contourArea(corners)
    
    # 적절한 크기 범위 (너무 작거나 크면 부정확할 수 있음)
    min_area, max_area = 500, 50000  # 픽셀 단위
    
    if area < min_area:
        return area / min_area
    elif area > max_area:
        return max_area / area
    else:
        return 1.0

def _evaluate_marker_orientation(corners):
    """마커 방향성 평가"""
    # 마커의 주축 방향 계산
    center = np.mean(corners, axis=0)
    
    # 첫 번째 코너에서 중심으로의 벡터
    main_vector = corners[0] - center
    
    # 벡터의 길이가 충분한지 확인
    if np.linalg.norm(main_vector) < 5:
        return 0.5  # 너무 작은 마커
        
    # 방향 일관성 (마커가 너무 기울어지지 않았는지)
    angle = np.arctan2(main_vector[1], main_vector[0])
    
    # 수평/수직에 가까우면 더 좋은 점수
    normalized_angle = abs(angle % (np.pi/2))
    orientation_score = 1.0 - (min(normalized_angle, np.pi/2 - normalized_angle) / (np.pi/4))
    
    return max(0.0, orientation_score)

def _calculate_detection_quality(aruco_corners, aruco_ids, enhanced_frame):
    """
    마커 인식의 정확성을 평가하는 종합 품질 점수 계산
    
    평가 기준:
    1. 검출된 마커 개수 (기본점수)
    2. 코너 점의 정확성 (subpixel refinement 품질)
    3. 마커의 기하학적 일관성 (정사각형 모양 유지 정도)
    4. 이미지 대비도 및 선명도
    5. 마커 크기의 일관성
    """
    if aruco_ids is None or len(aruco_ids) == 0:
        return 0.0
    
    total_quality = 0.0
    num_markers = len(aruco_ids)
    
    # 이미지 품질 평가
    img_quality = _evaluate_image_quality(enhanced_frame)
    
    for i, corners in enumerate(aruco_corners):
        marker_quality = 0.0
        corner_points = corners.reshape(-1, 2)
        
        # 1. 기본 점수 (검출됨)
        marker_quality += 1.0
        
        # 2. 코너점 정확성 평가 (subpixel 정확도)
        corner_quality = _evaluate_corner_accuracy(corner_points, enhanced_frame)
        marker_quality += corner_quality * 0.3
        
        # 3. 기하학적 일관성 평가 (정사각형 모양)
        geometric_quality = _evaluate_geometric_consistency(corner_points)
        marker_quality += geometric_quality * 0.4
        
        # 4. 마커 크기 평가
        size_quality = _evaluate_marker_size(corner_points)
        marker_quality += size_quality * 0.2
        
        # 5. 마커 방향성 평가
        orientation_quality = _evaluate_marker_orientation(corner_points)
        marker_quality += orientation_quality * 0.1
        
        total_quality += marker_quality
    
    # 이미지 전체 품질 보너스
    total_quality += img_quality * num_markers * 0.2
    
    # 다중 마커 보너스 (여러 마커가 동시에 잘 검출되면 추가 점수)
    if num_markers > 1:
        total_quality += (num_markers - 1) * 0.1
        
    return total_quality / max(1, num_markers)  # 마커당 평균 품질

def find_optimal_clahe_params():
    """
    Receives frames and tests different CLAHE parameters
    to find the set that maximizes ArUco detection quality.
    """
    # --- LOGGING SETUP ---
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"log_clahe_params_raw_{timestamp}.txt"
    original_stdout = sys.stdout
    log_file = open(log_filename, 'w')
    sys.stdout = log_file

    print(f"--- Starting CLAHE Parameter Optimization (Raw Image) ---")
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

        # --- Hardcoded Camera Parameters ---
        camera_matrix = np.array([[1185.96684, 0, 999.31995],
                                  [0, 890.7003, 569.28861],
                                  [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.array([-9.413361e-02, -8.374589e-04, 3.176887e-04, -3.987077e-04, 3.289896e-03, 0.0, 0.0, 0.0], dtype=np.float32)
        marker_length = 0.1

        # --- ArUco Detector Initialization (with optimal params from aruco_params.py) ---
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_250)
        aruco_params = cv2.aruco.DetectorParameters()
        aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG # From aruco_params.py best
        aruco_params.cornerRefinementWinSize = 30 # From aruco_params.py best
        aruco_params.cornerRefinementMaxIterations = 70 # From aruco_params.py best
        aruco_params.cornerRefinementMinAccuracy = 0.001 # From aruco_params.py best
        aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        # --- CLAHE Parameter Search Ranges ---
        clahe_clip_limits = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 7.0, 8.0]
        clahe_tile_grid_sizes = [(4,4), (6,6), (8,8), (10,10), (12,12), (16,16)]

        # --- Experiment settings ---
        FRAMES_PER_TEST = 100 # Increased for better statistical significance
        best_params = {}
        highest_quality_score = -float('inf')

        param_combinations = list(itertools.product(clahe_clip_limits, clahe_tile_grid_sizes))
        print(f"Starting CLAHE parameter search. Testing {len(param_combinations)} combinations.")
        print("IMPORTANT: For best results, keep the camera and ArUco marker STATIONARY during the test.")

        for i, (clipLimit, tileGridSize) in enumerate(param_combinations):
            print(f"\n--- Testing combination {i+1}/{len(param_combinations)}: ClipLimit={clipLimit}, TileGridSize={tileGridSize} ---")

            quality_scores = []
            frames_collected = 0

            while frames_collected < FRAMES_PER_TEST:
                try:
                    sock.settimeout(5.0) # Set a timeout for receiving data
                    packed_data, _ = sock.recvfrom(BUFFER_SIZE)
                    data = pickle.loads(packed_data)
                    cropped_frame = cv2.imdecode(data['frame'], cv2.IMREAD_COLOR)
                    crop_x_min, crop_y_min = data['offsets']

                    if cropped_frame is None:
                        print("Failed to decode frame, skipping.")
                        continue

                    gray_frame = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)
                    clahe = cv2.createCLAHE(clipLimit=clipLimit, tileGridSize=tileGridSize)
                    enhanced_frame = clahe.apply(gray_frame)

                    aruco_corners_cropped, aruco_ids, _ = aruco_detector.detectMarkers(enhanced_frame)

                    # Calculate quality score
                    quality_score = _calculate_detection_quality(aruco_corners_cropped, aruco_ids, enhanced_frame)
                    quality_scores.append(quality_score)
                    frames_collected += 1
                    print(f"\rCollected frames: {frames_collected}/{FRAMES_PER_TEST} (Current Quality: {quality_score:.4f})", end="")

                except socket.timeout:
                    print("\nSocket timeout. Make sure the stream is active. Moving to next combination.")
                    break
                except Exception as e:
                    print(f"\nAn error occurred during frame processing: {e}. Skipping frame.")
                    continue
            
            if len(quality_scores) > 0:
                avg_quality_score = np.mean(quality_scores)
                print(f"\nAverage Quality Score for this combination: {avg_quality_score:.6f}")

                if avg_quality_score > highest_quality_score:
                    highest_quality_score = avg_quality_score
                    best_params = {
                        'clipLimit': clipLimit,
                        'tileGridSize': tileGridSize
                    }
                    print(f"*** New best parameters found! ***")
            else:
                print("\nNo data collected for this combination.")

        print("\n\n===================================================")
        print("CLAHE Parameter search complete.")
        if best_params:
            print(f"Highest average quality score: {highest_quality_score:.6f}")
            print(f"Optimal CLAHE parameters found: {best_params}")
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
    find_optimal_clahe_params()
