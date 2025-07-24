from flask import Flask, Response, jsonify
import cv2
import numpy as np

app = Flask(__name__)
cap = cv2.VideoCapture("/dev/jetcocam0")

camera_matrix = np.load("/home/jetcobot/dev_ws/src/calibration_matrix.npy")
dist_coeffs = np.load("/home/jetcobot/dev_ws/src/distortion_coefficients.npy")
marker_length = 0.02  # 마커 한 변 길이 [m]

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

latest_tvec = None  # 마지막 인식된 마커 위치 저장

def generate_frames():
    global latest_tvec
    while True:
        success, frame = cap.read()
        if not success:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
            for rvec, tvec in zip(rvecs, tvecs):
                latest_tvec = tvec[0].tolist()
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_length * 0.5)

        _, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def index():
    return "<h1>JetCobot ArUco Stream</h1><img src='/video_feed'>"

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/pose')
def pose():
    if latest_tvec:
        return jsonify({"tvec": latest_tvec})
    else:
        return jsonify({"tvec": None})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
