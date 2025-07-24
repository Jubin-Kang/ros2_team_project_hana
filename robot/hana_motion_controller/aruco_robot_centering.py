import requests
import time
import numpy as np
from pymycobot.mycobot import MyCobot  # pymycobot 3.5.3 기준

# 로봇팔 초기화
mc = MyCobot('/dev/ttyUSB0', 1000000)
print("로봇팔 연결됨")

# Flask 서버 주소 (ArUco 마커 Pose 제공)
FLASK_URL = "http://192.168.0.161:5000/pose"

# 파라미터 설정
SCALE = 1000           # m → mm 변환
STEP = 10              # XY 최대 이동량(mm)
Z_STEP = 10            # Z 최대 이동량(mm)
THRESHOLD = 0.003      # XY 오차 허용 (3mm)
Z_THRESHOLD = 0.01     # Z 오차 허용 (1cm)
SAFE_DIST = 0.1        # 카메라-마커 목표 거리 (0.1m)
SPEED = 50             # 이동 속도
base_z = 256.6         # 기준 Z 높이

# 그리퍼 위치 오프셋 (단위: mm)
GRIPPER_FORWARD_OFFSET = 42    # 앞으로 4.5cm
GRIPPER_DOWN_OFFSET = 55       # 아래로 5.5cm

while True:
    try:
        # 현재 위치 저장
        current_coords = mc.get_coords()
        base_x, base_y, base_z_now = current_coords[0], current_coords[1], current_coords[2]

        # Flask 서버에서 tvec 받기
        res = requests.get(FLASK_URL).json()
        tvec = res.get("tvec")

        if tvec:
            offset_x, offset_y, offset_z = tvec[0], tvec[1], tvec[2]
            print(f"[Tvec] x: {offset_x:.4f}, y: {offset_y:.4f}, z: {offset_z:.4f}")

            move_required = False

            # 중심 정렬: X
            if abs(offset_x) > THRESHOLD:
                step_x = np.clip(offset_x * SCALE, -STEP, STEP)
                target_x = base_x + step_x
                move_required = True
            else:
                target_x = base_x

            # 중심 정렬: Y (좌우 반전 고려)
            if abs(offset_y) > THRESHOLD:
                step_y = np.clip(-offset_y * SCALE, -STEP, STEP)
                target_y = base_y + step_y
                move_required = True
            else:
                target_y = base_y

            # 거리 보정: Z
            if abs(offset_z - SAFE_DIST) > Z_THRESHOLD:
                delta_z = np.clip((offset_z - SAFE_DIST) * SCALE, -Z_STEP, Z_STEP)
                target_z = base_z_now - delta_z
                move_required = True
            else:
                target_z = base_z_now

            if move_required:
                print(f"[Move] → X: {target_x:.1f}, Y: {target_y:.1f}, Z: {target_z:.1f}")
                mc.send_coords([target_x, target_y, target_z, 180, 0, 0], SPEED)
                time.sleep(0.3)
            else:
                aligned = (
                    abs(offset_x) <= THRESHOLD and
                    abs(offset_y) <= THRESHOLD and
                    abs(offset_z - SAFE_DIST) <= Z_THRESHOLD
                )
                print(f"정렬 상태 확인 → aligned = {aligned}")

                if aligned:
                    print("중심 정렬 및 거리 정확히 일치 → Pick & Place 수행")
                    time.sleep(2.0)

                    # Pick
                    pick_coords = [
                        base_x,
                        base_y + GRIPPER_FORWARD_OFFSET,
                        base_z_now - GRIPPER_DOWN_OFFSET,
                        180, 0, 0
                    ]
                    mc.send_coords(pick_coords, SPEED)
                    time.sleep(2.0)

                    mc.set_gripper_value(50, 50)  # 닫기
                    time.sleep(2.0)

                    # 들어올리기
                    lift_coords = pick_coords.copy()
                    lift_coords[2] += 100
                    mc.send_coords(lift_coords, SPEED)
                    time.sleep(1.0)

                    '''# Place (오른쪽 13cm)
                    place_coords = [
                        base_x + 130,
                        base_y + GRIPPER_FORWARD_OFFSET,
                        base_z_now,
                        180, 0, 0
                    ]
                    mc.send_coords(place_coords, SPEED)
                    time.sleep(1.5)
                    
                    # 들어올린 만큼 다시 내리기
                    place_coords_down = place_coords.copy()
                    place_coords_down[2] -= 30  # Z축 30mm 다시 내림
                    mc.send_coords(place_coords_down, SPEED)
                    time.sleep(1.0)'''
                    
                    mc.send_angles([0, 0, 0, 0, 0, 0], 50)
                    time.sleep(2)
                    
                    mc.send_angles([0, -60, -25, 0, 0, 0], 50)
                    time.sleep(1.0)
                    
                    mc.set_gripper_value(100, 50)  # 열기
                    time.sleep(1.0)
                    
                                       
                    '''# 살짝 들어올리기 (충돌 방지)
                    place_coords_up = place_coords_down.copy()
                    place_coords_up[2] += 100
                    mc.send_coords(place_coords_up, SPEED)
                    time.sleep(0.8)'''

                    # 복귀
                    mc.send_angles([0, 0, 0, 0, 0, 0], 50)
                    time.sleep(2)
                    
                    print("Pick & Place 완료\n")
                    time.sleep(3.0)
                    break

                else:
                    print("정렬 또는 거리 조건 미충족 → 대기 중")
                    time.sleep(0.5)

        else:
            print("마커 인식 안됨")
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("종료됨")
        break
