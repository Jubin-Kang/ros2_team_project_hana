# web/utils.py  (경로: proj_root/web/utils.py)
import config as cfg

'''---------------------------------------------------------------'''
# 방문자 IP 확인 함수
from flask import Request

def get_client_ip(req: Request) -> str:
    """
    프록시/로드밸런서를 거친 경우를 포함해
    실제 클라이언트 IP를 반환.
    """
    if (xff := req.environ.get("HTTP_X_FORWARDED_FOR")):
        # 'A, B, C' 형태일 때 첫 번째 IP가 원본
        return xff.split(",")[0].strip()
    return req.remote_addr or "0.0.0.0"

'''---------------------------------------------------------------'''
#!/usr/bin/env python3
import socket
import cv2
import numpy as np
import threading
import queue
import struct

def run_udp_stream_display(udp_host= cfg.GLOBAL_CAMERA_IP, udp_port=cfg.GLOBAL_CAMERA_PORT):
    """UDP 멀티캐스트 스트림을 수신하고 프레임을 디코딩합니다."""
    # --- UDP 소켓 설정 ---
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', udp_port))
    mreq = struct.pack("4sl", socket.inet_aton(udp_host), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    print(f"UDP 멀티캐스트 수신 대기 중: {udp_host}:{udp_port}")

    # --- 큐 및 스레드 설정 ---
    frame_queue = queue.Queue(maxsize=2)
    def receive_frames():
        """UDP 패킷을 수신하여 큐에 넣는 내부 스레드 함수"""
        try:
            while True:
                try:
                    data, _ = sock.recvfrom(65536)
                    if not frame_queue.full():
                        frame_queue.put(data)
                except Exception as e:
                    print(f"프레임 수신 중 오류 발생: {e}")
        finally:
            sock.close()

    receive_thread = threading.Thread(target=receive_frames, daemon=True)
    receive_thread.start()

    # --- 프레임 처리 ---
    try:
        while True:
            try:
                data = frame_queue.get_nowait()
                # 항상 최신 프레임을 처리하기 위해 큐를 비움
                while not frame_queue.empty():
                    data = frame_queue.get_nowait()

                np_arr = np.frombuffer(data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if frame is None:
                    print("프레임 디코딩에 실패했습니다.")
                    continue

                # --- BGR을 RGB로 변환 (필요 시 활용 가능) ---
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                # --- 창 표시 제거, 프레임 처리만 수행 ---
                # frame_rgb를 여기서 추가 처리 가능 (예: 저장, 분석 등)

            except queue.Empty:
                continue  # 처리할 프레임이 없으면 계속
            except Exception as e:
                print(f"프레임 처리 중 오류 발생: {e}")

    except KeyboardInterrupt:
        print("키보드 인터럽트로 프로그램을 종료합니다.")
    finally:
        pass
