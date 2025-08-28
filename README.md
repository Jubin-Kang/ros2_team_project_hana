# roscamp-repo-1
ROS2와 AI를 활용한 자율주행 로봇개발자 부트캠프 1팀 저장소. AI 기반 요양 케어 자동화 시스템

<br />
<p align="center">
  <a href="https://github.com/addinedu-roscamp-5th/roscamp-repo-1">
    <img src="https://github.com/user-attachments/assets/a59ce405-fc45-4c24-baba-a17441084595" alt="Logo" width="620px">
  </a>

  <h3 align="center">HANA (Human-AI Nursing Assistant)</h3>
  <p align="center">
    <a href="https://www.canva.com/design/DAGwfz6HCGg/3tjJYCF4NwBcRcOnqCbbjg/edit">Presentation</a>
    <a> || </a>
    <a href="https://youtu.be/ZZ3zroNBlqo?si=c7IFToJwzir5f4Py">Video Demo</a>
  </p>
</p>
<hr>

> **HANA** 팀이 개발한, AI 기반 요양 케어 자동화 시스템입니다. 

---

## 프로젝트 개요

본 프로젝트는 자율주행 로봇과 로봇팔, AI 비전 기술을 융합하여 요양원의 돌봄 품질을 향상하며, 요양 보호사의 업무 부담을 경감하는 AI 기반 요양 케어 자동화 시스템을 구현합니다.
이를 통해 안전하고 신속한 돌봄 서비스와 시설 운영 효율성을 동시에 달성합니다.

- **프로젝트명**: HANA (Human-AI Nursing Assistant)
- **팀명**: T1
- **주제**: AI 기반 요양 케어 자동화 시스템
- **핵심 기술**: ROS2, YOLO, OpenCV, Azure, Fusion360, TCP/UDP

## 팀 구성 및 역할
|        | Name | Job |
|--------|------|-----|
| Leader | 강주빈 |  Project Manage, Manipulator, Perception |   
| Worker | 권상혁 |  Mobile Robot, Server, DB, GUI |   
| Worker | 김윤복 |  Manipulator |    
| Worker | 김주현 |  Perception, Computer Vision | 
| Worker | 최은정 |  Multi-Robot Control System, 3D Modeling | 

---

## 기술 스택 (Tech Stack)
| Category | Technology |
|----------|------------|
| Development Environment	| Linux, Ubuntu 24.04 LTS, ROS2 (Jazzy), Python venv |
| Language | Python |
| Framework |	Nav2, YOLO, OpenCV, Fusion360, Azure |
| Network |	TCP/IP, UDP/IP |
| Configuration Management | Github, Jira, Confluence, Slack |


---

## System Architecture
![Image](https://github.com/user-attachments/assets/e4ae8608-5f9b-4ebd-b7c2-dd7db8775f8c)

<br >

## Data Structure
![Image](https://github.com/user-attachments/assets/9c227d2b-735e-447f-b443-378c764ac177)

<br >

## Interface Specification
![Image](https://github.com/user-attachments/assets/5980b5cf-c713-46f2-b932-a4a1e79b41f0)
![Image](https://github.com/user-attachments/assets/1ad8dcbe-bb60-4604-b8e8-d1728766a4c6)
![Image](https://github.com/user-attachments/assets/77c768c6-cb1d-4fd6-8569-78f131175441)
![Image](https://github.com/user-attachments/assets/86d3a779-1ed6-48c7-a054-6efef0a5ca3b)

<br >

## Sequence Diagram
![Image](https://github.com/user-attachments/assets/93c2a772-3cf0-4064-bf61-013953c38a1c)
![Image](https://github.com/user-attachments/assets/d79edd55-8f75-4c25-a874-2bc3c725babd)
![Image](https://github.com/user-attachments/assets/5cad0199-bd49-4cfc-ba54-cded051bed52)
![Image](https://github.com/user-attachments/assets/dd78f1ed-0975-4409-b1b6-00f302927d9a)
![Image](https://github.com/user-attachments/assets/eb78f408-9201-4f79-a414-2cef01e72076)

<br >

## Map
![Image](https://github.com/user-attachments/assets/70f208c6-8202-43f4-b70c-6ff377562111)

<br >

## 3D Modeling
![Image](https://github.com/user-attachments/assets/3137c0dc-7103-48db-94c8-5b9c5eff2d0a)

<br >

## Implements
### Scenario
![Image](https://github.com/user-attachments/assets/34e3c718-48e7-46f7-8f9f-0aa2894e73d4)
![Image](https://github.com/user-attachments/assets/36888708-8558-4cf7-ab7f-1323dec34a75)

## Visual Perception
### Fall Detection <a href="https://youtu.be/RGaHGk8g8CM?si=JeyQEyDHIPCZi-QU">Video Demo</a>
![Image](docs/global_camera_0_fall_detection.png)

<br >


### ArUco Marker Detection
![Image](docs/global_camera_1_aruco_detection.png)
![Image](docs/global_camera_2_aruco_result.png)
![Image](docs/global_camera_3_aruco_distribution_result.png)
<br >

## Mobile Robots
### Autonomous Driving Architecture
![Image](docs/mobile_robot_0_architecture.png) 
### Aruco Marker Odometry with Sensor Fusion
![Image](docs/mobile_robot_1_aruco_marker_odom.png)
![Image](docs/mobile_robot_2_aruco_marker_ekf.png)
![Image](docs/mobile_robot_3_aruco_marker_odom_result.png)

### Mobile Robot Control
![Image](docs/mobile_robot_4_precise_control.png)
![Image](docs/mobile_robot_5_safety_control.png)

<br >

## Multi-Robot Control System
### ???


<br >

## Manipulator
### TCP vs. UDP <a href="https://youtube.com/shorts/jgOKWCgC6g4?si=p_9TIRYr5YMuHrkH">Video Demo</a>
![Image](https://github.com/user-attachments/assets/e2ac3703-e394-4f92-ad52-86c038c69b65)
![Image](https://github.com/user-attachments/assets/1190fe8f-aede-45a5-b1e8-a516d5a99e50)

### 물품 탐색 및 분류 후 Pick & Place <a href="https://youtube.com/shorts/nGim0zZXDp8?si=m3Fh_BKGykL91u_g">Video Demo</a>
![Image](https://github.com/user-attachments/assets/a4ed023e-5298-4649-aafe-ac842888e21a)


<br >


## GUI
### Client GUI
![Image]()

<br >

### Admin GUI
![Image](docs/gui_admin_0.png)
![Image](docs/gui_admin_1.png)
<br >

## AI 기반 요양 케어 자동화 시스템 <a href="https://youtu.be/ZZ3zroNBlqo?si=c7IFToJwzir5f4Py">Video Demo</a>

<br >

## Project Schedule
Project Period: 2025.06.23~2025.08.22
![Image](https://github.com/user-attachments/assets/64a657e1-f6d0-4e71-a621-926d4976c5ed)

<br >
