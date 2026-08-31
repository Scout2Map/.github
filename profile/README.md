# Scout2Map

**다중 센서 기반 환경 적응형 정찰 UGV와 SLAM 이벤트 맵 및 관제 시스템**

Scout2Map은 재난·재해 현장처럼 사람이 직접 들어가기 위험하거나 판단이 늦어지면 안 되는 공간에, 사람 대신 먼저 들어가 위험 요소를 표시된 지도로 만들어주는 소형 정찰 UGV(무인 지상 차량)와 관제 시스템이다. 로봇 한 대가 SLAM으로 공간을 매핑하는 동시에 온도·가스·조도·미세먼지 같은 환경 값과 AI 비전 인식 결과를 임계값으로 판정해 "이벤트"로 만들고, 관제 화면에는 원시 센서 로그가 아니라 2D 지도 위의 이벤트 마커로 결과가 올라온다. 지휘관(운용자)이 실시간 수치를 계속 들여다보지 않아도, 지도 위 마커만 보고 어디에 무슨 위험이 있는지 즉시 판단할 수 있게 하는 것이 이 프로젝트의 목표다.

---

## 왜 이렇게 만들었나

기존의 일반적인 정찰 로봇 원격 운용 방식은 세 가지 문제를 안고 있다.

첫째, 로봇 자체가 비싸다. 대당 1~3억 원 수준의 정찰 로봇은 재난 현장에서 파손·망실 위험을 감수하고 투입하기 어렵다. Scout2Map은 1.8kg급 경량 차체와 범용 부품으로 훨씬 낮은 비용에 다수를 운용할 수 있게 설계했다.

둘째, 원시 데이터를 그대로 실시간 전송하는 방식은 대역폭을 많이 쓴다. 영상·LiDAR 원시 스트림을 계속 흘려보내면 통신망이 불안정한 재난 현장에서 연결이 끊기기 쉽고, 끊기면 그 순간부터 로봇 상태를 알 수 없다. Scout2Map은 원시 데이터가 아니라 판정이 끝난 이벤트(수백 바이트)와 압축된 지도만 전송해, 평상시~탐사 중 기준 약 0.7~2.0 KB/s 수준으로 대역폭을 크게 절감한다.

셋째, 원시 수치 나열은 즉각적인 위치 판단을 어렵게 만든다. 로그창에 찍히는 숫자보다, 지도 위에 어디서 무엇이 감지됐는지 마커로 보여주는 쪽이 현장 판단 속도를 훨씬 높인다.

---

## 시스템 구조

Scout2Map은 물리적으로 3개의 컨트롤러가 역할을 나눠 맡는다.

```
                         ┌─────────────────────────────────────────┐
                         │              RPi 5 (SBC)                 │
                         │  ROS2 Jazzy · SLAM · Nav2 · 이벤트 엔진   │
                         │  AI Vision · 통신 릴레이 · GPIO 출력      │
                         └───────────────┬───────────────────────────┘
                                         │ USB CDC (시리얼 브릿지)
                ┌────────────────────────┼────────────────────────┐
                │                                                  │
    ┌───────────▼───────────┐                          ┌───────────▼───────────┐
    │   STM32F103 (주행 MCU)  │                          │  RPi Pico 2 (센서 MCU)  │
    │  100Hz 모터 PID 제어    │                          │  환경 센서 폴링/발행     │
    │  BTS7960 · BNO055 IMU  │                          │  AHT21 · ENS160        │
    │  엔코더 · 거리센서       │                          │  BH1750 · PMS7003      │
    └────────────────────────┘                          └────────────────────────┘

                         ┌─────────────────────────────────────────┐
                         │       S2M-CommRelay (WebSocket, 9091)     │
                         │   rosbridge 미사용, 자체 JSON 프로토콜     │
                         └───────────────┬───────────────────────────┘
                                         │
                         ┌───────────────▼───────────────────────────┐
                         │     S2M-Web-Monitoring (React 관제 대시보드) │
                         │  SLAM 지도 · 이벤트 마커 · 텔레메트리 · 제어 │
                         └─────────────────────────────────────────┘
```

RPi 5는 SLAM Toolbox, Nav2, EKF(`robot_localization`)로 위치 추정과 자율 탐사(frontier exploration)를 담당하고, 동시에 이벤트 엔진이 모든 센서·주행·통신·비전 판정을 한곳으로 모아 `/events`를 발행하는 유일한 주체 역할을 한다. STM32는 HAL·RTOS 없이 CMSIS 레지스터 레벨로 직접 구현한 주행 제어 펌웨어로 모터 PID와 오도메트리를 100Hz로 처리하고, RPi Pico 2(RP2350)는 환경 센서 4종을 읽어 USB CDC로 JSON 라인을 내보낸다. 두 MCU 모두 RPi 5의 브릿지 노드가 ROS2 토픽으로 변환하므로, SLAM이나 이벤트 엔진 같은 상위 레이어는 시리얼 프로토콜을 전혀 알 필요가 없다.

관제 측은 `rosbridge`를 쓰지 않는다. `S2M-CommRelay`가 ROS2 토픽(이벤트, 지도, 위치, 텔레메트리)을 자체 JSON-over-WebSocket 프로토콜로 한 방향 중계하고, 지도는 zlib으로 압축해서 보낸다. `S2M-Web-Monitoring`은 이 WebSocket에 브라우저 표준 API로 직접 붙어 지도·마커·상태를 그려내고, E-Stop이나 미션 제어 같은 명령은 반대 방향으로 릴레이를 거쳐 로봇에 전달된다.

---

## 레포지토리 구성

| 레포 | 역할 | 실행 위치 | 주요 기술 |
|---|---|---|---|
| [S2M-Hardware](https://github.com/Scout2Map/S2M-Hardware) | 차체·전장 설계 자료, ROS2/Gazebo 시뮬레이션용 URDF-xacro 모델 | - | Fusion 360, URDF/xacro, Gazebo |
| [S2M-FW-DrivingControl](https://github.com/Scout2Map/S2M-FW-DrivingControl) | 주행 제어 펌웨어: 모터 PID, 오도메트리, IMU 융합 | STM32F103C8T6 | CMSIS(HAL 미사용), BTS7960, BNO055 |
| [S2M-FW-SensorFusion](https://github.com/Scout2Map/S2M-FW-SensorFusion) | 환경 센서 펌웨어: 온습도/가스/조도/미세먼지 폴링 및 JSON 송출 | RPi Pico 2 (RP2350) | pico-sdk, AHT21, ENS160, BH1750, PMS7003 |
| [S2M-MCU-BridgeNode](https://github.com/Scout2Map/S2M-MCU-BridgeNode) | 두 MCU ↔ ROS2 브릿지, RPi5 GPIO 이벤트 출력 노드 | RPi 5 | ROS2(rclpy), pyserial, gpiozero |
| [S2M-Event-Engine](https://github.com/Scout2Map/S2M-Event-Engine) | 센서·주행·통신·비전 데이터를 임계값/ML로 판정해 이벤트를 발행하는 유일한 주체 | RPi 5 | ROS2, scikit-learn(Isolation Forest) |
| [S2M-SBC-Integration](https://github.com/Scout2Map/S2M-SBC-Integration) | 전체 통합/브링업: SLAM, Nav2, EKF, AI Vision, 자율 탐사 | RPi 5 | ROS2 Jazzy, SLAM Toolbox, Nav2, YOLOv8(ONNX) |
| [S2M-CommRelay](https://github.com/Scout2Map/S2M-CommRelay) | ROS2 ↔ 관제 웹 간 자체 WebSocket 프로토콜 릴레이 | RPi 5 | ROS2, WebSocket(JSON + zlib) |
| [S2M-Web-Monitoring](https://github.com/Scout2Map/S2M-Web-Monitoring) | 실시간 관제 대시보드: 지도, 이벤트 마커, 텔레메트리, 미션/드라이브 제어 | 관제 PC/브라우저 | React 19, Canvas, WebSocket |

레포 경계는 "어느 하드웨어에서 실행되는가"를 기준으로 나뉜다. 두 MCU용 펌웨어(`S2M-FW-DrivingControl`, `S2M-FW-SensorFusion`)는 각자 다른 빌드 체계(CMSIS Makefile, pico-sdk)를 쓰므로 독립 레포로 분리했고, RPi 5에서 도는 ROS2 패키지들은 브릿지(`S2M-MCU-BridgeNode`)·판정(`S2M-Event-Engine`)·통합(`S2M-SBC-Integration`)·중계(`S2M-CommRelay`) 역할별로 나눴다.

---

## 이벤트 체계

시스템에서 발생하는 모든 위험 판정은 `S2M-Event-Engine`을 거쳐 `/events`로 발행된다. 최종 보고서가 정의한 8종(고온, 가스 고농도, 저조도, 요철, 슬립 의심, 회전 곤란, 통신 품질 저하, 통신 두절)에 더해, 실제 구현에서는 미세먼지·저온·과다 조도·센서 링크 두절·AI 비전 검출·예측 기반 조기 경보·환경 이상 패턴 탐지(Isolation Forest) 등 11종을 확장 정의해 총 19종의 이벤트를 판정한다. 판정된 이벤트는 지도 위 좌표가 있는 마커로 관제 화면에 표시되며, `S2M-MCU-BridgeNode`의 GPIO 이벤트 노드를 통해 부저·경광등 같은 물리 출력 장치를 이벤트에 직접 연결할 수도 있다.

---

## 기술 스택

| 구분 | 내용 |
|---|---|
| 로봇 OS | ROS2 Jazzy (Ubuntu 24.04 LTS) |
| SBC | Raspberry Pi 5 (arm64, 8GB) |
| 주행 제어 MCU | STM32F103C8T6 (CMSIS, HAL/RTOS 미사용) |
| 센서 퓨전 MCU | Raspberry Pi Pico 2 (RP2350, pico-sdk 2.x) |
| SLAM / 내비게이션 | slam_toolbox, Nav2, robot_localization(EKF), m-explore-ros2(frontier exploration) |
| LiDAR | RPLiDAR C1 (`sllidar_ros2`) |
| AI Vision | YOLOv8 (ONNX 런타임 추론) |
| 이상 탐지 | scikit-learn Isolation Forest (환경 센서 패턴 이상) |
| 관제 통신 | 자체 JSON-over-WebSocket 프로토콜 (rosbridge 미사용) |
| 관제 프론트엔드 | React 19, Canvas 기반 지도 렌더링 |
| 시뮬레이션 | Gazebo, URDF/xacro |
| 라이선스 | Apache License 2.0 |

---

## 시작하기

가장 먼저 볼 곳은 [`S2M-SBC-Integration`](https://github.com/Scout2Map/S2M-SBC-Integration)이다. 이 레포가 나머지 레포들을 ROS2 워크스페이스로 통합하는 브링업 지점이며, 시뮬레이션(Gazebo)과 실차 실행 절차를 모두 문서화하고 있다.

```bash
mkdir -p ~/scout2map_ws/src && cd ~/scout2map_ws/src
git clone https://github.com/Scout2Map/S2M-SBC-Integration.git
git clone https://github.com/Scout2Map/S2M-MCU-BridgeNode.git
git clone https://github.com/Scout2Map/S2M-Event-Engine.git
git clone https://github.com/Scout2Map/S2M-CommRelay.git
git clone https://github.com/Scout2Map/S2M-Hardware.git   # UGV_description만 필요
```

두 MCU 펌웨어(`S2M-FW-DrivingControl`, `S2M-FW-SensorFusion`)와 관제 웹(`S2M-Web-Monitoring`)은 별도로 클론해 각 레포의 README를 따라 빌드·실행한다. 실행 순서, 파라미터, 트러블슈팅은 각 레포 README에 정리되어 있다.

---

## 라이선스

이 조직의 레포지토리는 Apache License 2.0을 따른다. 각 레포의 `LICENSE` 파일을 함께 참고한다.
