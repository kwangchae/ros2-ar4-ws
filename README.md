# AR4 Robot Control System

Unity와 ROS2를 연동한 AR4 로봇 제어 시스템입니다.

## 🌟 주요 기능

- **실시간 Unity-ROS2 통신**: WSL2 ↔ Windows Unity 연동
- **MoveIt 경로 계획**: RViz에서 계획하고 Unity에서 시각화
- **지능형 궤적 제어**: 부드럽고 자연스러운 로봇 움직임
- **키보드 텔레오퍼레이션**: 실시간 수동 제어
- **궤적 시각화**: 노란색 waypoint와 경로선 표시

## 🛠️ 시스템 구성

### 🏗️ 멀티-레포 구조
이 저장소는 [ar4-stack](https://github.com/kwangchae/ar4-stack) 메타 저장소의 서브모듈입니다:

```
ar4-stack/                    ← 메타 저장소
├── ros2-ar4-ws/             ← 이 저장소 (ROS2 워크스페이스)
└── unity-ar4-sim/           ← Unity 시뮬레이션 저장소
```

### 📁 ROS2 워크스페이스 구성
- `src/ar4_ros_driver/` - AR4 로봇 드라이버 패키지
- `src/ROS-TCP-Endpoint/` - Unity 통신 TCP 엔드포인트
- `src/*.py` - 커스텀 제어 및 브릿지 스크립트들

## 🚀 빠른 시작

### 🏁 전체 시스템 클론 (권장)
```bash
# 메타 저장소에서 모든 서브모듈과 함께 클론
git clone --recursive https://github.com/kwangchae/ar4-stack.git
cd ar4-stack/ros2-ar4-ws
```

### 🔧 개별 저장소 사용
```bash
# 이 저장소만 클론
git clone https://github.com/kwangchae/ros2-ar4-ws.git
cd ros2-ar4-ws
```

### 1단계: ROS2 워크스페이스 빌드
```bash
# 의존성 설치 (권장)
rosdep install --from-paths . --ignore-src -r -y

# 빌드
colcon build --symlink-install

# 환경 설정 로드
source install/setup.bash
```

### 2단계: TCP 서버 시작
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

### (선택) Docker로 ROS TCP 서버 실행
메타 저장소 루트에서 Docker Compose를 사용해 손쉽게 서버를 실행할 수 있습니다.
```bash
cd ..   # ar4-stack 루트로 이동
docker compose build
docker compose up        # 소프트웨어 시뮬레이션용 (ros)
docker compose up ros-hw # 실기 연결(USB)용
```
컨테이너는 포트 `10000`을 열고, `/ws`로 마운트된 이 워크스페이스를 빌드/사용합니다.

### 3단계: Unity 시뮬레이션 실행
Unity 프로젝트는 [unity-ar4-sim](https://github.com/kwangchae/unity-ar4-sim) 저장소에서 확인하세요.

### 4단계: MoveIt 실행
```bash
ros2 launch annin_ar4_moveit_config demo.launch.py
```

### 5단계: 제어 시스템 활성화
```bash
python3 src/smooth_robot_controller.py    # 터미널 1
python3 src/moveit_bridge.py             # 터미널 2
```

## 🎮 사용법

### RViz에서 로봇 제어
1. Interactive Marker를 드래그하여 목표 위치 설정
2. "Plan" 클릭 → Unity에서 노란색 waypoint 확인
3. "Execute" 클릭 → Unity 로봇이 경로를 따라 움직임

### 키보드 직접 제어
```bash
python3 src/simple_keyboard_teleop.py
```

### 지능형 궤적 패턴
```bash
python3 src/trajectory_publisher.py
```

## 📊 시스템 아키텍처

```
┌─────────────────┐    TCP/IP     ┌──────────────────┐
│   WSL2 (ROS2)   │◄─────────────►│ Windows (Unity)  │
│                 │   10000 port  │                  │
│ ├─ MoveIt       │               │ ├─ AR4 Robot     │
│ ├─ Controllers  │               │ ├─ Visualizer    │
│ └─ TCP Endpoint │               │ └─ ROS Manager   │
└─────────────────┘               └──────────────────┘
```

## 🔧 주요 토픽

- `/joint_command` - Unity 로봇 제어
- `/joint_states` - 로봇 상태 피드백
- `/trajectory_preview` - 궤적 waypoint 시각화
- `/display_planned_path` - MoveIt 계획된 경로

## 🛠️ 문제 해결

### Unity 연결 안 됨
```bash
# WSL2 IP 확인
hostname -I

# TCP 서버 상태 확인  
netstat -tlnp | grep 10000
```

### 로봇이 움직이지 않음
```bash
# 연결 테스트
python3 src/test_unity_connection.py
```

## 📝 개발 로그

- [v1.0] 기본 ROS2-Unity 통신 구현
- [v1.1] MoveIt 경로 계획 통합
- [v1.2] 부드러운 로봇 제어 시스템
- [v1.3] 궤적 시각화 및 UI 개선

## 🔗 관련 저장소

- **메타 저장소**: [ar4-stack](https://github.com/kwangchae/ar4-stack) - 전체 시스템 통합
- **Unity 시뮬레이션**: [unity-ar4-sim](https://github.com/kwangchae/unity-ar4-sim) - 3D 로봇 시각화
- **AR4 공식**: [Annin Robotics](https://www.anninrobotics.com/) - AR4 로봇 하드웨어

## 🤝 기여

이슈 및 개선 제안은 [ar4-stack](https://github.com/kwangchae/ar4-stack)에서 환영합니다!

## 📄 라이선스

MIT License

---

**개발 환경**
- WSL2: Ubuntu 24.04
- ROS2: Jazzy Jellyfish  
- Unity: 2022.3 LTS
- Windows: 11

> 🤖 이 저장소는 **AR4 Stack**의 일부입니다. 전체 시스템은 [ar4-stack](https://github.com/kwangchae/ar4-stack)에서 확인하세요.
