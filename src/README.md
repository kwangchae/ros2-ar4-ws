이 문서는 Annin Robotics의 **AR4-MK3** 로봇 모델을 Unity로 가져와 물리 설정을 하고, 독립적으로 제어하는 UI를 구현한 뒤, 최종적으로 **WSL2의 ROS 2와 연동**하여 실시간으로 제어하고 시각화하는 전체 과정을 상세히 설명합니다.

## 🎯 최종 목표

  * AR4 로봇의 URDF 모델을 Unity에서 물리적으로 동작하는 개체(Prefab)로 변환합니다.
  * Unity 내에서 직접 관절을 제어할 수 있는 테스트용 UI를 구현합니다.
  * WSL2에서 실행되는 ROS 2와 Windows의 Unity를 TCP로 연결하여 양방향 통신을 구현합니다.
  * ROS 2 (MoveIt, Gazebo 등)를 통해 로봇을 제어하고, 그 결과를 Unity에서 실시간으로 시각화합니다.

## 📋 시스템 요구사항 및 환경 구성

시작하기 전, 다음 환경이 구성되어 있는지 확인하세요.

  * **운영체제**: Windows 11
  * **WSL2**: Ubuntu 24.04
  * **ROS 2**: Jazzy Jellyfish
  * **Unity**: 2022.3 LTS 이상 (URDF Importer 및 ROS-TCP-Connector 호환 버전)
  * **기타**: Git, Python 3

### ROS 2 워크스페이스 구조 (예시)

이 가이드에서는 다음과 같은 ROS 2 워크스페이스 구조를 가정합니다. `ROS-TCP-Endpoint` 패키지는 Unity와의 통신을 위해 필수적입니다.

```
ar4_ws/
├── src/
│   ├── ar4_ros_driver/          # AR4 로봇 드라이버 및 모델 파일
│   │   ├── annin_ar4_description/
│   │   ├── annin_ar4_driver/
│   │   ├── annin_ar4_moveit_config/
│   │   └── annin_ar4_gazebo/
│   └── ROS-TCP-Endpoint/        # Unity 통신용 TCP 서버 패키지
├── build/
├── install/
└── log/
```

-----

## 단계 1: Unity용 로봇 모델(URDF) 준비

가장 먼저 할 일은 ROS에서 사용하는 `xacro` 파일을 Unity의 `URDF Importer`가 인식할 수 있는 순수 `urdf` 파일로 변환하는 것입니다.

### 1\. AR4 로봇 모델 다운로드

터미널(WSL2 또는 Git Bash)에서 다음 명령을 실행하여 로봇 설명 파일이 포함된 리포지토리를 복제합니다.

```bash
git clone https://github.com/ycheng517/ar4_ros_driver.git ~/ar4_ros_driver
```

### 2\. MK3 모델용 URDF 파일 생성

`xacro` 명령어를 사용해 `ar.urdf.xacro` 파일을 기반으로 `ar4_mk3.unity.urdf` 파일을 생성합니다.

```bash
# ROS 환경 설정 로드 (오류가 나도 무시)
source /opt/ros/jazzy/setup.bash 2>/dev/null || true

# xacro 파일이 있는 디렉토리로 이동
cd ~/ar4_ros_driver/annin_ar4_description

# xacro를 urdf로 변환 (그리퍼 제외)
xacro urdf/ar.urdf.xacro ar_model:=mk3 include_gripper:=false > urdf/ar4_mk3.unity.urdf
```

> **💡 팁:** 서보 그리퍼를 포함하려면 `include_gripper:=false`를 `true`로 변경하세요.

### ❗ `xacro` 실행 오류 해결

  * **`No such file or directory: ./urdf/ar4.urdf.xacro`**: 파일명이 `ar4.urdf.xacro`가 아닌 \*\*`ar.urdf.xacro`\*\*입니다. 위 명령어를 다시 확인하세요.
  * **`AttributeError: module 'xml' has no attribute 'parsers'`**: Python 환경에 `xml`이라는 다른 패키지가 설치되어 표준 라이브러리를 가리는 경우입니다. 다음 명령으로 충돌하는 패키지를 찾아 제거하세요.
    ```bash
    # 잘못 설치된 'xml' 패키지 제거
    python3 -m pip show xml >/dev/null 2>&1 && python3 -m pip uninstall -y xml
    ```

-----

## 단계 2: Unity 프로젝트 설정 및 로봇 임포트

이제 생성된 URDF 파일을 Unity로 가져와 실제 로봇처럼 움직이게 설정합니다.

### 1\. Unity 프로젝트 생성 및 패키지 설치

1.  Unity Hub에서 **3D (Built-in 또는 URP)** 템플릿으로 새 프로젝트를 생성합니다.
2.  메뉴에서 \*\*[Window] \> [Package Manager]\*\*를 엽니다.
3.  **[+]** 버튼 클릭 후 \*\*[Add package from git URL...]\*\*을 선택하고 다음 주소를 입력합니다.
    ```
    https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer#v0.5.2
    ```

### 2\. URDF 및 메시 파일 배치

Unity 프로젝트의 `Assets` 폴더 안에 다음과 같은 구조로 파일을 배치합니다.

```
Assets/
  └── Robots/
      └── AR4/
          ├── ar4_mk3.unity.urdf     # (단계 1에서 생성한 파일 복사)
          └── meshes/                # 'annin_ar4_description/meshes/' 폴더 전체 복사
              ├── ar4_mk3/
              │   ├── base_link.STL
              │   └── ...
              └── ar4_gripper/
                  └── ...
```

### 3\. URDF 파일 경로 수정 (⭐ 매우 중요)

Unity Importer는 URDF에 포함된 메시 파일 경로가 **URDF 파일 기준의 상대 경로**일 것을 요구합니다. `sed` 명령어를 사용해 모든 경로를 한 번에 수정합니다.

```bash
# Unity 프로젝트의 AR4 폴더로 이동
cd "<UnityProject>/Assets/Robots/AR4"

# 원본 백업
cp ar4_mk3.unity.urdf ar4_mk3.unity.urdf.bak

# 'package://' 또는 절대 경로를 'meshes/' 상대 경로로 변경
sed -E -i 's#filename="package://[^"]*/meshes/#filename="meshes/#g' ar4_mk3.unity.urdf
sed -E -i 's#filename="(file://)?/[^"]*/annin_ar4_description/(share/)?annin_ar4_description/meshes/#filename="meshes/#g' ar4_mk3.unity.urdf
```

### 4\. Unity에서 로봇 임포트

1.  Project 창에서 `ar4_mk3.unity.urdf` 파일을 우클릭하고 \*\*[Import Robot from Selected URDF file]\*\*을 선택합니다.
2.  임포트 설정 창에서 다음을 확인하세요.
      * **Collision Decomposition**: **Unity Mesh Decomposer**로 설정합니다. (일부 STL 파일에서 VHACD가 오류를 일으킬 수 있습니다.)
      * **Axis**: `Y Up` (Unity 기본값)
3.  임포트가 완료되면 생성된 로봇 프리팹(Prefab)을 씬(Scene)으로 드래그합니다.
4.  씬에 배치된 로봇의 최상위 객체(`ar4_mk3.unity`)를 선택하고, Inspector 창의 **Articulation Body** 컴포넌트에서 **Immovable** 옵션을 체크하여 로봇이 중력에 의해 떨어지지 않도록 고정합니다.

-----

## 단계 3: Unity 내 단독 제어 UI 구현 (동작 검증)

ROS와 연결하기 전에, Unity에서 로봇이 올바르게 움직이는지 검증하기 위해 간단한 제어 UI를 만듭니다.

### 1\. UI 파일 및 스크립트 준비

프로젝트에 다음 구조로 UI 관련 폴더와 파일을 생성합니다. (UI 파일 및 스크립트의 전체 코드는 제공된 `AR4-MK3를 Unity에 불러오기...md` 파일을 참고하세요.)

```
Assets/
  └── Robots/
      └── AR4/
          ├── UI/
          │   ├── AR4JoggerPanel.uxml  # UI 레이아웃
          │   └── AR4JoggerPanel.uss   # UI 스타일
          └── Scripts/
              └── AR4JoggerPanel.cs    # UI 로직 스크립트
```

### 2\. 씬(Scene) 설정

1.  하이어라키(Hierarchy) 창에서 빈 게임 오브젝트를 생성하고 이름을 `AR4_UI_Controller`로 지정합니다.
2.  `AR4_UI_Controller`에 **UI Document** 컴포넌트를 추가하고, `Source Asset` 필드에 `AR4JoggerPanel.uxml` 파일을 드래그합니다.
3.  같은 오브젝트에 **AR4JoggerPanel.cs** 스크립트를 추가합니다.
4.  Inspector 창에 나타난 `AR4JoggerPanel` 스크립트에서:
      * **Joints** 배열의 크기를 `6`으로 설정합니다.
      * 로봇의 `link_1`부터 `link_6`까지 각 관절(Articulation Body)을 순서대로 드래그하여 할당합니다.
      * (선택) **Home Pose Degrees**에 기본 홈 포즈 각도를 입력합니다.

### 3\. 사용 및 동작 확인

1.  Unity 에디터에서 **Play** 버튼을 누릅니다.
2.  화면 좌측 상단에 제어 패널이 나타납니다.
3.  **슬라이더**를 움직여 각 관절이 부드럽게 움직이는지 확인합니다.
4.  **[홈 포즈]**, **[현재 각도 저장/불러오기]** 기능이 정상적으로 동작하는지 테스트합니다.

> 👏 **성공\!** 여기까지 완료했다면 Unity에서 물리 기반의 AR4 로봇을 완벽하게 제어할 수 있게 된 것입니다.

-----

## 단계 4: ROS 2와 Unity 연결 설정

이제 Unity의 로봇을 외부, 즉 WSL2의 ROS 2와 연결하여 통신을 설정합니다.

### 1\. ROS 2 워크스페이스 빌드 및 환경 설정

먼저 `ROS-TCP-Endpoint`를 포함한 ROS 2 워크스페이스를 빌드합니다.

```bash
# 워크스페이스 디렉토리로 이동
cd ~/ar4_ws

# ROS 의존성 설치
rosdep install --from-paths . --ignore-src -r -y

# 전체 패키지 빌드
colcon build

# 빌드된 환경 설정 로드
source install/setup.bash
```

### 2\. ROS TCP 서버 실행

WSL2 터미널에서 TCP Endpoint 서버를 실행하여 Unity의 연결을 기다립니다. `ROS_IP`를 `0.0.0.0`으로 설정하여 외부 연결을 허용합니다.

```bash
# 환경 설정 로드
source install/setup.bash

# TCP Endpoint 서버 실행
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

성공적으로 실행되면 다음과 같은 메시지가 출력됩니다.
`[INFO] [timestamp] [UnityEndpoint]: Starting server on 0.0.0.0:10000`

### 3\. WSL2의 IP 주소 확인

Unity가 연결할 WSL2의 IP 주소를 확인해야 합니다. 새 WSL2 터미널을 열고 다음 명령을 실행합니다.

```bash
hostname -I
```

출력된 IP 주소(예: `172.27.144.1`)를 기록해 둡니다.

### 4\. Unity ROS-TCP-Connector 설정

1.  Unity의 **Package Manager**에서 Git URL로 다음 패키지를 추가합니다.
    ```
    https://github.com/Unity-Technologies/ROS-TCP-Connector.git
    ```
2.  메뉴에서 \*\*[Robotics] \> [ROS Settings]\*\*를 엽니다.
3.  Inspector 창에서 다음과 같이 설정합니다.
      * **ROS IP Address**: 방금 확인한 **WSL2의 IP 주소** (예: `172.27.144.1`)
      * **ROS Port**: `10000`
      * **Protocol**: `TCP`

### 5\. 연결 확인

Unity 에디터에서 **Play** 버튼을 누릅니다. ROS TCP 서버를 실행한 WSL2 터미널에 다음과 같은 연결 성공 메시지가 나타나면 설정이 완료된 것입니다.

```
[INFO] [timestamp] [UnityEndpoint]: Connection from 172.27.144.1
[INFO] [timestamp] [UnityEndpoint]: RegisterSubscriber(...) OK
[INFO] [timestamp] [UnityEndpoint]: RegisterPublisher(...) OK
```

-----

## 단계 5: ROS 2를 이용한 로봇 제어 및 시뮬레이션

연결이 완료되었으니, ROS 2의 강력한 도구들을 사용하여 Unity의 로봇을 제어해 보겠습니다.

### 주요 통신 토픽

  * `/joint_command` (Unity → ROS 2): Unity에서 발행하는 로봇 관절 명령 (조그 UI 등)
  * `/joint_states` (ROS 2 → Unity): ROS에서 계산된 로봇의 관절 상태
  * `/tf` (ROS 2 → Unity): 로봇의 각 링크 좌표계 정보

### 1\. MoveIt 데모 실행 (시뮬레이션)

새 WSL2 터미널을 열고 MoveIt을 실행하여 가상 컨트롤러로 로봇을 제어합니다.

```bash
source ~/ar4_ws/install/setup.bash
ros2 launch annin_ar4_moveit_config demo.launch.py
```

RViz에서 로봇 팔을 움직이면 Unity의 로봇이 똑같이 따라 움직이는 것을 확인할 수 있습니다.

### 2\. Gazebo 시뮬레이션 연동

Gazebo 시뮬레이터와 Unity를 동시에 연동할 수도 있습니다.

```bash
# 터미널 1: Gazebo 실행
source ~/ar4_ws/install/setup.bash
ros2 launch annin_ar4_gazebo gazebo.launch.py

# 터미널 2: MoveIt 실행 (시뮬레이션 시간 사용)
source ~/ar4_ws/install/setup.bash
ros2 launch annin_ar4_moveit_config moveit.launch.py use_sim_time:=true
```

### 3\. Unity 스크립트로 ROS 토픽 발행/구독 예제

Unity의 C\# 스크립트를 통해 ROS 2 토픽과 직접 상호작용할 수 있습니다.

#### ROS 2 토픽 구독 (`/joint_states` 받기)

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStateSubscriber : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>("/joint_states", OnJointStateReceived);
    }

    void OnJointStateReceived(JointStateMsg jointState)
    {
        Debug.Log($"Joint positions: {string.Join(", ", jointState.position)}");
        // 이 데이터를 사용하여 Unity 로봇 모델을 직접 업데이트할 수 있습니다.
    }
}
```

#### ROS 2 토픽 발행 (`/joint_command` 보내기)

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointCommandPublisher : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>("/joint_command");
    }

    public void PublishJointCommand()
    {
        var jointCommand = new JointStateMsg
        {
            name = new string[] { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" },
            position = new double[] { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 }
        };
        ros.Publish("/joint_command", jointCommand);
    }
}
```

-----

## 🛠️ 종합 문제 해결 (Troubleshooting)

  * **TCP 포트 충돌**: `OSError: [Errno 98] Address already in use` 메시지가 나타나면 기존 `default_server_endpoint` 프로세스를 종료하세요.
    ```bash
    pkill -f default_server_endpoint
    ```
  * **Unity 임포트 시 `NullReferenceException`**: 임포트 설정에서 **Collision Decomposition**을 **Unity**로 바꾸거나, 문제가 되는 링크의 `<collision>` 태그를 단순한 형태(cylinder, box)로 수정한 뒤 다시 시도하세요.
  * **Unity `Input System` 예외**: **[Edit] \> [Project Settings] \> [Player] \> [Other Settings]** 에서 **Active Input Handling**을 **Both**로 변경하세요.
  * **메시 파일 경로/대소문자 문제**: URDF 파일 내의 모든 `<mesh filename="...">` 경로가 `meshes/`로 시작하는지, 그리고 실제 파일명과 대소문자가 일치하는지 다시 한번 확인하세요.

## 📚 참고 자료

  * [AR4 ROS Driver GitHub](https://github.com/ycheng517/ar4_ros_driver)
  * [Unity ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
  * [Unity URDF-Importer](https://github.com/Unity-Technologies/URDF-Importer)
  * [ROS 2 Jazzy 공식 문서](https://docs.ros.org/en/jazzy/)

## ✨ 결론

이 가이드를 통해 여러분은 WSL2의 ROS 2 환경과 Windows의 Unity를 성공적으로 연동했습니다. 이제 로봇 시뮬레이션, 디지털 트윈 구축, 강화학습 환경 구성, 고품질 시각화 등 무한한 가능성을 탐색할 수 있습니다.