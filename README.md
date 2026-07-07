# Industrial Sim Capstone

Gazebo 시뮬레이션 위에서 컨베이어 공급, 로봇팔 Pick & Place, RGB-D 비전 분류, Nav2 기반 AMR 운송을 하나의 ROS 2 상태 흐름으로 연동한 스마트 팩토리 물류 자동화 시뮬레이션입니다.

## 1. 프로젝트 개요

### 해결하려는 문제

스마트 팩토리를 구성하는 컨베이어, 로봇팔, 비전 시스템, 무인운반로봇(AMR)은 각기 다른 벤더와 제어 방식으로 개발되는 경우가 많아, 이들을 하나의 공정 흐름으로 통합해 검증하는 작업 자체가 비용이 크고 실물 장비 없이는 실험하기 어렵습니다.

### 사용자가 겪는 Pain Point

- 실제 로봇팔·AMR·컨베이어 장비를 갖추지 않고서는 전체 물류 공정의 동작을 사전에 검증할 수 없습니다.
- 개별 장비(비전, 팔, AMR)를 따로 테스트하면 정상 동작해도, 신호 연동 시점이 어긋나면 전체 공정이 멈추는 문제를 사전에 발견하기 어렵습니다.
- 장애물 회피, 위치 추정 실패, 배송 재시도 같은 예외 상황은 실제 환경에서 재현하고 검증하기 위험하거나 번거롭습니다.

### 프로젝트의 핵심 아이디어

물리 기반 시뮬레이터(Gazebo)와 ROS 2 표준 인터페이스만으로 컨베이어 구동, 흡착식 Pick & Place, OpenCV 기반 RGB-D 비전 분류, Nav2 자율주행을 모두 하나의 공정 상태 머신(`factory_manager`)으로 엮어, 실장비 없이도 스마트 팩토리 물류 사이클 전체를 처음부터 끝까지 재현 가능하게 만듭니다.

### 최종적으로 구현한 서비스/시스템의 역할

컨베이어에 투입된 A/B/C 세 종류의 물품을 비전으로 인식하고, 로봇팔이 이를 집어 지정된 이동식 상자에 적재한 뒤, 적재가 끝난 상자가 AMR(LiDAR·AMCL 위치 추정과 Nav2 경로 계획)을 통해 각자의 출하 도크까지 자율 주행하고, 도착 후 다시 적재 위치로 복귀해 다음 생산 주기를 자동으로 시작하는 무한 반복 시뮬레이션 시스템입니다. 전체 공정 상태는 브라우저 기반 관제 대시보드에서 실시간으로 확인할 수 있습니다.

### 캡스톤 프로젝트로서의 의의

개별 로봇 제어 이론(모션 제어, 컴퓨터 비전, 경로 계획, 상태 머신 설계)을 따로 학습하는 데 그치지 않고, ROS 2 노드 간 인터페이스 설계, 물리 시뮬레이션 통합, 예외 처리 및 재시도 로직까지 하나의 동작하는 시스템으로 완성했다는 데 의의가 있습니다.

## 2. 주요 기능

- **컨베이어 물품 공급 및 자동 정지**: `conveyor_control` 패키지가 Gazebo의 `TrackController` 플러그인을 통해 벨트를 구동하고, 논리 카메라(logical camera) 또는 비전 인식 결과를 기준으로 목표 물품이 픽업 위치에 도달하면 자동으로 정지합니다. 사용자는 컨베이어 속도나 정지 로직을 직접 조작하지 않아도 물품이 항상 정확한 픽업 지점에서 멈추는 것을 확인할 수 있습니다.
- **시드 기반 랜덤 배치 생성**: `random_spawner` 노드가 지정된 시드로 A/B/C 물품의 도착 순서와 간격을 재현 가능하게 생성하고, `/item_spawner/manifest` 토픽으로 배치 정보를 JSON으로 발행합니다. 같은 시드를 지정하면 동일한 시연을 반복 재현할 수 있어 디버깅과 발표 시연에 유리합니다.
- **RGB-D 비전 기반 물품 분류**: `item_vision` 노드가 OpenCV HSV 색상 분할과 깊이 이미지를 결합해 A(빨강 큐브)/B(초록 원기둥)/C(파랑 육각기둥)를 분류하고, 카메라 내부 파라미터를 이용해 3차원 월드 좌표와 신뢰도 점수를 함께 계산합니다. 이를 통해 실제 산업 현장의 비전 검사 공정과 유사한 방식으로 물품 위치를 추정하는 과정을 시뮬레이션에서 확인할 수 있습니다.
- **관절 피드백 기반 로봇팔 Pick & Place**: `arm_control`이 회전(yaw)·승강(lift) 관절 상태를 실시간으로 확인하며 목표각과 목표 높이에 도달했는지 판정하고, 흡착식 `DetachableJoint`로 물품을 집고 해당 클래스의 박스에 내려놓는 전체 시퀀스(대기 → 흡착 → 상승 → 회전 → 하강 → 분리 → 복귀)를 상태 머신으로 처리합니다. 물리 엔진의 실제 관절 값을 기준으로 판정하므로 타이밍이 어긋나도 안정적으로 동작합니다.
- **Nav2 기반 AMR 자율 운송과 자동 재시도**: `amr_control`이 각 이동식 상자(bin_a_red/bin_b_green/bin_c_blue)의 360도 LiDAR, 휠 오도메트리를 AMCL에 연결해 위치를 추정하고, Nav2 `NavigateToPose` 액션으로 경로 계획과 추종을 수행합니다. 시간 초과, 목표 도달 오차 초과, 진행 정체가 감지되면 설정된 횟수(기본 2회)까지 자동으로 재시도합니다. 사용자는 배송이 일시적으로 실패해도 시스템이 스스로 복구를 시도하는 과정을 지켜볼 수 있습니다.
- **동적 장애물 회피 데모**: `dynamic_obstacle_demo` 노드가 B 상자의 첫 주행 구간 근처에서 팔레트 장애물을 일시적으로 이동시켜, Nav2가 LiDAR로 장애물을 인식하고 지역/전역 costmap을 갱신해 경로를 재계획하는 과정을 RViz2에서 시각적으로 확인할 수 있게 합니다.
- **전체 공정 상태 머신과 무한 반복 생산 사이클**: `factory_manager`가 컨베이어·로봇팔·AMR의 완료 신호를 집계해 `IDLE → CONVEYOR_RUNNING → ITEM_READY → PICKING → BOX_READY → AMR_MOVING → DELIVERED`로 이어지는 공정 상태를 관리하고, 배송이 끝나면 자동으로 다음 배치를 준비합니다. 사용자가 별도로 개입하지 않아도 공정이 계속 반복됩니다.
- **실시간 브라우저 관제 대시보드**: `dashboard_node`가 외부 프레임워크 없이 Python 표준 라이브러리(`http.server`)만으로 웹 서버를 열어, 공정 상태, 비전 신뢰도, 박스별 적재/배송 상태, 이벤트 로그를 `http://127.0.0.1:8080`에서 실시간으로 보여줍니다. 사용자는 Gazebo 화면 없이도 브라우저만으로 공정 진행 상황을 파악할 수 있습니다.
- **재현 가능한 자동화 테스트**: 각 패키지에 `pytest` 기반 단위 테스트가 포함되어 있어(총 20개 테스트, 현재 전량 통과 확인), 재시도 정책, 랜덤 배치 생성, 월드 자산 구성, 대시보드 데이터 요소를 코드 변경 시마다 자동으로 검증할 수 있습니다.

## 3. 시스템 아키텍처

```text
[Gazebo 물리 시뮬레이션 (factory_test.sdf)]
  컨베이어(TrackController) / 로봇팔 관절 / RGB-D 카메라 / 논리 카메라
  이동식 상자 3대(DiffDrive + GPU LiDAR) / 팔레트 장애물
        │  ros_gz_bridge (parameter_bridge)
        ▼
[ROS 2 노드 계층]

  random_spawner ──(/item_spawner/manifest, JSON)──▶ conveyor_node, arm_controller, dashboard_node
        │ SetEntityPose 서비스로 물품 배치
        ▼
  conveyor_node ──(/conveyor/state: STOPPED:ITEM_READY)──▶ arm_controller, factory_manager
        │ 논리 카메라 / vision_node 감지 결과로 정지 판단
        ▲
  vision_node (OpenCV HSV + Depth) ──(/vision/detections)──▶ conveyor_node, arm_controller, dashboard_node
        │
        ▼
  arm_controller (관절 피드백 상태 머신) ──(/arm/task_complete)──▶ factory_manager
        │ 흡착(DetachableJoint) attach/detach
        ▼
  factory_manager (전체 공정 상태 머신)
        │ 목표 수량(3개) 적재 완료 시 ──(/amr/start_delivery)──▶ amr_controller
        ▼
  amr_controller (AMCL 위치 추정 + Nav2 NavigateToPose)
        │ /scan, /nav/odom, /amcl_pose ↔ nav2_amcl, nav2_controller,
        │ nav2_planner, nav2_bt_navigator, nav2_behaviors
        │ dynamic_obstacle_demo가 장애물 위치를 주입해 회피 시나리오 유발
        ▼
  각 상자 출하 도크 도착 ──(/amr/delivery_complete, /factory/state=DELIVERED)──▶ factory_manager, dashboard_node
        │
        ▼
[결과 확인]
  RViz2 (지도, TF, AMCL, LiDAR, costmap, 전역 경로 시각화)
  브라우저 대시보드 http://127.0.0.1:8080 (공정 상태 · 비전 신뢰도 · 박스별 배송 상태 · 이벤트 로그)
```

이 프로젝트는 별도의 프론트엔드/백엔드 서버나 데이터베이스를 갖는 웹 서비스가 아니라, ROS 2 노드 간 토픽/서비스/액션 통신으로 구성된 로봇 시뮬레이션 시스템입니다. "AI/LLM/Agent 처리 흐름"에 해당하는 부분은 없으며, 물품 인식은 OpenCV 기반의 전통적 컴퓨터 비전(HSV 색상 분할)으로 수행됩니다. 데이터 저장은 별도 DB 없이 각 노드가 메모리 상에서 상태를 관리하고, 대시보드가 이를 실시간 스냅숏으로 제공합니다.

## 4. 기술 스택

`package.xml`, `setup.py`의 `install_requires`/`exec_depend` 및 실제 import 구문을 기준으로 확인한 내용입니다.

- **시뮬레이션 엔진**: Gazebo Harmonic (`gz sim`, `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces`)
- **로봇 미들웨어**: ROS 2 Jazzy (`rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `action_msgs`, `vision_msgs`, `tf2_ros`)
- **자율주행**: Nav2 스택 — `nav2_amcl`, `nav2_controller`(`RegulatedPurePursuitController`), `nav2_planner`(`NavfnPlanner`), `nav2_bt_navigator`, `nav2_behaviors`, `nav2_lifecycle_manager`, `nav2_map_server`
- **컴퓨터 비전**: OpenCV(`cv2`), `cv_bridge`, `numpy` — HSV 색상 분할과 Depth 이미지 기반 3차원 위치 추정
- **관제 대시보드**: Python 표준 라이브러리 `http.server`(`ThreadingHTTPServer`)만 사용한 자체 구현 웹 서버 및 인라인 HTML/JS(`dashboard_page.py`), 외부 웹 프레임워크·데이터베이스 미사용
- **시각화**: RViz2 (지도, TF, AMCL, LiDAR, global/local costmap, 전역 경로)
- **빌드 시스템**: `colcon`, `ament_python` (모든 패키지가 순수 Python 기반 `ament_python` 빌드 타입)
- **테스트**: `pytest`, `ament_copyright`, `ament_flake8`, `ament_pep257` (`colcon test` / `colcon test-result`)
- **개발/실행 환경**: Ubuntu 24.04, ROS 2 Jazzy, Bash 스크립트(`scripts/setup-dev.sh`, `scripts/run-demo.sh`)

인프라 배포(클라우드, 컨테이너 오케스트레이션 등)나 별도의 DB는 사용하지 않습니다. Dockerfile은 저장소에 존재하지 않습니다.

## 5. 폴더 구조

```text
industrial-sim-capstone/
├── README.md
├── LICENSE
├── media/
│   └── industrial_sim_story_example.mp4   # 콘셉트 시연 영상(최종 실행 결과 아님)
├── docs/
│   ├── architecture.md                    # ROS 2 인터페이스와 통합 실행 정의
│   ├── requirements.md                    # MVP 요구사항과 완료 기준
│   └── team-tasks.md                      # 역할 분담 및 협업 원칙
├── scripts/
│   ├── setup-dev.sh                       # rosdep 설치 + colcon build
│   └── run-demo.sh                        # 원버튼 시연 실행 스크립트
└── src/
    ├── factory_description/               # 공장 월드(factory_test.sdf), 통합 launch 파일
    │   ├── launch/                        # factory_test.launch.py, demo.launch.py
    │   ├── worlds/                        # factory_test.sdf
    │   └── test/
    ├── conveyor_control/                  # 컨베이어 구동, 물품 랜덤 배치 생성
    │   ├── conveyor_control/              # conveyor_node.py, random_spawner.py
    │   └── test/
    ├── arm_control/                       # 로봇팔 Pick & Place 상태 머신
    │   └── arm_control/                   # arm_controller.py
    ├── amr_control/                       # AMR LiDAR/AMCL 위치추정, Nav2 운송, 동적 장애물 데모
    │   ├── amr_control/                   # amr_controller.py, navigation_policy.py, dynamic_obstacle_demo.py
    │   ├── config/                        # nav2_params.yaml, factory_map.yaml/.pgm, factory_nav.rviz
    │   └── test/
    ├── item_vision/                       # RGB-D 색상·형상 기반 물품 분류
    │   └── item_vision/                   # vision_node.py
    └── factory_manager/                   # 전체 공정 상태 머신, 관제 대시보드
        └── factory_manager/               # factory_manager.py, dashboard_node.py, dashboard_page.py, constants.py
```

`build/`, `install/`, `log/`는 colcon이 생성하는 산출물로 `.gitignore`에 의해 저장소에서 제외됩니다.

## 6. 설치 및 실행 방법

### 사전 요구사항

- Ubuntu 24.04
- ROS 2 Jazzy (`/opt/ros/jazzy/setup.bash`가 존재해야 함)
- Gazebo Harmonic 및 `ros_gz` 브리지 패키지
- `colcon`, `rosdep`

### 저장소 클론

```bash
git clone https://github.com/GitHaewon/industrial-sim-capstone.git
cd industrial-sim-capstone
```

### 의존성 설치 및 빌드

```bash
source /opt/ros/jazzy/setup.bash
./scripts/setup-dev.sh
```

`setup-dev.sh`는 `rosdep install --from-paths src --ignore-src -r -y`로 의존 패키지를 설치한 뒤 `colcon build --symlink-install`을 실행합니다.

### 개발/테스트용 단일 월드 실행

```bash
source install/setup.bash
ros2 launch factory_description factory_test.launch.py
```

기본값은 RViz2와 동적 장애물 데모를 함께 켭니다. 필요에 따라 개별적으로 끌 수 있습니다.

```bash
ros2 launch factory_description factory_test.launch.py rviz:=false dynamic_obstacle:=false
```

다른 랜덤 배치를 재현하려면 시드를 지정합니다. 동일한 시드는 동일한 도착 순서와 간격을 생성합니다.

```bash
ros2 launch factory_description factory_test.launch.py random_seed:=17
```

### 발표/시연용 원버튼 실행

```bash
./scripts/run-demo.sh
```

`run-demo.sh`는 필요한 패키지(`amr_control`, `factory_description`, `conveyor_control`, `arm_control`, `item_vision`, `factory_manager`)를 먼저 빌드한 뒤 `demo.launch.py`(결정적 기본 시드 42, RViz2·동적 장애물 데모 활성화)를 실행합니다. 이미 빌드가 끝난 상태라면 빌드를 생략할 수 있습니다.

```bash
./scripts/run-demo.sh --skip-build
```

시연 옵션은 launch 인자로 그대로 전달할 수 있습니다.

```bash
./scripts/run-demo.sh random_seed:=17
./scripts/run-demo.sh rviz:=false
./scripts/run-demo.sh dynamic_obstacle:=false
```

### 테스트 실행

```bash
colcon test
colcon test-result --verbose
```

이 저장소 기준 현재 6개 패키지, 총 20개 테스트가 모두 통과하는 것을 확인했습니다(`amr_control`, `arm_control`, `conveyor_control`, `factory_manager`, `item_vision`, `factory_description`).

### 빌드 방법 (참고)

별도의 빌드 산출물(바이너리 배포판)은 없으며, `colcon build --symlink-install`이 곧 빌드 과정입니다. 모든 패키지가 `ament_python` 빌드 타입이므로 컴파일 단계 없이 Python 소스가 `install/` 아래에 배치됩니다.

## 7. 환경변수 설정

이 프로젝트는 `.env` 파일이나 API 키를 사용하지 않습니다. 코드 전체를 검색한 결과 `os.environ`, `getenv`, API 키 관련 로직이 존재하지 않는 것을 확인했습니다.

다만 `factory_test.launch.py`가 Gazebo 멀티캐스트 충돌을 줄이기 위해 다음 환경변수를 launch 시점에 자동으로 설정합니다(사용자가 직접 설정할 필요 없음).

```env
GZ_IP=127.0.0.1
IGN_IP=127.0.0.1
GZ_PARTITION=industrial_sim_capstone
```

| 변수 | 용도 |
| --- | --- |
| `GZ_IP` / `IGN_IP` | 단일 PC 환경에서 Gazebo transport가 로컬 루프백을 사용하도록 고정 |
| `GZ_PARTITION` | 동일 네트워크의 다른 Gazebo 인스턴스와 트랜스포트가 섞이지 않도록 분리 |

## 8. 사용 방법

이 프로젝트는 웹 서비스가 아닌 ROS 2/Gazebo 시뮬레이션이므로, 실행 후 화면을 관찰하는 방식으로 사용합니다.

1. `./scripts/run-demo.sh` 또는 `ros2 launch factory_description factory_test.launch.py`로 시뮬레이션을 실행합니다.
2. Gazebo 창에서 컨베이어에 A/B/C 물품이 시드에 따라 랜덤 순서로 등장하고 이동하는 것을 확인합니다.
3. 물품이 픽업 위치에 도달하면 로봇팔이 자동으로 Pick & Place를 수행하는 것을 관찰합니다.
4. 3개 물품 적재가 끝나면 해당 이동식 상자(A/B/C)가 자동으로 Nav2 경로를 따라 출하 도크로 이동하는 것을 확인합니다.
5. RViz2에서 `/scan`, `/amcl_pose`, `/global_costmap/costmap`, `/local_costmap/costmap`, `/plan`을 통해 위치 추정과 경로 계획, 동적 장애물 회피 과정을 시각적으로 확인합니다.
6. 브라우저에서 `http://127.0.0.1:8080` 대시보드를 열어 공정 상태, 비전 신뢰도, 박스별 배송 상태, 이벤트 로그를 실시간으로 확인합니다.
7. 배송이 완료되면 `/factory/state` 토픽 또는 대시보드에서 `DELIVERED` 상태를 확인합니다. 이후 상자가 자동으로 복귀하고 새로운 랜덤 배치가 시작되며 공정이 반복됩니다.

현재 배치 정보(시드, 도착 순서, 물품별 작업 ID)는 `/item_spawner/manifest` 토픽에 JSON으로 발행되어 다른 도구에서도 구독해 확인할 수 있습니다.

## 9. 핵심 구현 내용

- **관절 피드백 기반 상태 머신** (`arm_control/arm_controller.py`): 목표 각도·목표 높이와 실제 관절 값의 오차(`_at_yaw`, `_at_lift`)를 각각 허용 오차(`0.06 rad`, `0.035 m`) 이내로 판정해 다음 상태로 전이합니다. 시간 기반이 아닌 실제 물리 피드백 기반 판정이라 시뮬레이션 속도 변화에 강건합니다.
- **비전 기반 픽업과 논리 카메라 이중화** (`conveyor_control/conveyor_node.py`): Gazebo의 `LogicalCameraImage`(정답에 가까운 신호)와 `item_vision`의 `Detection3DArray`(실제 인식 파이프라인) 두 경로를 모두 구독해, 비전 인식이 실패하거나 지연되어도 논리 카메라로 컨베이어 정지를 보장하는 이중 안전장치를 구성했습니다.
- **AMCL 위치 추정 검증 로직** (`amr_control/amr_controller.py`): AMCL이 발행한 추정 위치를 Gazebo 그라운드트루스와 비교(`hypot` 거리 오차)해 일정 범위(`1.0m`~`1.5m`) 밖의 추정치는 기각하고, 후보 위치가 `1.0`초 이상 `0.3m` 이내로 안정되어야 실제 위치로 확정하는 방식으로 시뮬레이션 국소 정합성 문제를 보완했습니다.
- **좁은 적재 구역 탈출(Staging Escape) 로직**: B 상자는 Nav2에 곧바로 목표를 전달하기 전에 `_begin_escape`/`_drive_escape`로 좁은 적재 베이에서 벗어나는 별도의 직접 속도 제어 단계를 거칩니다. 이는 Nav2 costmap 인플레이션 반경 안에서 시작할 때 발생하는 계획 실패를 우회하기 위한 것입니다.
- **실패 감지와 자동 재시도 정책** (`amr_control/navigation_policy.py`, `amr_controller.py`): 목표 응답 시간 초과(10초), 전체 주행 시간 초과(`navigation_timeout`), 진행 정체(`stuck_timeout`), 목표 도달 오차 초과를 각각 감지해 `_attempt_failed`로 통합 처리하고, `retry_allowed()` 순수 함수로 재시도 가능 여부를 판정합니다. 이 함수는 별도 단위 테스트(`test_amr_controller.py`)로 검증됩니다.
- **시드 기반 재현 가능한 랜덤 배치 생성** (`conveyor_control/random_spawner.py`): `random.Random(seed)`로 물품 순서를 셔플하고 간격을 균등분포에서 추출하되, 동일 시드에 대해 항상 동일한 배치를 생성하도록 순수 함수(`generate_layout`)로 분리했습니다. 이는 발표 시연 재현성과 단위 테스트 가능성을 동시에 확보하기 위한 설계입니다.
- **HSV 색상 분할 기반 3차원 위치 추정** (`item_vision/vision_node.py`): RGB 이미지를 HSV로 변환한 뒤 클래스별 색상 범위(A: 빨강 2-range, B: 초록, C: 파랑)로 마스크를 생성하고, 모폴로지 연산(open/close)으로 노이즈를 제거한 후 컨투어 중심의 깊이 값과 카메라 내부 파라미터(fx, fy, cx, cy)로 카메라 좌표 → 월드 좌표를 역산합니다. 신뢰도는 형상 solidity와 채도 평균의 가중합으로 계산합니다.
- **의존성 없는 관제 대시보드** (`factory_manager/dashboard_node.py`, `dashboard_page.py`): Flask, FastAPI 등 외부 웹 프레임워크 없이 Python 표준 라이브러리 `ThreadingHTTPServer`만으로 `/`(HTML)와 `/api/status`(JSON 스냅숏) 두 엔드포인트를 구현했습니다. 모든 ROS 2 토픽 콜백은 `threading.Lock`으로 보호된 공유 딕셔너리에 상태를 기록하고, HTTP 요청 시점에 락을 잡고 JSON 직렬화된 스냅숏(`snapshot()`)을 반환합니다.
- **테스트 구조**: 각 패키지가 `ament_python` 표준에 따라 `test/` 디렉터리에 `pytest` 테스트를 두고 있으며, 순수 로직(재시도 정책, 랜덤 배치 생성)은 ROS 노드와 분리된 함수로 작성해 ROS 런타임 없이도 테스트 가능하도록 설계했습니다. `factory_description` 패키지는 SDF 월드 XML을 직접 파싱해 모델·센서·플러그인 구성을 검증하는 통합 테스트를 포함합니다.

## 10. 결과물 및 기대효과

### 사용자가 얻게 되는 결과

실제 로봇팔, AMR, 컨베이어, 산업용 비전 카메라 없이도 컨베이어 투입부터 비전 분류, Pick & Place, 자율주행 배송, 복귀까지 이어지는 전체 스마트 팩토리 물류 사이클을 Gazebo 위에서 반복 재현하고 관찰할 수 있는 시뮬레이션 환경을 얻습니다.

### 기존 방식 대비 개선점

개별 서브시스템(비전, 팔, AMR)을 별도로 검증하는 대신, ROS 2 표준 토픽/서비스/액션 인터페이스로 연결된 하나의 통합 launch 파일에서 전체 신호 흐름을 한 번에 확인할 수 있어, 통합 단계에서 발생하는 인터페이스 불일치 문제를 조기에 발견할 수 있습니다.

### 자동화·효율성·정확성·사용성 측면의 기대효과

- **자동화**: 배송 완료 후 자동 복귀와 다음 배치 생성까지 사람의 개입 없이 무한 반복됩니다.
- **정확성**: 관절 피드백과 AMCL 위치 오차 검증 등 실측값 기반 판정을 사용해 타이밍 추정에 의존하지 않습니다.
- **사용성**: 원버튼 스크립트(`run-demo.sh`)와 브라우저 대시보드로 ROS 2/Gazebo에 익숙하지 않은 참관자도 진행 상황을 쉽게 확인할 수 있습니다.

### 확장 가능성

물품 클래스나 도크 수를 늘리는 구조(`MODEL_BY_CLASS`, `CARRIERS` 등 설정 테이블 중심 설계)이므로, 물품 종류·상자 수·창고 레이아웃을 확장하는 방향으로 비교적 쉽게 발전시킬 수 있습니다.

## 11. 한계점 및 향후 개선 방향

- **비전 인식 방식의 한계**: 현재 `item_vision`은 딥러닝 기반 객체 인식이 아닌 HSV 색상 분할 방식이므로, 조명 변화나 색상이 유사한 물품이 추가될 경우 인식 정확도가 떨어질 수 있습니다. `docs/requirements.md`의 범위 제외 항목에도 "AI 영상 인식"이 명시되어 있어, 딥러닝 기반 인식은 향후 과제로 남아 있습니다.
- **다중 로봇/다중 AMR 미지원**: 현재는 AMR 역할을 이동식 상자 3대가 순차적으로 수행하며, 동시에 여러 대가 병렬로 주행하는 구조는 아닙니다(`docs/requirements.md`의 범위 제외 항목 참고).
- **실시간 경로 최적화 및 생산 스케줄링 미구현**: 현재는 고정된 순서(A → B → C)로 상자가 이동하며, 복잡한 생산 스케줄링이나 동적 우선순위 조정은 범위에 포함되지 않았습니다.

## 콘셉트 영상

[스마트팩토리 구현 스토리 예시 영상 보기](media/industrial_sim_story_example.mp4)

> 이 영상은 최종 Gazebo 실행 결과가 아니라 프로젝트의 목표 공정과 시연 흐름을 설명하기 위한 콘셉트 영상입니다.

## 협업 방법

1. 담당 기능의 브랜치를 생성합니다.
2. 기능을 구현하고 로컬 실행을 확인합니다.
3. Pull Request를 생성합니다.
4. 팀원 한 명 이상의 검토 후 `main`에 병합합니다.

브랜치 이름 예시:

```text
feature/world
feature/conveyor
feature/arm
feature/amr
feature/factory-manager
```

## 문서

- [요구사항](docs/requirements.md)
- [시스템 구조와 인터페이스](docs/architecture.md)
- [팀 작업 분담](docs/team-tasks.md)

## 라이선스

이 저장소는 MIT License를 따릅니다. 자세한 내용은 [LICENSE](LICENSE)를 참고하세요.
