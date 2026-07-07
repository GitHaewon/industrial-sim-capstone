# 시스템 구조와 인터페이스

## 공정 상태

```text
IDLE
→ CONVEYOR_RUNNING
→ ITEM_READY
→ PICKING
→ BOX_READY
→ AMR_MOVING
→ DELIVERED
```

`factory_manager`가 상태 전이를 관리하고 각 담당 노드는 작업 완료 신호를
발행한다.

## ROS 2 인터페이스 초안

| 이름 | 타입 | 설명 |
| --- | --- | --- |
| `/conveyor/start` | `std_msgs/msg/Bool` | 컨베이어 시작 명령 |
| `/conveyor/stop` | `std_msgs/msg/Bool` | 컨베이어 정지 명령 |
| `/item/ready` | `std_msgs/msg/Bool` | 물품 작업 위치 도착 |
| `/arm/start_pick` | `std_msgs/msg/Bool` | Pick & Place 시작 |
| `/arm/task_complete` | `std_msgs/msg/Bool` | 로봇팔 작업 완료 |
| `/box/item_count` | `std_msgs/msg/Int32` | 현재 박스 적재 수량 |
| `/box/ready` | `std_msgs/msg/Bool` | 박스 적재 완료 |
| `/amr/start_delivery` | `std_msgs/msg/Bool` | A/B/C 적재 상자 운송 시작 |
| `/amr/delivery_complete` | `std_msgs/msg/Bool` | 세 상자의 구역 배송 완료 |
| `/amr/state` | `std_msgs/msg/String` | AMR 운송·재시도·도착 상태 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 현재 운송 상자의 360도 LiDAR |
| `/nav/odom` | `nav_msgs/msg/Odometry` | 현재 운송 상자의 휠 오도메트리 |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 지도 기준 AMCL 추정 위치 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 상자 전환 시 AMCL 초기 위치 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Nav2 전역 costmap |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Nav2 지역 costmap |
| `/plan` | `nav_msgs/msg/Path` | 현재 Nav2 전역 계획 경로 |
| `/factory/state` | `std_msgs/msg/String` | 전체 공정 상태 |

인터페이스를 변경할 경우 관련 담당자와 합의한 후 이 문서를 먼저 수정한다.

## 통합 실행

최종적으로 다음 launch 파일에서 Gazebo, 센서 브리지, AMCL, Nav2와
공정 제어 노드를 함께 실행한다.

```text
factory_description/launch/factory_test.launch.py
```

기본 실행은 RViz2 시각화와 동적 팔레트 장애물 데모를 함께 켠다.
`rviz:=false` 또는 `dynamic_obstacle:=false` launch 인자로 각각 비활성화할
수 있다. 동적 장애물 노드는 `/amr/state`가 `NAVIGATING:B`일 때
`/world/factory_test/set_pose` 서비스를 사용해 `moving_pallet_obstacle`
모델을 B 박스의 첫 Nav2 주행 구간 근처에서 짧게 이동시킨 뒤 안전 위치로
복귀시킨다.
