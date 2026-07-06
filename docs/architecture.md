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
| `/factory/state` | `std_msgs/msg/String` | 전체 공정 상태 |

인터페이스를 변경할 경우 관련 담당자와 합의한 후 이 문서를 먼저 수정한다.

## 통합 실행

최종적으로 다음 launch 파일에서 모든 구성요소를 실행한다.

```text
factory_manager/launch/factory_demo.launch.py
```
