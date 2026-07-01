# 팀 작업 분담

| 역할 | 주요 작업 | 담당 패키지 |
| --- | --- | --- |
| 팀장·통합 | 상태 머신, 통합 launch, 인터페이스 관리 | `factory_manager` |
| 월드 담당 | 공장 월드와 모델 배치 | `factory_description` |
| 컨베이어 담당 | 물품 생성, 이동 및 정지 | `conveyor_control` |
| 로봇팔 담당 | Pick & Place 구현 | `arm_control` |
| AMR 담당 | 출발지에서 창고까지 이동 | `amr_control` |

## 공통 작업 원칙

- 각 기능은 다른 패키지에 직접 의존하기보다 문서화된 ROS 2 인터페이스로 연결한다.
- Pull Request에는 실행 방법과 확인 결과를 작성한다.
- 생성된 `build`, `install`, `log` 폴더는 커밋하지 않는다.
- 대용량 시연 영상은 일반 Git 커밋에 포함하지 않는다.

## GitHub Issue 목록

- `[WORLD] 공장 Gazebo 월드 구성`
- `[CONVEYOR] 물품 이동 및 정지 구현`
- `[ARM] Pick & Place 구현`
- `[AMR] 출발지에서 창고까지 이동`
- `[INTEGRATION] 공정 상태 머신과 통합 launch 구현`
- `[DOCS] 실행 방법 및 시연 영상 시나리오 작성`
