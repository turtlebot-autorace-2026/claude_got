# miracle_ws — 2026 STEAMCUP IRC 자율주행(TurtleBot3 AutoRace) 워크스페이스

`turtle_ws`의 시뮬레이션 환경(`~/gazebo_simenv/autorace_sim_share_20261109`, ROS 2 Humble + Gazebo Classic 11)에서
신호등 → 갈림길 → 공사구간 → 주차 → 차단바 → 터널을 처음부터 끝까지 수행한다.

- **차선 SLAM**: 카메라(`/camera`) 차선을 **YOLO 분할**(turtle_ws `lane_yolo26n_seg`, BEV 입력)로 검출 → 마스크를 역원근변환 + IPM으로 지면 점으로 변환 →
  차선 격자맵을 미리 만들고, 주행 중 파티클 필터로 맵 위 위치를 추정 (`lane_detector:=hsv`로 색 임계값 방식 선택 가능)
- **경로 주행**: 맵핑 때 기록한 구간별 경로를 Pure Pursuit로 추종(후진·제자리 회전·횡오프셋 지원)
- **장애물**: LiDAR로 실시간 회피(경로 좌표계 gap 방식)
- **터널**: slam_toolbox로 미리 만든 LiDAR 맵 + Nav2(플래너/RPP 컨트롤러). 터널 안 위치추정은 같은 파티클 필터가 LiDAR 맵 우도를 사용

## 패키지 구조

| 패키지 | 역할 | 주요 노드 |
|---|---|---|
| `autorace_common` | SE2 기하, 경로(Path)·routes.yaml 입출력, JSON 토픽/TF 헬퍼 | (라이브러리) |
| `lane_slam` | 차선 IPM·추출, 차선 격자맵, 상관 매칭(맵핑), 파티클 필터(위치추정)+LiDAR 융합 | `lane_slam_node`, `lane_map_viewer` |
| `route_follower` | 기록 경로 추종(Pure Pursuit), 경로 기록기, 경로 편집 도구 | `route_follower_node`, `route_recorder`, `route_tool` |
| `mission_traffic_light` | 신호등 색 인식(`/camera2`) | `traffic_light_node` |
| `mission_intersection` | 갈림길 이정표(곡선 화살표 ↰/↱) 방향 인식 | `intersection_node` |
| `mission_construction` | LiDAR 실시간 장애물 회피(횡오프셋 생성) | `avoidance_node` |
| `mission_parking` | LiDAR로 하우스 로봇이 없는 주차칸 판별 | `parking_node` |
| `mission_level_crossing` | 차단바 상태(closed/moving/open) 인식 | `level_crossing_node` |
| `mission_tunnel` | slam_toolbox 맵핑 설정, LiDAR 맵 정렬, Nav2 런치/파라미터, Nav2 브리지 | `tunnel_nav_node`, `tunnel_map_align` |
| `autorace_main` | **메인**: 미션 매니저(상태머신), cmd_vel 먹스, 런치, 맵·경로 데이터 | `mission_manager`, `cmd_mux` |
| `autorace_sim_tools` | 시뮬레이션 전용: 맵핑용 가상 텔레옵, 평가 모니터, 하우스 로봇 배치 | `mapping_driver`, `race_monitor`, `spawn_house_robot` |

```
/camera ──► lane_slam ──(TF map→odom, /lane_slam/pose)──► route_follower ──/cmd_vel/route──┐
/scan ────► lane_slam(터널: LiDAR 우도)                        ▲ /route_follower/offset      │
/scan ────► mission_construction ──────────────────────────────┘                            ├─► cmd_mux ─► /cmd_vel
/camera2 ─► traffic_light / intersection / level_crossing ──┐                               │
/scan ────► parking ────────────────────────────────────────┼─► mission_manager ─┬─► route 명령│
                                       Nav2(터널) ─/cmd_vel/nav2──────────────────┼────────────┘
                                                       tunnel_nav ◄─ /tunnel/command ┘
```

미션 매니저(`autorace_main/mission_manager.py`)는 미션마다 제너레이터 함수(`m_traffic_light`, `m_intersection`, …)를 하나씩 갖고
10 Hz로 진행한다. 주행은 경로추종기/Nav2가 하고 매니저는 명령·대기·판단만 한다.

## 준비 (최초 1회)

Nav2·slam_toolbox는 sudo 없이 `third_party/install_nav2_nosudo.sh`로 `third_party/root`에 설치한다(`third_party/setup_extra.bash`로 환경 설정).
sudo 권한이 있으면 `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox`로 대체 가능.

```bash
# 모든 터미널 공통
source ~/gazebo_simenv/autorace_sim_share_20261109/scripts/env.bash   # Humble + sim (ROS_DOMAIN_ID=112)
source ~/miracle_ws/third_party/setup_extra.bash                       # Nav2 / slam_toolbox
cd ~/miracle_ws && colcon build --symlink-install --base-paths src
source ~/miracle_ws/install/local_setup.bash
```
YOLO 차선 추론은 onnxruntime-gpu(또는 torch/ultralytics)가 설치된 파이썬으로 lane_slam 노드를 실행한다.
기본값 `lane_python:=~/autorace/yolo_venv/bin/python` (launch 인자로 변경).

## 1단계: 맵핑 (차선맵 + 경로 + 터널 LiDAR 맵)

맵/경로는 `src/autorace_main/maps`, `src/autorace_main/routes`에 저장된다(환경변수 `AUTORACE_DATA_DIR`로 변경 가능).
**이미 시뮬레이션용 맵과 경로가 들어 있으므로 바로 2단계 실행 가능.**

시뮬레이션(가상 텔레옵이 운전, 공사 장애물이 없는 상태 = 장애물 설치 전 맵핑):
```bash
bash ~/gazebo_simenv/autorace_sim_share_20261109/scripts/run_sim.sh referee:=false   # 터미널 1
for b in barrier_1 barrier_2 barrier_3; do ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: $b}"; done
ros2 launch autorace_main mapping.launch.py session:=full_left lidar:=true             # 세션 A: 전체 코스(왼쪽 갈림길)
# 종료 후 /reset_world 하고
ros2 launch autorace_main mapping.launch.py session:=right_branch extend_map:=true    # 세션 B: 오른쪽 갈림길 추가
```
- 가상 텔레옵(`mapping_driver`)은 Gazebo 정답 자세로 **운전만** 한다. 맵과 경로는 전부 카메라·odom 기반 차선 SLAM 추정 자세로 만들어진다.
- 실차: `sim_driver:=false use_sim_time:=false`로 실행 후 조종기로 같은 순서로 주행하며 구간마다
  `ros2 topic pub --once /route_recorder/segment std_msgs/String "data: '{\"name\": \"start_to_fork\", \"speed\": 0.2}'"`
  처럼 구간 이름을 표시한다(구간 목록·속성은 `autorace_sim_tools/config/mapping_plan.yaml` 참고).
- 확인: `ros2 run lane_slam lane_map_viewer src/autorace_main/maps/lane_map out.png src/autorace_main/routes/routes.yaml`
- 경로 구간 경계·속도 수정: `ros2 run route_follower route_tool list|shift|set ...`

## 2단계: 본 주행

```bash
bash ~/gazebo_simenv/autorace_sim_share_20261109/scripts/run_sim.sh      # 터미널 1 (심판 포함)
ros2 launch autorace_main race.launch.py                                 # 터미널 2: 출발점에서 초록불 대기 → 자동 출발
ros2 run autorace_sim_tools race_monitor --ros-args -p use_sim_time:=true  # (선택) 심판 이벤트·위치오차·접촉 확인
```
- 상태 확인: `ros2 topic echo /mission/state`
- 중간 미션부터 시험: 시뮬레이터를 그 위치에 스폰(`run_sim.sh x:= y:= yaw:=`)하고
  `race.launch.py start_mission:=construction initial_pose:="[mx, my, yaw]"` (map 좌표 = 월드 − 출발점(0.687, −1.999))
- 하우스 로봇 배치(시험용): `ros2 run autorace_sim_tools spawn_house_robot left|right|remove`

## 미션별 동작 요약

| 미션 | 인식 | 행동 |
|---|---|---|
| 신호등 | `/camera2` ROI의 켜진 램프(HSV + 원형 블롭) | 빨강/노랑을 본 뒤 **초록으로 바뀌는 순간** 출발(이미 초록이면 다음 주기 대기) |
| 갈림길 | 파란 원형 표지판, 위/아래 절반 흰색 무게중심 차이로 ↰/↱ 판정, 투표 누적 | 접근 중 판정 실패 시 갈림길 중앙으로 회전해 확인 → `fork_left/right` |
| 공사구간 | LiDAR 점을 경로 좌표(s, l)로 군집화 | 가장 가까운 미통과 장애물 옆의 넓은 빈 공간 중앙으로 횡오프셋 |
| 주차 | 기록 경로 끝점 기준 칸 영역의 LiDAR 점 개수 | 빈칸 진입 → 정지 → 후진 탈출 → 회전 → 복귀 |
| 차단바 | `/camera2` 빨강 줄무늬의 열 분포(closed/moving/open) | S1과 S2 사이 정지 → 내려왔다 올라가면(1 s 유지) 통과, 안 내려오면 3 s 후 통과 |
| 터널 | LiDAR 맵 + 파티클 필터 | 입구까지 경로 → Nav2로 출구를 바라보는 자세까지 → 출구 경로. Nav2 실패 시 기록 경로+회피로 대체 |

## 시뮬레이션 검증 결과 (2026-11-12)

- 차선 SLAM: 맵핑 세션 A/B로 전체 코스 차선맵 생성, 위치추정 오차(정답 대비, 맵 누적오차 포함) 평균 약 5 cm·최대 약 10 cm
- 전체 주행(출발→터널 출구): 심판 판정 start/intersection/barrier 모두 pass, 약 220 s(제한 300 s)
- 개별 시험: 갈림길 left/right, 주차(하우스 로봇 왼쪽/오른쪽 배치 시 반대 칸 선택), 공사구간 접촉 0회, 터널 Nav2 통과
- 시뮬레이터 심판은 공사·주차·터널 성공을 판정하지 않으므로 `race_monitor`의 접촉 감지와 궤적으로 확인

## 실차 적용 시 주의

- `/odom`이 시뮬레이터에서는 사실상 정답이다. 실차 odom은 오차가 커서 `lane_slam` 매칭 파라미터(`match_gain` 등)와
  파티클 필터 잡음(`ParticleFilter.alpha`)을 조정해야 한다. 맵핑 루프 폐합(loop closure)은 구현하지 않았다.
- 카메라 외부파라미터: TF가 없으면 `lane_slam.yaml`의 `cam_*` 값을 실측으로 넣는다. 차선 HSV 임계값도 조명에 맞게 조정.
- 신호등·이정표·차단바 HSV/ROI는 시뮬레이터 렌더링 기준이다. 실물 영상으로 다시 확인한다.
- 시뮬레이터 터널 출입구 폭은 약 0.23 m로 규정(0.3 m)보다 좁다. Nav2 `robot_radius`·inflation은 이에 맞춰 작게 잡았다.
