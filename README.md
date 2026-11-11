# miracle_ws — 2026 STEAMCUP IRC AutoRace (Jetson Orin TurtleBot3)

Jetson Orin에 올린 TurtleBot3(Burger, 카메라 2대 + LDS LiDAR)가 경기장에서
**신호등 → 갈림길 → 공사구간 → 주차 → 차단바 → 터널**을 처음부터 끝까지 수행하기 위한 ROS 2 워크스페이스.

| 기능 | 방식 |
|---|---|
| 위치 파악 | **차선 SLAM**: `/camera` 차선을 YOLO 분할(turtle_ws `lane_yolo26n_seg`) → IPM 지면 투영 → 미리 만든 차선 격자맵에서 파티클 필터로 위치추정 |
| 주행 | 맵핑 때 기록한 구간별 경로를 Pure Pursuit로 추종(후진·제자리 회전·끼임 복구) |
| 장애물 | LiDAR 실시간 회피(경로 좌표계 gap 방식, 횡오프셋) |
| 터널 | slam_toolbox로 미리 만든 LiDAR 맵 + Nav2. 터널 안 위치추정은 같은 파티클 필터가 LiDAR 맵 우도 사용 |
| 표지/장치 | `/camera2` 영상 규칙 기반(신호등 램프, 곡선 화살표 이정표, 차단바 줄무늬), 주차칸은 LiDAR |

---

## 1. 시스템 구성

```
[TurtleBot3 bringup]  /scan, /odom, TF(odom→base_footprint→base_scan), /cmd_vel 수신
[camera_ros]          /camera  (차선 카메라, 보정(undistort)된 640x480 JPEG, CompressedImage)
                      /camera2 (전방 표지판 카메라, 640x480 JPEG)
            │
            ▼  miracle_ws
lane_slam ─(TF map→odom)─► route_follower ─/cmd_vel/route─┐
   ▲ /camera, /scan             ▲ /route_follower/offset   │
mission_construction ───────────┘                          ├─► cmd_mux ─► /cmd_vel
traffic_light / intersection / level_crossing (/camera2)   │
parking (/scan) ──────────► mission_manager ── 명령 ───────┤
Nav2(터널) ─/cmd_vel/nav2─────────────────────────────────┘
```

| 패키지 | 역할 |
|---|---|
| `autorace_common` | SE2 기하, 경로/`routes.yaml` 입출력, JSON 토픽·TF 헬퍼 |
| `lane_slam` | 차선 검출(YOLO/HSV), IPM, 차선맵 맵핑, 파티클 필터 위치추정(+LiDAR) |
| `route_follower` | 경로 추종 `route_follower_node`, 경로 기록 `route_recorder`, 구간 표시 `route_mark`, 경로 편집 `route_tool` |
| `mission_traffic_light` | 신호등 색 인식 |
| `mission_intersection` | 갈림길 이정표(↰/↱) 방향 인식 |
| `mission_construction` | LiDAR 장애물 회피 |
| `mission_parking` | 하우스 로봇이 없는 주차칸 판별 |
| `mission_level_crossing` | 차단바 상태(closed / moving / open) |
| `mission_tunnel` | slam_toolbox 맵핑 설정, LiDAR 맵 정렬, Nav2 설정·런치, Nav2 브리지 |
| `autorace_main` | **메인**: 미션 매니저 상태머신, cmd_vel 먹스, 런치, 맵·경로 데이터 |
| `autorace_sim_tools` | (시뮬레이션 전용) 가상 텔레옵, 평가 모니터, 하우스 로봇 배치 |

입력 토픽 이름이 다르면 각 노드의 `camera_topic`/`image_topic` 파라미터나 launch remap으로 맞춘다.

---

## 2. Jetson 설치

### 2.1 ROS 2 / 의존 패키지
ROS 2, TurtleBot3 bringup, 카메라 패키지(turtle_ws `camera_ros`)가 이미 동작한다고 가정한다.
```bash
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-slam-toolbox \
                 python3-opencv python3-numpy python3-yaml
```
sudo가 없으면 `third_party/install_nav2_nosudo.sh`로 `.deb`를 받아 `third_party/root`에 풀고
`source third_party/setup_extra.bash`를 실행한다(x86 Humble에서 검증).

> **ROS 배포판**: Jetson 실차는 ROS 2 **Galactic**(JetPack 5 / Ubuntu 20.04 / Python 3.8) 기준이다.
> 터널 Nav2는 `ROS_DISTRO`를 보고 설정을 자동으로 고른다(`ros_distro:=` 인자로 강제 가능).
> - galactic: `mission_tunnel/config/nav2_tunnel_galactic.yaml` + `nav2_recoveries/recoveries_server`
>   (navigation2 galactic 브랜치 기본 파라미터 기준 BT 플러그인 목록·RPP 파라미터 이름)
> - humble: `nav2_tunnel_humble.yaml` + `nav2_behaviors/behavior_server` (개발 PC 시뮬레이션용)
>
> Python 노드는 3.8 문법만 쓰고, Humble에만 있는 rclpy API(`try_shutdown`, `ExternalShutdownException`)는 있을 때만 쓴다.
> 개발 PC(Humble)에서는 Galactic 설정을 실제로 실행해 보지 못했다. Jetson에서 처음 실행할 때 Nav2 노드 로그를 꼭 확인한다.

### 2.2 YOLO 추론 환경 (onnxruntime-gpu)
차선 YOLO는 ONNX 모델(`lane_slam/models/lane_yolo26n_seg.onnx`, 입력 480×640)을 onnxruntime으로 추론한다. torch는 필요 없다.
- JetPack에 맞는 `onnxruntime-gpu` 휠(Jetson Zoo)을 설치한다. pip의 일반 `onnxruntime`은 CPU 전용이다.
- TensorRT가 있으면 첫 실행 때 FP16 엔진을 빌드해 `~/.cache/yolo_trt`에 저장한다. 수 분 걸리고, 이후에는 재사용한다.
- onnxruntime을 시스템 파이썬(`/usr/bin/python3`)에 설치했다면 따로 설정할 것이 없다.
  다른 venv에 설치했다면 `export AUTORACE_LANE_PYTHON=/경로/venv/bin/python`(또는 launch 인자 `lane_python:=`).
- GPU를 쓸 수 없으면 `lane_detector:=hsv`(색 임계값 방식)로 바로 대체할 수 있다.

### 2.3 빌드
```bash
cd ~/miracle_ws
colcon build --symlink-install --base-paths src
source install/setup.bash
```
`--symlink-install`로 빌드하면 맵핑 결과(맵·경로)가 바로 `src/autorace_main/{maps,routes}`에 저장된다.
다른 위치에 저장하려면 `export AUTORACE_DATA_DIR=/경로`.

---

## 3. 실차 보정 (경기장 도착 후 먼저)

| 항목 | 위치 | 방법 |
|---|---|---|
| 차선 카메라 장착 | `lane_slam/config/lane_slam.yaml` `cam_x, cam_y, cam_z, cam_pitch, cam_yaw` | 실측한다. 기본값은 높이 0.135 m, 아래로 31.6°(0.5515 rad). TF에 `camera_optical_frame`이 있으면 TF가 우선 |
| 차선 카메라 내부 파라미터 | `cam_intrinsics: [fx, fy, cx, cy]` | `/camera/camera_info`가 없을 때 사용. 기본값은 보정 후 640×480 `/camera`(bag 기준) |
| 지면 높이 | `ground_z` | base_footprint 기준 지면 z(보통 0 ~ -0.01) |
| IPM 확인 | — | 차선 위에 로봇을 두고 `/lane_slam/status`의 `npts`(차선 점 수)를 확인 |
| 신호등 | `mission_traffic_light` 파라미터 `roi`, `min_area` | `debug_images:=true` 후 `/traffic_light/debug` 확인 |
| 이정표·차단바 | `mission_intersection`, `mission_level_crossing` 파라미터 | `debug_images:=true` 후 `/intersection/debug`, `/level_crossing/debug` 확인 |
| HSV 방식 사용 시 | `white_v_min`, `yellow_*` | 조명에 맞게 조정 |

```bash
ros2 launch autorace_main race.launch.py debug_images:=true start_mission:=finish  # 인식 노드만 동작(주행 안 함)
ros2 topic echo /lane_slam/status
ros2 run rqt_image_view rqt_image_view
```

---

## 4. 1단계: 맵핑 (실제 경기장에서 반드시 새로)

차선맵·경로·터널 LiDAR 맵은 **실제 경기장에서 새로 만든다**. 저장소에 들어 있는 맵은 시뮬레이션용이라 실차에 쓸 수 없다.
- 시뮬레이터 맵은 실물과 축척이 다르다(차선 중심 간격: sim 약 286 mm, 실물 bag 기준 약 249 mm).
- 차선맵은 그 로봇의 카메라 보정값으로 만들어진다. 장착 각도가 조금만 달라도 맵이 어긋난다.
- 경로는 맵을 만들 때의 SLAM 좌표로 기록되므로, 맵과 경로는 항상 같은 맵핑 주행에서 함께 만들어야 한다.
- 맵핑 1회(세션 A+B)는 약 5~6분 걸린다. 경기의 준비시간(5분)으로는 빠듯하므로, 연습 주행 시간에 미리 맵핑해 둔다.
  대회 당일 코스 배치가 바뀌면 그날 다시 맵핑해야 한다.
- 맵 좌표계(map)의 원점은 **맵핑을 시작할 때 로봇 자세**, 즉 출발선 뒤 출발 위치다. 본 주행도 같은 위치·방향에서 시작해야 한다.
- 공사구간 장애물과 하우스 로봇은 가능하면 치운 상태에서 맵핑한다(차선이 가려지지 않게).

```bash
# 터미널 1: 로봇 bringup + 카메라 (기존 방식)
# 터미널 2: 맵핑 (차선 SLAM + 경로 기록 + slam_toolbox + LiDAR 맵 정렬)
ros2 launch autorace_main mapping.launch.py lidar:=true
# 터미널 3: 조종 (차선 중앙을 따라 천천히, 매끄럽게)
ros2 run turtlebot3_teleop teleop_keyboard
# 터미널 4: 구간 표시 (각 구간 시작 직전에 실행)
ros2 run route_follower route_mark start_to_fork 0.2
```
터널은 안을 지나가기만 하면 slam_toolbox가 맵을 만든다.

### 구간 순서 (세션 A: 왼쪽 갈림길로 전체 코스)
| 구간 | 명령 | 주행 내용 |
|---|---|---|
| `start_to_fork` | `route_mark start_to_fork 0.2` | 출발 위치 → 갈림길 입구(이정표 앞) |
| `fork_left` | `route_mark fork_left 0.18` | 왼쪽 갈래 → 합류 직후 |
| `merge_to_construction` | `route_mark merge_to_construction 0.2` | 합류 → 공사구간 진입 커브 전 |
| `construction` | `route_mark construction 0.15 --max-offset 0.17` | 공사구간 중앙선(첫 장애물 전 ~ 마지막 장애물을 지나 커브까지) |
| `to_parking` | `route_mark to_parking 0.2` | 주차장 진입로, 칸들 바로 앞에서 정지 |
| `park_left_in` | `route_mark park_left_in 0.12` | 진행방향 왼쪽 칸 안으로 완전히 진입 |
| `park_left_out` | `route_mark park_left_out 0.12 --reverse` | 후진으로 진입로 정지점까지 |
| `park_right_in` / `park_right_out` | 위와 같음 | 오른쪽 칸 |
| (정지·제자리 회전) | `route_mark pause` | 진입로에서 출구 방향으로 회전 |
| `parking_exit` | `route_mark parking_exit 0.18` | 진입로 → 본 도로 합류 |
| `to_crossing` | `route_mark to_crossing 0.2` | 차단바 앞 약 0.3 m(1번 센서와 2번 센서 사이)에서 정지 |
| `crossing_to_tunnel` | `route_mark crossing_to_tunnel 0.2` | 차단바 통과 → 터널 입구 앞 |
| `tunnel_entry` | `route_mark tunnel_entry 0.12` | 터널 입구 안쪽 0.3 m까지 |
| `tunnel_inside` | `route_mark tunnel_inside 0.12 --max-offset 0.15` | 터널 내부(Nav2 실패 시 쓰는 대체 경로) |
| `tunnel_exit` | `route_mark tunnel_exit 0.12` | 출구 앞 → 출구 밖 |
| 저장 | `route_mark save` 후 Ctrl+C | 차선맵·경로·`tunnel_map.yaml` 저장 |

### 세션 B: 오른쪽 갈림길 추가
로봇을 다시 **출발 위치**에 두고 기존 맵을 확장한다.
```bash
ros2 launch autorace_main mapping.launch.py extend_map:=true
ros2 run route_follower route_mark pause              # 출발~갈림길은 기록하지 않고 주행
ros2 run route_follower route_mark fork_right 0.18    # 갈림길 입구에서
ros2 run route_follower route_mark save
```

### 결과 확인·수정
```bash
ros2 run lane_slam lane_map_viewer src/autorace_main/maps/lane_map /tmp/map.png src/autorace_main/routes/routes.yaml
ros2 run route_follower route_tool list  src/autorace_main/routes/routes.yaml
ros2 run route_follower route_tool shift src/autorace_main/routes/routes.yaml merge_to_construction construction 0.5   # 구간 경계 이동
ros2 run route_follower route_tool set   src/autorace_main/routes/routes.yaml construction speed 0.15
```
구간 이름·속성 예시는 시뮬레이션 맵핑 계획 `autorace_sim_tools/config/mapping_plan.yaml`과 같다.

---

## 5. 2단계: 본 주행

로봇을 **맵핑을 시작한 출발 위치·방향**에 놓고 실행한다.
```bash
ros2 launch autorace_main race.launch.py
ros2 topic echo /mission/state     # 진행 상황
```
- 미션 매니저는 lane_slam이 준비되면 신호등을 본다. **빨강/노랑을 본 뒤 초록으로 바뀌는 순간** 자동 출발한다.
  이미 초록이면 다음 주기를 기다리므로 노란불에 출발선을 넘는 실패를 막는다.
- 30초 정지 규정은 출발 이후부터 적용되므로 신호 대기는 문제없다.
- 중간 미션부터 시험: `start_mission:=parking initial_pose:="[x, y, yaw]"`(map 좌표).
- 비상 정지: `ros2 topic pub --once /cmd_mux/select std_msgs/String "data: stop"`.

### 미션별 동작
| 미션 | 인식 | 행동 |
|---|---|---|
| 신호등 | `/camera2` ROI의 켜진 램프(HSV + 원형 블롭, 3프레임 연속 확인) | 초록으로 바뀌는 순간 출발 |
| 갈림길 | 파란 원형 표지판 → 원 안 흰 화살표의 위/아래 무게중심 차이로 ↰/↱ 판정, 투표 누적 | 도착 시 미확정이면 갈림길 중앙으로 회전해 확인(최대 8 s) → `fork_left/right` |
| 공사구간 | LiDAR 점을 경로 좌표로 군집화 | 아직 지나지 않은 가장 가까운 장애물 옆의 넓은 빈 공간 중앙으로 횡이동(접촉은 감점 없음) |
| 주차 | `park_*_in` 경로 끝 기준 칸 영역의 LiDAR 점 수 | 빈칸 진입 → 1 s 정지 → 후진 탈출 → 회전 → 복귀 |
| 차단바 | `/camera2` 빨강 줄무늬의 열 분포 | 정지점에서 차단바가 그대로면 0.05 m/s로 최대 0.12 m 서행 접근(1번 센서 감지 유도) → 내려오면 정지 → 올라가서 1 s 유지되면 통과 |
| 터널 | LiDAR 맵 + 파티클 필터 | 입구까지 경로 → Nav2로 출구를 정면으로 보는 자세까지 → 출구 경로. Nav2 실패 시 `tunnel_inside` + 회피 |

대기 시간·속도 배율·터널 Nav2 사용 여부 등은 `autorace_main/config/mission.yaml`에서 설정한다.

---

## 6. 실차 튜닝 포인트

- **odom 오차**: 시뮬레이터 odom은 거의 정답이라 실차보다 훨씬 좋다. 실차 위치추정이 흔들리면
  `lane_slam`의 파티클 필터 잡음(`estimator.ParticleFilter.alpha`), `pf_gain`, 맵핑 매칭 `match_gain`을 조정한다.
  맵핑에 루프 폐합(loop closure)이 없으므로 맵핑은 천천히, 한 번에 매끄럽게 주행한다.
- **속도**: Burger 최대 속도 0.22 m/s(cmd_mux에서 제한). 시간이 부족하면 `mission.yaml`의 `speed_scale`을 올린다.
- **처리 시간**: YOLO 차선 추론은 GTX 1060 기준 프레임당 약 30~40 ms. Jetson에서는 TensorRT FP16을 권장한다.
  `/lane_slam/status` 발행이 카메라 주기(약 15 Hz)를 못 따라가면 `bev_stride`를 키우거나 `lane_detector:=hsv`를 쓴다.
- **차단바 정지점**: 1번 센서 위치는 대회 당일 바뀔 수 있다. `to_crossing` 경로 끝을 차단바 앞 약 0.3 m에 두면
  서행 접근 로직이 센서를 밟게 된다. 2번 센서(차단바 6 cm 앞)를 넘지 않도록 `crossing_creep_dist`를 조정한다.
- **터널**: Nav2 `robot_radius` 0.10, inflation 0.18. 실물 출입구는 규정상 주행로 폭(0.3 m)이라 시뮬레이터(약 0.23 m)보다 여유가 있다.

---

## 7. 시뮬레이션 검증 (개발 PC)

`~/gazebo_simenv/autorace_sim_share_20261109`(Humble + Gazebo Classic 11).
```bash
source ~/gazebo_simenv/autorace_sim_share_20261109/scripts/env.bash
bash ~/gazebo_simenv/autorace_sim_share_20261109/scripts/run_sim.sh            # GUI 포함
export AUTORACE_LANE_PYTHON=~/autorace/yolo_venv/bin/python                     # onnxruntime-gpu가 있는 venv
ros2 launch autorace_main race.launch.py use_sim_time:=true
ros2 run autorace_sim_tools race_monitor --ros-args -p use_sim_time:=true       # 심판 이벤트·위치오차·접촉
ros2 run autorace_sim_tools spawn_house_robot left|right|remove                 # 하우스 로봇 배치
```
시뮬레이션 맵핑에서는 가상 텔레옵(`mapping_driver`)이 Gazebo 정답 자세로 **운전만** 하고, 맵·경로는 차선 SLAM 추정으로 만든다.
순서: `run_sim.sh referee:=false` → 공사 장애물 삭제 →
`mapping.launch.py use_sim_time:=true sim_driver:=true session:=full_left lidar:=true` → 출발점 리셋 → `... session:=right_branch extend_map:=true`.

검증 결과(2026-11-12, YOLO 차선):
- 출발 → 터널 출구 약 190 s(제한 300 s)
- 심판 판정 start / intersection(left·right) / barrier 모두 pass
- 공사구간 접촉 0회, 주차는 하우스 로봇이 없는 칸 선택
- 위치추정 오차 평균 약 6 cm

시뮬레이터 심판은 공사·주차·터널을 판정하지 않으므로 이 세 미션은 궤적과 모니터로 확인했다. **실차 검증은 아직 하지 않았다.**
