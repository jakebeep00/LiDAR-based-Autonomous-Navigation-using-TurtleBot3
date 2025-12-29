```markdown
# 🏁 TurtleBot3 기반 LiDAR 자율주행 (ROS 2)

## 1. 대회 개요 (Competition Overview)

본 프로젝트는 **국제 대학생 EV 자율주행 경진대회**의 일환으로 수행되었으며,  
**미로형 실내 환경에서 자율주행 로봇이 목표 지점까지 안정적으로 주행하는 것**을 목표로 한다.

### 대회 환경
- 실내 가벽으로 구성된 **미로형 트랙**
- 좁은 통로, 급격한 코너, 반복적인 벽 구조
- GPS 사용 불가 환경
- **LiDAR 기반 SLAM 및 자율주행 필수**

### 로봇 플랫폼
- **ROBOTIS TurtleBot3**
- 2D LiDAR, 휠 엔코더(Odometry)
- ROS 2 기반 분산 시스템

---

## 2. 시스템 아키텍처 (System Architecture)

본 시스템은 **Remote PC 중심 구조**로 설계되었다.

```

[ Remote PC (Ubuntu 22.04 + ROS 2 Humble) ]
├─ Cartographer SLAM
├─ Navigation2 (Nav2)
├─ RViz2 (시각화 및 Goal 설정)
└─ Teleop (키보드 수동 제어)

[ TurtleBot3 ]
├─ 2D LiDAR
├─ Wheel Encoder (Odometry)
└─ cmd_vel Subscriber (모터 제어)

```

- SLAM 및 경로 계획은 **Remote PC**에서 수행
- TurtleBot3는 센서 데이터 송신 및 주행 명령 실행 담당

---

## 3. 개발 환경 (Development Environment)

### OS & Middleware
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill

### 주요 패키지
- `turtlebot3`
- `cartographer_ros`
- `navigation2`
- `gazebo_ros`

### 참고 자료
- ROBOTIS e-Manual  
  https://emanual.robotis.com/docs/en/platform/turtlebot3/
- TurtleBot3 GitHub  
  https://github.com/ROBOTIS-GIT/turtlebot3

---

## 4. 자율주행 전체 파이프라인 (A to Z)

### 4.1 센서 설정 및 통신

- TurtleBot3에서 LiDAR Scan, Odometry 데이터 수집
- ROS 2 DDS 기반으로 Remote PC와 통신
- TF 트리 구조:
```

map → odom → base_link → laser

```

---

### 4.2 수동 주행 (Teleoperation)

초기 지도 생성을 위해 **키보드 기반 teleop 주행**을 수행했다.

**목적**
- 미로 환경 전체를 주행하며 벽면 및 코너 관측
- LiDAR 데이터 충분히 확보
- Loop Closure 발생 조건 생성

**사용 패키지**
- `turtlebot3_teleop`

---

### 4.3 SLAM: Cartographer 기반 지도 생성

본 프로젝트에서는 **Cartographer SLAM**을 사용하여 실시간 지도 생성과 위치 추정을 수행했다.

**특징**
- 2D LiDAR 기반 Graph-based SLAM
- Localization과 Mapping 동시 수행
- Loop Closure를 통한 전역 맵 정합

**발생한 문제**
- 코너 구간에서 지도 왜곡
- 벽이 두껍게 겹쳐 그려지는 현상
- 반복 구조로 인한 Scan Matching 오류

**개선 방법**
- Teleop 주행 속도 감소
- 급격한 회전 최소화
- Cartographer 파라미터 튜닝

→ **주행 방식 자체가 SLAM 품질에 직접적인 영향을 미친다는 점을 실험적으로 확인**

---

### 4.4 맵 완성 및 Localization 안정화

- 충분한 주행 후 Occupancy Grid Map 고정
- SLAM 상태를 Localization 중심으로 안정화
- 이후 Navigation 단계로 전환

---

### 4.5 Navigation (경로 계획)

자율주행은 **ROS 2 Navigation2 (Nav2)** 스택을 사용해 구현했다.

#### Global Planner
- Occupancy Grid 기반 최단 경로 생성
- 벽을 피해 각지고 직선적인 경로 생성

#### Local Planner
- Dynamic Window Approach (DWA/DWB)
- 실시간 속도 샘플링
- 장애물 회피 및 부드러운 곡선 궤적 생성

> Global Path를 Local Planner가 실제 주행 가능한 trajectory로 변환

---

### 4.6 제어 실행 (Control)

- Local Planner에서 생성된 `cmd_vel` 메시지 사용
- TurtleBot3 내부 저수준 PID 제어를 통해 바퀴 속도 제어
- 좁은 미로 환경에서도 안정적인 주행 확보

**제어 파라미터 튜닝**
- 최대 선속도 / 각속도 제한
- Robot footprint 및 inflation radius 조정
- 코너 구간에서 발생하는 진동(oscillation) 현상 완화

---

### 4.7 목표 지점 도달

- RViz에서 2D Nav Goal 설정
- 생성된 경로를 따라 목표 지점까지 자율 주행
- 미로 종료 지점 도달 확인

---

## 5. 프로젝트 핵심 정리 (Key Takeaways)

- SLAM, 경로 계획, 제어는 **독립적인 기능이 아닌 하나의 시스템**
- SLAM 품질이 자율주행 성공률을 결정
- 알고리즘 구현보다 **환경 이해 및 파라미터 조정 능력이 중요**
- 실제 주행 환경에서 발생하는 문제를 반복적으로 분석하고 개선

---

## 6. 요약 (Summary)

> ROBOTIS TurtleBot3 공식 ROS 2 패키지를 기반으로 Cartographer SLAM을 사용해 미로형 실내 환경을 지도화했으며, teleop 주행을 통해 SLAM 품질을 개선한 후 Navigation2를 활용해 생성된 맵 상에서 목표 지점까지 자율 주행을 구현했습니다.

---

## 7. 향후 개선 방향 (Future Work)

- SLAM 파라미터 자동 튜닝
- MPC 기반 제어 기법 적용
- IMU 등 추가 센서 융합
```
