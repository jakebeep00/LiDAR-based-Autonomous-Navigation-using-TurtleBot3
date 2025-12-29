# LiDAR-based-Autonomous-Navigation-using-TurtleBot3

1. Competition Overview

본 프로젝트는 국제 대학생 EV 자율주행 경진대회의 일환으로 수행되었으며,
미로형 실내 주행 환경에서 자율주행 로봇이 목표 지점까지 안정적으로 도달하는 것을 목표로 한다.

Competition Environment

실내 가벽으로 구성된 미로형 트랙

직선 구간 + 급격한 코너 + 반복 구조

GPS 사용 불가

LiDAR 기반 인지 및 위치 추정 필수

Robot Platform

ROBOTIS TurtleBot3

2D LiDAR, Wheel Encoder 기반

ROS 2 통신 구조 사용

2. System Architecture

본 시스템은 Remote PC 중심 구조로 설계되었다.

[ Remote PC (Ubuntu 22.04 + ROS 2 Humble) ]
 ├─ Cartographer SLAM
 ├─ Navigation2 (Nav2)
 ├─ RViz2 (Visualization & Goal Input)
 └─ Teleop (Manual Control)

[ TurtleBot3 ]
 ├─ 2D LiDAR
 ├─ Wheel Encoder
 └─ cmd_vel Subscriber (Motor Control)


연산량이 큰 SLAM 및 Navigation은 Remote PC에서 수행

TurtleBot3는 센서 데이터 송신 및 주행 명령 실행 역할

3. Development Environment
OS & Middleware

Ubuntu 22.04 LTS

ROS 2 Humble Hawksbill

Core Packages

turtlebot3

cartographer_ros

navigation2

gazebo_ros

Reference

ROBOTIS e-Manual: https://emanual.robotis.com/docs/en/platform/turtlebot3/

TurtleBot3 GitHub: https://github.com/ROBOTIS-GIT/turtlebot3

4. Autonomous Driving Pipeline (A to Z)

본 프로젝트는 자율주행 전 과정을 순차적으로 수행하였다.

4.1 Sensor Setup & Communication

TurtleBot3에서 LiDAR scan, Odometry 데이터 수집

ROS 2 DDS 기반으로 Remote PC와 통신

TF Tree 구성:

map → odom → base_link → laser

4.2 Manual Teleoperation (Teleop)

초기 SLAM 수행을 위해 키보드 기반 teleop 주행을 진행했다.

목적:

로봇이 관측할 수 있도록 트랙 전체를 주행

모든 벽면 및 코너를 LiDAR로 스캔

Loop Closure 발생 조건 확보

사용 패키지:

turtlebot3_teleop

4.3 SLAM: Cartographer-based Mapping

본 프로젝트에서는 Cartographer SLAM을 사용하여 지도 생성을 수행했다.

SLAM Characteristics

2D LiDAR 기반 Graph-based SLAM

실시간 Localization + Mapping 동시 수행

Loop Closure를 통한 지도 정합

Observed Issues

코너 구간에서 맵 왜곡

벽이 두껍게 겹쳐 그려지는 현상

반복 구조로 인한 scan matching 오류

Improvements

Teleop 주행 속도 감소

급격한 회전 최소화

Cartographer 파라미터 조정

→ 주행 방식 자체가 SLAM 품질에 직접적인 영향을 미침을 확인

4.4 Map Finalization & Localization

충분한 주행 후 Occupancy Grid Map 고정

SLAM 상태를 Localization 중심으로 안정화

이후 Navigation 단계로 전환

4.5 Navigation (Path Planning & Control)

Navigation은 ROS 2 Navigation2 (Nav2) 스택을 사용했다.

Global Planning

Occupancy Grid 기반 경로 생성

벽을 피해 목표 지점까지 최단 경로 계산

각지고 직선적인 경로 형태

Local Planning

Dynamic Window Approach(DWA/DWB) 기반

실시간 속도 샘플링

장애물 회피 + 부드러운 곡선 주행

Global Path → Local Planner를 통해 실제 주행 가능한 trajectory로 변환

4.6 Control Execution

Local Planner에서 생성된 cmd_vel 메시지 송신

TurtleBot3 내부 저수준 PID 제어를 통해 바퀴 속도 제어

좁은 미로 환경에서도 안정적인 주행 확보

Control Tuning

최대 선속도 / 각속도 제한

Robot footprint 및 inflation radius 조정

코너 구간 진동(oscillation) 현상 완화

4.7 Goal Achievement

RViz에서 2D Nav Goal 설정

생성된 경로를 따라 목표 지점까지 자율 주행 성공

미로 끝 지점 도달 확인

5. Key Takeaways

SLAM, Planning, Control은 독립적인 모듈이 아닌 하나의 시스템

SLAM 품질이 Navigation 성공률을 결정

알고리즘 구현보다 환경 이해와 파라미터 조정 능력이 중요

실제 주행 환경에서 발생하는 문제를 반복적으로 분석하고 개선

6. Summary

ROBOTIS TurtleBot3 공식 ROS 2 패키지를 기반으로 Cartographer SLAM을 사용해 미로형 실내 환경을 지도화했으며, teleop 주행을 통해 SLAM 품질을 개선한 후 Navigation2를 활용해 생성된 맵 상에서 목표 지점까지 자율 주행을 구현했습니다.

7. Future Work

SLAM 파라미터 자동 튜닝

MPC 기반 제어기 적용

센서 융합 (IMU)
