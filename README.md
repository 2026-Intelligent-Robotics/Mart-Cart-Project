# 🛒 Mart-Cart-Project

## 🎯 프로젝트 목표 및 주제

* **주제**: TIAGO 기반 상품 자동 적재 마트 카트 시스템 개발 (사용자 추종 및 능동적 상품 인지)
* **프로젝트 목표**: 본 프로젝트는 대형 마트 환경에서 사용자를 안전하게 추종하고, 사용자가 지정한 물품을 로봇팔로 자동 적재하는 시스템 구축을 목표로 한다.

---

## 🔄 시스템 시나리오 (Step 1 ~ 5)

### **Step 1. 사용자 추종**
* **상황**: 사용자가 키보드로 사람 캐릭터를 조작해 이동 시작
* **로봇 동작**: 인식된 사람과의 최단 거리를 LiDAR 센서로 실시간 파악하여 정해진 간격(1m)을 유지하며 추종

### **Step 2. 섹션 도착 및 정지**
* **상황**: 사용자가 원하는 진열대(과일 섹션 등) 앞에서 캐릭터를 정지시킴
* **로봇 동작**: 사람이 멈춘 것을 감지하고 사람과 0.45m ~ 1.0m 거리에서 정지

### **Step 3. 물체 지정 및 인식**
* **상황**: 사용자가 원하는 물건을 선택 (터미널에 입력)
* **로봇 동작**:
  1. 로봇의 머리를 돌려 카메라가 진열대를 스캔
  2. 스캔한 물건의 실시간 3D 좌표($x, y, z$)를 추출
  3. 터미널에서 물건 종류를 사용자 input으로 받음
  4. 사용자가 원하는 물건을 집기 위한 로봇 팔의 목표 지점 계산

### **Step 4. 피킹 및 바구니 적재**
* **로봇 동작**:
  1. 계산된 경로로 로봇팔 뻗어 물체 파지
  2. 파지한 물체를 들어 올려 로봇에 설치된 바구니 위치로 이동
  3. Gripper를 열어 물건 놓기
  4. 적재 완료 후 로봇팔을 주행용 기본 자세로 복귀

### **Step 5. 시나리오 반복 및 종료**
* **로봇 동작**: 로봇의 머리를 돌려 카메라 각도를 다시 정면으로 복귀
* **반복**: 사용자가 다시 이동을 시작하면 LiDAR 센서가 거리 변화를 감지하고, 사람 추종 모드로 자동 전환 후 다시 Step 1부터 시작
* **종료**: 사람이 계산대 근처 일정 반경 내에 진입 $\rightarrow$ 로봇과 사람이 동시에 진입 $\Rightarrow$ 모든 태스크를 완료하고 시나리오 종료

---

## 🛠 구현한 노드 (Nodes)

1. 👤 **센서 기반 사람 추종 노드 (`human_following.py`)**
   * LiDAR 센서 데이터를 기반으로 고정 장애물(벽)을 분리하고, 사람만을 정밀하게 추종하는 노드
2. 🏪 **진열대 가까이 이동 노드 (`shelf_docking.py`)**
   * TIAGO 로봇이 물체를 잡기 편하도록 진열대를 정면으로 바라보고, 진열대 가까이 밀착하도록 하는 노드
3. 🔍 **물체 인식 노드 (`object_detector.py`)**
   * 로봇이 상품을 감지하고, 선택할 상품을 사용자로부터 입력받은 뒤 상품의 좌표를 반환하는 노드
4. 🦾 **물체 피킹 노드 (`picking_node.py`)**
   * 물체 인식 노드로부터 받은 좌표를 바탕으로 상품을 집어서 바구니 안에 넣은 뒤 다시 사람 추종을 시작하는 노드

---

## 🤖 로봇 커스텀 (Robot Customization)

<img width="500" alt="로봇 커스텀 이미지" src="https://github.com/user-attachments/assets/2071f111-b18b-4e51-b255-ccc5ba41bde0" />

* **기본 모델**: PAL Robotics TIAGO 모델
* **그리퍼**: TIAGO 패키지에서 제공하는 집게 모양의 `pal-gripper` 사용
* **기구부 설정**: 로봇의 키를 최대 길이인 **145cm**로 설정
* **바구니 추가**: TIAGO의 하단 구동부(`base_link`) 앞부분을 확장하여, 가로 40cm, 세로 40cm, 높이 30cm 규격의 **적재용 바구니**를 추가

---

## 🧱 마트 환경 세팅 (World Environment)

<img width="522" height="680" alt="마트 환경 세팅 이미지" src="https://github.com/user-attachments/assets/e45cd83a-262b-4757-81e5-a6e70f57cf4b" />

* 마트 벽 구성
* 과일/채소 진열대 (L자형 배치)
* **피킹 대상 물체**: 바나나, 사과, 당근, 브로콜리 (각 2개씩 배치)

---

## 🧍 사람 모델 (Human Model)

<img width="200" alt="사람 모델 이미지" src="https://github.com/user-attachments/assets/498a65da-71f0-45a3-a6c5-ecbef4e9d283" />

* **기구부 구성**: 몸체(cylinder) + 상체(box) + 머리(sphere) + 바퀴 2개 + 캐스터 2개로 구성된 diff-drive 이동 가능 모델
* **플러그인**: `libgazebo_ros_diff_drive.so` 플러그인 사용
* **제어 방식**: `/person/cmd_vel` 토픽으로 이동을 제어하며, 로봇의 사람 추종 노드가 해당 모델을 따라가도록 설계

---

## 👥 역할 분담 (Role Division)

| 테스크 | 로봇 세팅 | 마트 환경 세팅 | 사용자 인터렉션 및 인지 | 매니퓰레이션 작업 |
| :--- | :--- | :--- | :--- | :--- |
| **세부 작업** | • TIAGo 로봇 모델(URDF) 설정<br>• Gazebo 파라미터 최적화 | • 마트 진열대 및 상품 배치<br>• 키보드로 사람 조종 | • RGB-D 카메라를 활용한 사용자 식별 및 추종 알고리즘<br>• 상품 인식 | • 7-DOF 로봇 팔의 궤적 생성<br>• 그리퍼 제어<br>• 사용자 인풋에 따른 물품 피킹 및 적재 |
| **담당자** | **방가은** | **정아현** | **방가은** | **정아현** |

---

## AI 활용 내역 (AI Utilization)

Claude Code (VS Code) 활용

* **코드 스켈레톤 설계**: ROS 2 노드 및 패키지의 초기 기본 구조 구축
* **노드 기능 구현**: 핵심 알고리즘 및 데이터 처리 로직 구체화
* **디버깅 및 트러블슈팅**: 시뮬레이션 및 통신 중 발생한 에러 원인 분석 및 해결

## 📚 참고 자료 (References)

* **로봇 및 시뮬레이션 관련**
  * [PAL Robotics TIAGo 공식 문서](https://docs.pal-robotics.com/edge/tiago) — TIAGo 로봇 구조, URDF 및 그리퍼 파라미터
  * [TIAGo 공식 깃허브 레포지토리](https://github.com/pal-robotics/tiago_simulation) — TIAGo Simulation Packages
  * [Gazebo Classic ODE 물리 파라미터 문서](https://gazebosim.org/tutorials?tut=physics_params) — `kp`, `minDepth`, `mu` 등 접촉 및 마찰 설정

* **네비게이션 및 사람 추종**
  * [Nav2 공식 튜토리얼](https://docs.nav2.org/tutorials/docs/navigation2_dynamic_point_following.html) — Dynamic Point Following (Object Following)

* **비전 인식 및 좌표 변환**
  * [OpenCV HSV 필터링 기반 물체 인식](https://swbee.tistory.com/26) — 색상 필터링 기법 기술 블로그
  * [ROS 2 카메라 캘리브레이션 및 TF2 변환 가이드](https://thomasthelliez.com/blog/deep-dive-into-ros-2-camera-calibration-tf2-and-optical-frames/) — Camera Calibration, TF2, Optical Frames 딥다이브 블로그

* **매니퓰레이션 및 프레임워크**
  * [pymoveit2 GitHub 레포지토리](https://github.com/peterdavidfagan/pymoveit2) — MoveIt2 Python Wrapper 관련 문서
  * [MoveIt2 공식 가이드 문서](https://moveit.picknik.ai/humble/index.html) — Cartesian path 및 Planning Scene 설정
  * [ROS2 Humble 공식 가이드 문서](https://docs.ros.org/en/humble/) — TF2, MultiThreadedExecutor, ReentrantCallbackGroup을 활용한 동시성 제어

## 영상, 깃허브 링크 
youtu.be/vJhfu0eixjM

<br>
https://github.com/2026-Intelligent-Robotics
