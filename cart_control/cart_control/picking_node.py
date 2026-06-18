#!/usr/bin/env python3
"""
Step 4 + 5: picking_node
- /target_object_pose 구독 (PoseStamped, head_front_camera_rgb_optical_frame)
- TF2로 base_footprint 변환 후 큐 적재
- 큐 소진까지: 그리퍼 열기 → MoveIt IK 파지 → 그리퍼 닫기 → 바구니 드롭 → 그리퍼 열기 → 홈 복귀
- Step 5: 머리 정면 복귀 → 회전 탐색 → 사람 추종 전환

사전 설치 필요:
  sudo apt install ros-humble-pymoveit2
"""

import threading
import time
from collections import deque

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import tf2_ros
import tf2_geometry_msgs  # PoseStamped 변환 핸들러 등록 (import 필수, 직접 호출 없음)

from geometry_msgs.msg import PoseStamped, Pose, Twist
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

from pymoveit2 import MoveIt2

# ── pymoveit2 spin_once 패치 ─────────────────────────────────────────────────
# pymoveit2의 plan()/wait_until_executed()가 내부적으로 rclpy.spin_once()를 호출함.
# 백그라운드 스레드에서 새 SingleThreadedExecutor 생성 시 AttributeError: __enter__ 발생
# (ROS2 Humble WSL2 환경의 알려진 문제).
# 메인 스레드의 MultiThreadedExecutor가 모든 콜백을 처리하므로,
# spin_once를 sleep으로 대체해도 action 결과가 정상 수신됨.
import rclpy as _rclpy_ref
_rclpy_ref.spin_once = lambda node, executor=None, timeout_sec=0.1: \
    time.sleep(min(timeout_sec if timeout_sec is not None else 0.1, 0.1))

# ── 상수 ────────────────────────────────────────────────────────────────────

ARM_JOINT_NAMES = [
    'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint',
    'arm_5_joint', 'arm_6_joint', 'arm_7_joint',
]
GRIPPER_JOINT_NAMES = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

# TIAGo 주행(tucked) 자세
ARM_HOME_JOINTS = [0.07, -1.34, -0.2, 1.94, -1.57, 1.37, 0.0]

# 바구니 드롭 좌표 (base_footprint 기준)
# 바구니 벽 상단(z=0.25) + 25cm 여유 → z=0.50
# x: 바구니 중심(0.35), y: 정중앙(0.0)
DROP_POSITION = [0.50, 0.0, 0.70]

# ── 파지/드롭 자세 (quaternion xyzw) ──────────────────────────────────────
# gripper_grasping_frame 기준. 시뮬레이션에서 반드시 검증 후 조정할 것.
# arm_tool_link z축이 기본 +z(위)를 향하므로:
#   X축 180° 회전 → z축이 -z(아래)를 향함 = 위에서 내려오는 파지
GRASP_QUAT_XYZW = [1, 0, 0, 0]  # 위에서 잡기 (top-down) — 베이스라인 복귀
DROP_QUAT_XYZW  = [1, 0, 0, 0]  # 파지 자세 유지 (top-down) — 바구니로 내려놓기

# PAL Gripper 프리즈매틱 관절 한계 (Gazebo Classic: lower=0.0, upper=0.045)
GRIPPER_OPEN   = [0.044, 0.044]
# 브로콜리 충돌 박스 6cm cube → 반폭 0.03m (접촉점).
# 목표값 이진탐색: 0.000=오차30mm→튕김, 0.025=오차5mm→파지력 부족
# → 0.015(오차15mm): 충분한 파지력 + 과도한 충격 방지 목표
# Kp=400(시뮬 안정화): 0.015 오차 → 6N/손가락 × μ1.0 = 12N 마찰 >> 중력 0.98N
GRIPPER_CLOSED = [0.015, 0.015]

# 어프로치: 물체 위 오프셋 (m). 위에서 내려오기 전 중간 경유점.
APPROACH_OFFSET_Z = 0.4
# 파지 목표 z: 모든 물체가 world z=0.83 (진열대 상면 0.80 + 반지름 0.03)에 고정
# 카메라 추정 pz는 흔들리므로 x, y만 사용하고 z는 이 고정값으로 대체
GRASP_Z_FIXED = 0.84  # 손가락 z = gz 직접 대응. 0.83=하단 모서리(튕김), 0.85=상단 모서리(뒤틀림), 0.84=정중앙 접촉

# 진열대 충돌 박스 파라미터 (base_footprint 기준, 도킹 완료 후 추정값)
# 높이를 실제(0.8m)보다 크게(1.0m) 설정: OMPL이 z<1.0 구간 우회 경로 선택 방지
# → 어프로치(z=1.23)는 박스 위에 있으므로 OMPL이 반드시 위에서 아래로 접근
SHELF_COLLISION_ID   = 'shelf_body'
SHELF_BOX_CENTER_XYZ = [1.0, 0.0, 0.5]   # z 중심 0.4→0.5 (높이 확장에 맞춰 조정)
SHELF_BOX_SIZE_XYZ   = [0.6, 3.0, 1.0]   # 높이 0.8→1.0m

# 마지막 좌표 수신 후 피킹 시퀀스 시작까지 대기 시간 (s)
# object_detector는 좌표를 1초 간격으로 발행하므로 3초면 충분.
COLLECTION_TIMEOUT_SEC = 3.0

# MoveIt 속도/가속도 스케일링 (0~1)
MAX_VELOCITY    = 0.3
MAX_ACCEL       = 0.1

# Step 5: 사람 탐색 회전 시간 (s) 및 각속도 (rad/s)
ROTATION_TIME    = 10.0
ROTATION_SPEED   = 0.3


# ── 노드 ────────────────────────────────────────────────────────────────────

class PickingNode(Node):

    def __init__(self):
        super().__init__('picking_node')
        self.cbg = ReentrantCallbackGroup()

        # TF2
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 목표 좌표 큐 (최대 2개)
        self.pose_queue: deque[PoseStamped] = deque()
        self._q_lock        = threading.Lock()
        self._collect_timer = None

        # 퍼블리셔
        self.cmd_pub     = self.create_publisher(Twist,          '/cmd_vel',                              10)
        self.head_pub    = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory',    10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.scene_pub   = self.create_publisher(PlanningScene,   '/planning_scene',                      10)

        # /target_object_pose 구독
        self.create_subscription(
            PoseStamped, '/target_object_pose',
            self._on_target_pose, 10,
            callback_group=self.cbg,
        )

        # MoveIt2 (arm 그룹, IK)
        # end_effector_name: gripper_grasping_frame (pal-gripper 실제 파지점)
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ARM_JOINT_NAMES,
            base_link_name='base_footprint',
            end_effector_name='gripper_grasping_frame',
            group_name='arm',
            callback_group=self.cbg,
        )
        self.moveit2.max_velocity_scaling_factor     = MAX_VELOCITY
        self.moveit2.max_acceleration_scaling_factor = MAX_ACCEL
        self.moveit2.planning_time                   = 15.0
        self.moveit2.num_planning_attempts           = 5

        self.get_logger().info('피킹 노드 준비 완료 — /target_object_pose 대기 중')

    # ── 좌표 수신 콜백 ──────────────────────────────────────────────────────

    def _on_target_pose(self, msg: PoseStamped):
        """camera optical frame → base_footprint 변환 후 큐 적재."""
        try:
            msg.header.stamp = rclpy.time.Time().to_msg()  # 최신 TF 사용 (sim/wall 시간 불일치 회피)
            if msg.header.frame_id in ('', 'base_footprint'):
                # 이미 base_footprint 기준이거나 테스트용 직접 발행인 경우 변환 생략
                msg.header.frame_id = 'base_footprint'
                transformed = msg
            else:
                transformed = self.tf_buffer.transform(
                    msg, 'base_footprint',
                    timeout=rclpy.duration.Duration(seconds=1.0),
                )
        except Exception as e:
            self.get_logger().error(f'TF 변환 실패: {e}')
            return

        with self._q_lock:
            self.pose_queue.append(transformed)
            n = len(self.pose_queue)

        p = transformed.pose.position
        self.get_logger().info(
            f'좌표 수신 [{n}번째] (base_footprint): '
            f'x={p.x:.3f}  y={p.y:.3f}  z={p.z:.3f}'
        )

        # 타이머 리셋: 마지막 수신 후 COLLECTION_TIMEOUT_SEC 뒤에 처리 시작
        if self._collect_timer is not None:
            self._collect_timer.cancel()
        self._collect_timer = self.create_timer(
            COLLECTION_TIMEOUT_SEC, self._start_picking,
            callback_group=self.cbg,
        )

    def _start_picking(self):
        self._collect_timer.cancel()
        self._collect_timer = None
        with self._q_lock:
            n = len(self.pose_queue)
        self.get_logger().info(f'좌표 수집 완료 ({n}개) — 피킹 시퀀스 시작')
        # MoveIt subscriber 연결 대기 (shelf collision은 각 pick 직전에 추가)
        time.sleep(1.5)
        threading.Thread(target=self._picking_loop, daemon=True).start()

    # ── MoveIt 플래닝 씬 헬퍼 ───────────────────────────────────────────────

    def _apply_shelf_collision(self, add: bool):
        """진열대 충돌 박스를 플래닝 씬에 추가(add=True) 또는 제거(add=False)."""
        co = CollisionObject()
        co.header = Header()
        co.header.frame_id = 'base_footprint'
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = SHELF_COLLISION_ID

        if add:
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = SHELF_BOX_SIZE_XYZ

            pose = Pose()
            pose.position.x = SHELF_BOX_CENTER_XYZ[0]
            pose.position.y = SHELF_BOX_CENTER_XYZ[1]
            pose.position.z = SHELF_BOX_CENTER_XYZ[2]
            pose.orientation.w = 1.0

            co.primitives = [prim]
            co.primitive_poses = [pose]
            co.operation = CollisionObject.ADD
        else:
            co.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [co]
        self.scene_pub.publish(scene)
        action = '등록' if add else '제거'
        self.get_logger().info(f'플래닝 씬: 진열대 충돌 박스 {action}')

    # ── 그리퍼 / 머리 / 팔 제어 헬퍼 ───────────────────────────────────────

    def _gripper(self, open_: bool, duration_sec: int = 1):
        """그리퍼 JointTrajectory 토픽으로 제어."""
        msg = JointTrajectory()
        msg.joint_names = GRIPPER_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = GRIPPER_OPEN if open_ else GRIPPER_CLOSED
        pt.time_from_start.sec = duration_sec
        msg.points.append(pt)
        self.gripper_pub.publish(msg)
        # RTF≈0.3~0.5 보정: sim-time duration_sec를 실제 대기로 환산 (2.5배 + 여유)
        time.sleep(duration_sec * 2.5 + 0.5)

    def _head(self, pan: float, tilt: float, duration_sec: int = 2):
        """머리 관절 제어."""
        msg = JointTrajectory()
        msg.joint_names = ['head_1_joint', 'head_2_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [pan, tilt]
        pt.time_from_start.sec = duration_sec
        msg.points.append(pt)
        self.head_pub.publish(msg)

    def _arm_home(self):
        """팔을 주행 자세(tucked)로 복귀."""
        self.get_logger().info('팔 → 주행 자세 복귀')
        self.moveit2.move_to_configuration(ARM_HOME_JOINTS)
        self.moveit2.wait_until_executed()

    # ── 피킹 시퀀스 ─────────────────────────────────────────────────────────

    def _pick(self, target: PoseStamped) -> bool:
        """
        물체를 향해 2단계로 접근(어프로치 → 파지)한 뒤 그리퍼 닫기.

        Returns:
            True if 파지 성공, False if 계획/실행 실패
        """
        px = target.pose.position.x
        py = target.pose.position.y
        # pz는 카메라 추정값이라 흔들림 → z는 GRASP_Z_FIXED 고정값 사용

        # 1단계: 어프로치 (물체 APPROACH_OFFSET_Z 위에서 대기)
        approach_z = GRASP_Z_FIXED + APPROACH_OFFSET_Z
        self.get_logger().info(
            f'어프로치: ({px:.3f}, {py:.3f}, {approach_z:.3f})')
        self.moveit2.move_to_pose(
            position=[px, py, approach_z],
            quat_xyzw=GRASP_QUAT_XYZW,
            tolerance_position=0.03,
            tolerance_orientation=0.1,
        )
        if not self.moveit2.wait_until_executed():
            self.get_logger().warn('어프로치 이동 실패')
            return False
        time.sleep(0.3)

        # 2단계: 파지 위치로 직선 하강 (Cartesian path)
        # 진열대 충돌 박스가 물체 좌표를 포함하므로, 하강 중에만 충돌 체크 비활성화
        gz = GRASP_Z_FIXED
        self.get_logger().info(f'파지 하강 (Cartesian): ({px:.3f}, {py:.3f}, {gz:.3f})  [고정 z={GRASP_Z_FIXED}]')
        self.moveit2.cartesian_avoid_collisions = False
        self.moveit2.move_to_pose(
            position=[px, py, gz],
            quat_xyzw=GRASP_QUAT_XYZW,
            cartesian=True,
            cartesian_max_step=0.01,
            tolerance_position=0.02,
            tolerance_orientation=0.1,
        )
        ok = self.moveit2.wait_until_executed()
        self.moveit2.cartesian_avoid_collisions = True
        if not ok:
            self.get_logger().warn('파지 하강 실패')
            return False
        time.sleep(0.3)

        # 그리퍼 닫기 (duration=5: 느리게 닫아 접촉 충격 감소 → 물체 튕김 방지)
        self.get_logger().info('그리퍼 닫기')
        self._gripper(open_=False, duration_sec=5)
        time.sleep(2.0)  # Gazebo 접촉 물리 안정화 대기 (RTF≈0.3~0.5 환경)

        # 진열대 충돌 박스에서 탈출: avoid_collisions=False로 하강했으므로 역방향 상승 필요
        # 이 단계 없으면 팔이 박스 안에 갇혀 드롭/복귀 OMPL 계획이 실패함
        # 속도를 낮춰서 상승: 빠른 가속이 물체를 그리퍼 손목 쪽으로 밀어내는 현상 방지
        lift_z = GRASP_Z_FIXED + APPROACH_OFFSET_Z
        self.get_logger().info(f'파지 후 수직 상승 (저속): z={lift_z:.3f}')
        self.moveit2.max_velocity_scaling_factor     = 0.05
        self.moveit2.max_acceleration_scaling_factor = 0.05
        self.moveit2.cartesian_avoid_collisions = False
        self.moveit2.move_to_pose(
            position=[px, py, lift_z],
            quat_xyzw=GRASP_QUAT_XYZW,
            cartesian=True,
            cartesian_max_step=0.01,
            tolerance_position=0.03,
            tolerance_orientation=0.1,
        )
        self.moveit2.wait_until_executed()
        self.moveit2.cartesian_avoid_collisions = True
        self.moveit2.max_velocity_scaling_factor     = MAX_VELOCITY
        self.moveit2.max_acceleration_scaling_factor = MAX_ACCEL

        return True

    def _drop(self):
        """바구니 위 고정 좌표로 이동 → 그리퍼 열기."""
        self.get_logger().info(
            f'[DROP] 목표: x={DROP_POSITION[0]}, y={DROP_POSITION[1]}, z={DROP_POSITION[2]}')
        self.moveit2.move_to_pose(
            position=DROP_POSITION,
            quat_xyzw=DROP_QUAT_XYZW,
            tolerance_position=0.05,
            tolerance_orientation=0.5,
        )
        ok = self.moveit2.wait_until_executed()
        if not ok:
            self.get_logger().warn('[DROP] Planning 실패 — 현재 위치에서 그리퍼 열기')
        time.sleep(0.3)
        self.get_logger().info('그리퍼 열기 → 물건 낙하')
        self._gripper(open_=True)

    def _picking_loop(self):
        """큐에 쌓인 좌표를 순서대로 처리 후 Step 5 진입."""
        with self._q_lock:
            total = len(self.pose_queue)

        self.get_logger().info(f'═══ 피킹 루프 시작: 총 {total}개 ═══')

        # 베이스 잠금 스레드: 도킹 노드 없는 테스트 환경 안전망
        # 실 데모에선 shelf_docking이 이미 퍼블리시하므로 중복돼도 값이 같아 무해
        self._base_lock_active = True

        def _base_lock_thread():
            while self._base_lock_active:
                self.cmd_pub.publish(Twist())
                time.sleep(0.2)

        base_lock = threading.Thread(target=_base_lock_thread, daemon=True)
        base_lock.start()

        try:
            for idx in range(1, total + 1):
                with self._q_lock:
                    if not self.pose_queue:
                        break
                    target = self.pose_queue.popleft()

                self.get_logger().info(f'[{idx}/{total}] 파지 준비')
                self._gripper(open_=True)  # 파지 전 그리퍼 열기 확인

                # 파지 전: 진열대 충돌 박스 추가 후 planning scene 전파 대기
                # RTF≈0.3~0.5 환경에서 0.5s로는 MoveIt에 전파 전 planning이 시작될 수 있음
                self._apply_shelf_collision(add=True)
                time.sleep(1.5)

                success = self._pick(target)

                # 파지 후 (팔이 LIFT로 진열대 위로 이미 올라온 상태): 충돌 박스 제거
                # 드롭·홈복귀 시 shelf box 우회 없이 OMPL이 직접 경로 탐색 가능
                self._apply_shelf_collision(add=False)
                time.sleep(0.3)

                if success:
                    self._drop()
                else:
                    self.get_logger().warn(f'[{idx}/{total}] 파지 실패 — 그리퍼 열고 다음으로')
                    self._gripper(open_=True)

                # 매 사이클 후 팔 주행 자세 복귀
                self._arm_home()
        finally:
            self._base_lock_active = False
            base_lock.join(timeout=1.0)

        self.get_logger().info('═══ 전체 피킹 완료 → Step 5 실행 ═══')
        self._step5()

    # ── Step 5: 사람 방향 복귀 ──────────────────────────────────────────────

    def _step5(self):
        """
        모든 피킹 완료 후 사람 추종 모드로 전환.
        (object_detector.py 물체 미감지 시 후진+회전 로직과 동일 패턴)
        """
        # 1. 머리 정면 복귀
        self.get_logger().info('Step 5: 머리 정면 복귀')
        self._head(0.0, 0.0, 2)
        time.sleep(2.5)

        # 2. 회전하며 사람 탐색
        self.get_logger().info('Step 5: 사람 방향 회전 시작')
        tw = Twist()
        tw.angular.z = ROTATION_SPEED
        end_t = time.time() + ROTATION_TIME
        while time.time() < end_t:
            self.cmd_pub.publish(tw)
            time.sleep(0.1)
        self.cmd_pub.publish(Twist())

        self.get_logger().info('Step 5 완료 — 사람 추종 모드로 전환')


# ── 엔트리포인트 ─────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PickingNode()

    # MoveIt2 액션 통신과 콜백 동시 처리를 위해 MultiThreadedExecutor 사용
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
