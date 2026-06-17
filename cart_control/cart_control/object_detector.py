#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker  # RViz 시각화 마커 메시지 타입 임포트
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import sys
import threading
import time

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        self.bridge = CvBridge()  # ROS 이미지 메시지와 OpenCV 이미지 포맷 간 변환 브릿지 객체 생성
        
        self.current_rgb = None   # 현재 프레임의 RGB 이미지 데이터 저장 변수 초기화
        self.current_depth = None # 현재 프레임의 32비트 실수형 깊이 이미지 데이터 저장 변수 초기화
        self._head_2_joint = 0.0  # head_2_joint 현재 위치 (joint_states 콜백에서 업데이트)
        
        # 4종 상품별 HSV 색상 공간 임계치 정의 딕셔너리
        self.color_ranges = {
            'apple': {
                'lower1': np.array([0, 120, 70]), 'upper1': np.array([10, 255, 255]),
                'lower2': np.array([170, 120, 70]), 'upper2': np.array([180, 255, 255])
            },
            'banana': {
                'lower': np.array([20, 100, 100]), 'upper': np.array([35, 255, 255])
            },
            'broccoli': {
                'lower': np.array([35, 60, 40]), 'upper': np.array([85, 255, 255])
            },
            'carrot': {
                'lower': np.array([5, 120, 100]), 'upper': np.array([20, 255, 255])
            }
        }

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.is_running = True  # 노드 제어 루프 실행 상태 플래그 설정
        self.motion_done = False  # 동작 완전히 끝난 후 종료 허용 플래그
        
        # ROS 2 토픽 퍼블리셔 선언
        self.head_pub = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/target_object_pose', 10)
        # RViz 시각화 마커 퍼블리셔 유지
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        
        # ROS 2 토픽 서브스크라이버 선언
        self.rgb_sub = self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/head_front_camera/depth/image_raw', self.depth_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        
        # 사용자 명령 입력을 위한 비동기 백그라운드 스레드 생성 및 구동
        self.input_thread = threading.Thread(target=self.user_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        self.get_logger().info('물체 인식 및 도달 가능 영역 연산 노드 구동 시작')

    def rgb_callback(self, msg):
        # RGB 카메라 이미지 수신 콜백 함수
        try:
            self.current_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'RGB 데이터 수신 에러: {str(e)}')

    def depth_callback(self, msg):
        # 깊이 카메라 이미지 수신 콜백 함수
        try:
            self.current_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f'Depth 데이터 수신 에러: {str(e)}')

    def _joint_state_cb(self, msg: JointState):
        try:
            idx = msg.name.index('head_2_joint')
            self._head_2_joint = msg.position[idx]
        except ValueError:
            pass

    def move_head(self, pan, tilt, duration_sec):
        # 로봇 머리 관절 제어 토픽 발행 함수
        msg = JointTrajectory()
        msg.joint_names = ['head_1_joint', 'head_2_joint']
        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start.sec = duration_sec
        point.time_from_start.nanosec = 0
        msg.points.append(point)
        self.head_pub.publish(msg)

    def detect_all_objects(self, target_item):
        # 색상 마스킹 기법 기반 물체 검출 및 중심점 픽셀 좌표 추출 함수
        if self.current_rgb is None or self.current_depth is None:
            return []

        img_bgr = self.current_rgb.copy()
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        
        if target_item == 'apple':
            ranges = self.color_ranges['apple']
            mask1 = cv2.inRange(img_hsv, ranges['lower1'], ranges['upper1'])
            mask2 = cv2.inRange(img_hsv, ranges['lower2'], ranges['upper2'])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            ranges = self.color_ranges[target_item]
            mask = cv2.inRange(img_hsv, ranges['lower'], ranges['upper'])
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_list = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 150:
                continue
                
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                detected_list.append((cx, cy))
                
        detected_list.sort(key=lambda x: x[0])
        return detected_list

    def publish_rviz_marker(self, x, y, z, item_name, idx):
        # RViz 시각화 목적 마커 생성 및 발행 함수
        pass
        # marker = Marker()
        # marker.header.frame_id = "base_footprint"
        # marker.header.stamp = self.get_clock().now().to_msg()
        # marker.ns = f"{item_name}_targets"
        # marker.id = idx
        # marker.type = Marker.SPHERE
        # marker.action = Marker.ADD
        # marker.pose.position.x = float(z) - 0.15
        # marker.pose.position.y = -float(x) - 0.45
        # marker.pose.position.z = 1.15 - float(y)
        # marker.pose.orientation.w = 1.0
        # marker.scale.x = 0.1
        # marker.scale.y = 0.1
        # marker.scale.z = 0.1
        # marker.lifetime.sec = 0
        # marker.lifetime.nanosec = 0
        # if item_name == 'apple':
        #     marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0
        # elif item_name == 'banana':
        #     marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 1.0
        # elif item_name == 'broccoli':
        #     marker.color.r = 0.0; marker.color.g = 0.5; marker.color.b = 0.0; marker.color.a = 1.0
        # else:
        #     marker.color.r = 1.0; marker.color.g = 0.5; marker.color.b = 0.0; marker.color.a = 1.0
        # self.marker_pub.publish(marker)

    def publish_target_pose(self, cx, cy, item_name, idx):
        # 핀홀 카메라 모델 투영 수식 기반 3차원 공간 좌표 계산 및 결과 송출 함수
        z_meters = self.current_depth[cy, cx]
        
        if np.isnan(z_meters) or np.isinf(z_meters) or z_meters <= 0:
            self.get_logger().warn(f'[{item_name}_{idx}] 깊이 데이터 왜곡으로 데이터 발행 취소')
            return False

        fx, fy = 525.0, 525.0
        cx_cam, cy_cam = 320.0, 240.0
        
        x_meters = (cx - cx_cam) * z_meters / fx
        y_meters = (cy - cy_cam) * z_meters / fy

        self.get_logger().info(f'[{item_name}_{idx}] 추출 좌표 전송 -> X:{x_meters:.2f}m, Y:{y_meters:.2f}m, Z:{z_meters:.2f}m')
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "head_front_camera_color_optical_frame"
        pose_msg.pose.position.x = float(x_meters)
        pose_msg.pose.position.y = float(y_meters)
        pose_msg.pose.position.z = float(z_meters)
        pose_msg.pose.orientation.w = 1.0
        
        self.pose_pub.publish(pose_msg)
        
        # RViz 마커 발행 함수 호출부 주석 처리
        # self.publish_rviz_marker(x_meters, y_meters, z_meters, item_name, idx)
        return True

    def scan_shelf_environment(self):
        self.get_logger().info('자동 진열대 스캔 시퀀스 가동')
        # -0.45 명령: undershoot 여유 확보 → 실제 -0.30 이하 도달 보장
        self.move_head(0.0, -0.45, 5)

        # head_2_joint가 -0.30 이하에 실제로 도달할 때까지 폴링 (최대 20초)
        # 절대값 기준(-0.30): broccoli 가시성 확인된 최소 각도
        self.get_logger().info(f'고개 -0.30rad 이하 도달 대기 중... (현재: {self._head_2_joint:.3f}rad)')
        start_wait = time.time()
        while self._head_2_joint > -0.32:
            if time.time() - start_wait > 20.0:
                self.get_logger().warn(
                    f'고개 도달 타임아웃 (20s 초과). 현재: {self._head_2_joint:.3f}rad → 그냥 진행')
                break
            time.sleep(0.1)
        else:
            self.get_logger().info(
                f'고개 도달 확인 ({time.time() - start_wait:.1f}s). 현재: {self._head_2_joint:.3f}rad')

        # 고개가 이동 중인 상태에서 캡처되는 것을 방지하기 위해 0.5초 대기
        time.sleep(0.5)

        # 고개 도달 완료 후 이전 프레임 제거 → 목표 각도의 새 이미지만 사용
        # (move_head 전에 None으로 초기화하면 고개 이동 중 이미지가 들어와 버리는 타이밍 버그 발생)
        self.current_rgb = None
        self.current_depth = None

        # 새 프레임 도착 대기
        self.get_logger().info('카메라 이미지 최신화 대기 중...')
        start_wait = time.time()
        while (self.current_rgb is None or self.current_depth is None) and (time.time() - start_wait < 3.0):
            time.sleep(0.1)

        self.get_logger().info('이미지 수신 확인됨, 스캔 시작')

        current_inventory = {}
        for item in self.color_ranges.keys():
            found_objects = self.detect_all_objects(item)
            if len(found_objects) > 0:
                current_inventory[item] = found_objects
                
        return current_inventory

    def user_input_loop(self):
        # 사용자 명령 처리 및 그리핑 가능 영역 검증 루프 제어 스레드 함수
        time.sleep(1.0)
        
        # ── 1회만 스캔 ────────────────────────────────────────────────
        inventory = self.scan_shelf_environment()
        
        # 로봇 팔 조작 도달 가능 거리(그리핑 반경 마진) 분석 및 가용 리스트 필터링 연산
        # 티아고 매니퓰레이터의 유효 작업 한계 거리를 고려하여 최소 0.5m ~ 최대 1.0m 이내 상품 판단
        reachable_inventory = {}
        
        for item_name, points in inventory.items():
            valid_points = []
            for pt in points:
                cx, cy = pt
                depth_val = self.current_depth[cy, cx]
                # 수치 유효성 검사 및 하드웨어 물리 반경 조건식 판별
                if not np.isnan(depth_val) and not np.isinf(depth_val):
                    if 0.4 <= depth_val <= 1.0:  # 원본: 0.5 / 가까운 거리 허용
                        valid_points.append(pt)
            
            if len(valid_points) > 0:
                reachable_inventory[item_name] = valid_points
        
        print("\n=====================================================================")
        print("전체 진열대 물체 스캔 결과 (원거리 포함)")
        print("=====================================================================")
        if not inventory:
            print("감지된 상품이 없습니다. 로봇을 진열대 근처로 이동시켜 주세요.")
        else:
            for item_name, points in inventory.items():
                print(f"- {item_name}: 총 {len(points)}개 감지 완료")
        
        print("\n=====================================================================")
        print("피킹 가능 상품 리스트")
        print("=====================================================================")
        if not reachable_inventory:
            print("피킹 반경 내 감지된 상품이 없습니다. 로봇을 진열대 근처로 이동시켜 주세요.")
        else:
            for item_name, points in reachable_inventory.items():
                print(f"- {item_name}: 피킹 가능 수량 {len(points)}개")
        print("=====================================================================")

        # ── 감지 결과에 따라 분기 ─────────────────────────────────────
        if not reachable_inventory:
            print('물체 미감지: 후진 후 사람 추종 모드로 전환합니다')
            self.get_logger().info('후진 시작')

            # 퍼블리셔 등록 안정화 대기
            time.sleep(0.5)

            # 40cm 후진 (0.1m/s * 4s)
            twist = Twist()
            twist.linear.x = -0.10
            end_time = time.time() + 4.0
            while time.time() < end_time:
                self.cmd_pub.publish(twist)
                time.sleep(0.1)

            # 정지
            self.cmd_pub.publish(Twist())
            time.sleep(0.5)

            # 헤드를 정면으로 복귀
            self.move_head(0.0, 0.0, 2)
            time.sleep(2.5)

            print('사람을 찾기 위해 회전합니다.')
            twist_turn = Twist()
            twist_turn.angular.z = 0.3 # 더 천천히 회전 (시야 확보)
            
            # 10초간 회전 (사람을 발견하면 추종 노드가 멈추거나 동작을 제어할 것입니다)
            end_time = time.time() + 10.0 
            while time.time() < end_time:
                self.cmd_pub.publish(twist_turn)
                time.sleep(0.1)

            # 정지
            self.cmd_pub.publish(Twist())
            print('사람 추종 모드로 전환합니다')
            self.motion_done = True
            self.is_running = False
            return

        # 감지된 물체 있음 → 사용자 입력 받기
        user_target = input("어떤 상품을 선택하시겠습니까? (종료: q): ").strip()
        
        if user_target == 'q':
            self.get_logger().info('종료 요청 수신으로 인한 콘솔 루프 해제')
            self.motion_done = True
            self.is_running = False
            return
            
        # 피킹 가용 리스트 상에서만 인풋 데이터를 검증하도록 예외 필터링 구조 설계
        if user_target not in reachable_inventory:
            self.get_logger().warn('피킹 불가능 영역 내 존재 물체이거나 무효한 식별자 입력')
            self.motion_done = True
            self.is_running = False
            return
            
        objects = reachable_inventory[user_target]
        total_found = len(objects)
        
        while True:
            try:
                count_input = input(f"해당 상품을 카트에 몇 개 담으시겠습니까? (1 ~ {total_found}개 가능): ").strip()
                count = int(count_input)
                if 1 <= count <= total_found:
                    break
                else:
                    print(f"오류: 피킹 가능한 개수를 초과했습니다.")
            except ValueError:
                print("오류: 정수 데이터 타입만 인식 가능")
        
        self.get_logger().info(f'[{user_target}] 지정 수량 {count}개 연산 및 MoveIt 시퀀스 토픽 방출')
        
        for i in range(count):
            cx, cy = objects[i]
            success = self.publish_target_pose(cx, cy, user_target, i + 1)
            if success:
                time.sleep(1.0)

        # 사용자 입력까지 완료 → 피킹 모드 전환
        print('피킹 모드로 전환합니다')
        self.get_logger().info('피킹 모드로 전환합니다')
        self.motion_done = True
        self.is_running = False

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    
    # 별도 스레드에서 spin 실행
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.daemon = True
    spin_thread.start()

    # ★ 수정 포인트: input_thread가 완전히 종료될 때까지 대기
    node.input_thread.join()
    
    # 1초 정도 여유를 두어 마지막 발행된 Twist 명령이 모터에 도달하게 함
    time.sleep(1.0) 

    # 동작 완료 후 안전하게 종료
    executor.shutdown()
    spin_thread.join()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()