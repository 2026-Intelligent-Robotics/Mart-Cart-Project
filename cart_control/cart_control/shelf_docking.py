#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
from collections import deque

class ShelfDockingController(Node):
    def __init__(self):
        super().__init__('shelf_docking_controller')

        self.scan_sub = self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.seq_sub  = self.create_subscription(String, '/sequence_control', self.seq_callback, 10)

        self.is_active = False
        self.is_docking_complete = False

        self.TARGET_DIST = 0.07   # 목표 거리 (m)
        self.DIST_TOL    = 0.005   # 거리 허용 오차 (m)
        self.ANGLE_TOL   = 0.03  # 각도 허용 오차

        # 거리가 이 값 이하면 전진 없이 제자리 회전만 허용
        self.SAFE_DIST_THRESHOLD = 0.07  # m

        self.dist_buffer = deque(maxlen=5)

        self.feedback_pub = self.create_publisher(String, '/robot_state_feedback', 10)

    def seq_callback(self, msg):
        if msg.data == "START_SHELF_DOCKING":
            if not self.is_active:
                self.is_active = True
                self.is_docking_complete = False
                self.dist_buffer.clear()
                self.get_logger().info('정밀 진열대 도킹 시퀀스 가동')

    def scan_callback(self, msg):
        if not self.is_active:
            return

        if self.is_docking_complete:
            self.cmd_pub.publish(Twist())
            return

        twist = Twist()
        angle_min      = msg.angle_min
        angle_increment = msg.angle_increment

        front_dists = []
        rear_dists  = []

        for i, distance in enumerate(msg.ranges):
            if distance < 0.02 or distance > 2.0:
                continue

            angle = angle_min + i * angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)

            if y >= -0.02 or not (-0.5 <= x <= 0.5):
                continue

            dist_y = abs(y)
            if x > 0.0:
                front_dists.append(dist_y)
            else:
                rear_dists.append(dist_y)

        if not front_dists and not rear_dists:
            self.get_logger().warn('오른쪽 벽 감지 안 됨', throttle_duration_sec=2.0)
            self.cmd_pub.publish(twist)
            return

        def trimmed_mean(lst):
            lst.sort()
            n = max(1, len(lst) // 5)
            return sum(lst[:n]) / n

        dist_front = trimmed_mean(front_dists) if front_dists else None
        dist_rear  = trimmed_mean(rear_dists)  if rear_dists  else None

        if dist_front is not None and dist_rear is not None:
            raw_dist    = (dist_front + dist_rear) / 2.0
            angle_error = dist_front - dist_rear
        elif dist_front is not None:
            raw_dist, angle_error = dist_front, 0.0
        else:
            raw_dist, angle_error = dist_rear, 0.0

        self.dist_buffer.append(raw_dist)
        smooth_dist = sum(self.dist_buffer) / len(self.dist_buffer)
        dist_error  = smooth_dist - self.TARGET_DIST

        self.get_logger().info(
            f'[도킹] 거리: {smooth_dist:.3f}m  거리오차: {dist_error:+.3f}m  각도오차: {angle_error:+.3f}m',
            throttle_duration_sec=0.3
        )

        # 수렴 판정
        if abs(dist_error) <= self.DIST_TOL and abs(angle_error) <= self.ANGLE_TOL:
            self.is_docking_complete = True
            self.get_logger().info(f'✅ 도킹 완료 (거리: {smooth_dist:.3f}m  각도오차: {angle_error:+.3f}m)')
            feedback = String()
            feedback.data = "DOCKING_SUCCESS"
            self.feedback_pub.publish(feedback)
            self.cmd_pub.publish(Twist())
            return

        # ── 핵심 변경: 거리에 따른 제어 모드 분리 ──────────────────────
        too_close = smooth_dist < self.SAFE_DIST_THRESHOLD

        if too_close:
            # 진열대에 가까운 상태: 전진 금지, 각도만 제자리 회전으로 보정
            # 각도오차가 허용 범위 내면 아무것도 안 함
            if abs(angle_error) <= self.ANGLE_TOL:
                # 각도는 ok, 거리만 문제 → 후진으로 살짝 멀어짐
                twist.linear.x  = -0.03
                twist.angular.z = 0.0
                self.get_logger().info('[모드] 후진으로 거리 확보', throttle_duration_sec=1.0)
            else:
                # 제자리 회전으로 각도만 보정 (전진 없음)
                twist.linear.x  = 0.0
                twist.angular.z = max(-0.3, min(0.3, -2.0 * angle_error))
                self.get_logger().info('[모드] 제자리 각도 보정', throttle_duration_sec=1.0)
        else:
            # 충분한 거리 확보 상태: 전진하며 거리+각도 동시 보정
            steer_dist  = -1.2 * dist_error
            steer_angle = -1.5 * angle_error
            angular     = max(-0.5, min(0.5, steer_dist + steer_angle))

            twist.linear.x  = 0.10
            twist.angular.z = angular

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ShelfDockingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()