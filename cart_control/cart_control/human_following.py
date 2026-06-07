#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String  # 다음 노드 가동을 위한 이벤트 토픽 타입 임포트
import math

class UltraDensityFollower(Node):
    def __init__(self):
        super().__init__('ultra_density_follower')
        
        # 라이다 데이터 수신 및 속도 명령 발행을 위한 퍼블리셔/서브스크라이버 설정
        self.scan_sub = self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # [구조 분리] 정차 완료 후 도킹 노드를 깨우기 위한 시퀀스 퍼블리셔 선언
        self.sequence_pub = self.create_publisher(String, '/sequence_control', 10)
        
        self.get_logger().info('안전 거리 확보형 사람 추종 전용 모듈 가동')

        self.feedback_pub = self.create_publisher(String, '/robot_state_feedback', 10)

    def scan_callback(self, msg):
        twist = Twist()
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        # 전방 사람 탐색 범위 설정 (라디안)
        search_limit_rad = math.radians(40.0)
        valid_points = []
        
        # 라이다 데이터 파싱 및 전방 사람 후보군 추출
        for i, distance in enumerate(msg.ranges):
            if distance < 0.05 or distance > 4.5:
                continue
                
            current_angle = angle_min + (i * angle_increment)
            if abs(current_angle) > search_limit_rad:
                continue
                
            x = distance * math.cos(current_angle)
            y = distance * math.sin(current_angle)
            valid_points.append({'dist': distance, 'angle': current_angle, 'x': x, 'y': y})

        # 거리 기반 점군 군집화 알고리즘 구동
        clusters = []
        if len(valid_points) > 0:
            current_cluster = [valid_points[0]]
            for p in valid_points[1:]:
                prev_p = current_cluster[-1]
                spatial_dist = math.sqrt((p['x'] - prev_p['x'])**2 + (p['y'] - prev_p['y'])**2)
                
                if spatial_dist < 0.15:
                    current_cluster.append(p)
                else:
                    clusters.append(current_cluster)
                    current_cluster = [p]
            clusters.append(current_cluster)

        target_human = None
        min_distance = float('inf')
        
        # 신체 규격 매칭 기반 최종 타깃 선택
        for cluster in clusters:
            num_points = len(cluster)
            if 3 <= num_points <= 28:
                width = math.sqrt((cluster[0]['x'] - cluster[-1]['x'])**2 + (cluster[0]['y'] - cluster[-1]['y'])**2)
                if 0.20 <= width <= 1.10:
                    avg_dist = sum([p['dist'] for p in cluster]) / len(cluster)
                    if avg_dist < min_distance:
                        min_distance = avg_dist
                        target_angle = sum([p['angle'] for p in cluster]) / len(cluster)
                        target_human = {'dist': min_distance, 'angle': target_angle}

        # 제어 플로우 연산 파트
        if target_human is not None:
            d = target_human['dist']
            a = target_human['angle']
            
            # 조향 각도 게인 산출
            twist.angular.z = 1.8 * a
            angle_degree = abs(math.degrees(a))
            speed_factor = max(0.25, 1.0 - (angle_degree / 50.0))

            # [수정 보완 핵심: 진열대 모퉁이 데이터 왜곡 마진을 수용한 정차 임계 권역 최적화]
            # 인접 진열대 평면 결합 노이즈로 인해 계측 간격 수치가 0.99m 권역에 수렴하는 현상을 방어하기 위해 상한선을 1.0m로 리사이징
            if d <= 0.45:  
                twist.linear.x = -0.10  
                self.get_logger().info(f'안전 마진 한계 도달 후진 (거리: {d:.2f}m)', throttle_duration_sec=1.0)
                
            elif 0.45 < d <= 1.0:
                # 90cm 대 영역 상태를 안정적으로 수용하여 모터 브레이크 제동 강제 스위칭
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                
                # 정차 확정과 동시에 후속 정밀 도킹 노드(shelf_docking) 구동 명령 신호 송출
                status_msg = String()
                status_msg.data = "START_SHELF_DOCKING"
                self.sequence_pub.publish(status_msg)
                self.get_logger().info(f'사람 추종 정착 완료 및 도킹 시퀀스 이관 (거리: {d:.2f}m)', throttle_duration_sec=2.0)
                feedback = String()
                feedback.data = "FOLLOWING_STOPPED"
                self.feedback_pub.publish(feedback)
                
            elif d > 1.0:
                # 1.0m 원거리 구간 주행 선형 속도 제어
                base_speed = min(0.50, 0.65 * (d - 0.50))
                twist.linear.x = base_speed * speed_factor
                self.get_logger().info(f'사람 추종 전진 중 (거리: {d:.2f}m)', throttle_duration_sec=1.0)
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = UltraDensityFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()