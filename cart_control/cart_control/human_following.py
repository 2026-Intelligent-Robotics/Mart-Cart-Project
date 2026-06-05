#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class UltraDensityFollower(Node):
    def __init__(self):
        super().__init__('ultra_density_follower')
        
        # 라이다 데이터 및 속도 명령 토픽 설정
        self.scan_sub = self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('사람 추종 노드 시작')

    def scan_callback(self, msg):
        twist = Twist()
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        # 전방 탐색 범위 설정 (라디안 변환)
        search_limit_rad = math.radians(40.0)
        
        # 1차 필터링: 유효 거리 및 시야각 내 포인트 추출
        valid_points = []
        for i, distance in enumerate(msg.ranges):
            if distance < 0.2 or distance > 4.5:
                continue
                
            current_angle = angle_min + (i * angle_increment)
            if abs(current_angle) > search_limit_rad:
                continue
                
            # 로봇 기준 XY 직교좌표계 변환
            x = distance * math.cos(current_angle)
            y = distance * math.sin(current_angle)
            valid_points.append({'dist': distance, 'angle': current_angle, 'x': x, 'y': y})

        # 2차 필터링: 근접 포인트를 하나의 물체 그룹으로 묶는 군집화 기법 적용
        clusters = []
        if len(valid_points) > 0:
            current_cluster = [valid_points[0]]
            for p in valid_points[1:]:
                prev_p = current_cluster[-1]
                spatial_dist = math.sqrt((p['x'] - prev_p['x'])**2 + (p['y'] - prev_p['y'])**2)
                
                # 포인트 간 거리가 15cm 이내일 경우 동일 물체로 판정
                if spatial_dist < 0.15:
                    current_cluster.append(p)
                else:
                    clusters.append(current_cluster)
                    current_cluster = [p]
            clusters.append(current_cluster)

        target_human = None
        min_distance = float('inf')
        
        # 3차 필터링: 생성된 군집 중 사람의 물리적 특징 조건 매칭
        for cluster in clusters:
            num_points = len(cluster)
            # 포인트 밀도 검증 (3개 이상 28개 이하)
            if 3 <= num_points <= 28:
                # 군집 양 끝점 기준 가로 너비 계산
                width = math.sqrt((cluster[0]['x'] - cluster[-1]['x'])**2 + (cluster[0]['y'] - cluster[-1]['y'])**2)
                
                # 바퀴형 사람 베이스의 너비 조건 매칭 (20cm 이상 1.1m 이하)
                if 0.20 <= width <= 1.10:
                    avg_dist = sum([p['dist'] for p in cluster]) / len(cluster)
                    
                    # 조건을 충족하는 물체 중 가장 가까운 타겟 선정
                    if avg_dist < min_distance:
                        min_distance = avg_dist
                        target_angle = sum([p['angle'] for p in cluster]) / len(cluster)
                        target_human = {'dist': min_distance, 'angle': target_angle, 'points': num_points}

        # 로봇 제어 알고리즘 연산
        if target_human is not None:
            d = target_human['dist']
            a = target_human['angle']
            pts = target_human['points']
            
            # 회전 각도 제어 산출
            twist.angular.z = 2.5 * a

            # 조향 연동형 코너링 감속 계수 계산 (회전각이 클수록 선속도 감속 제어)
            angle_degree = abs(math.degrees(a))
            speed_factor = max(0.25, 1.0 - (angle_degree / 50.0))

            # 타겟 거리별 선속도 제어 구간 분기
            
            # 구간 A: 최종 충돌 방지 및 안전거리 확보를 위한 백스텝 제동 구간
            if d <= 0.45:  
                twist.linear.x = -0.15  
                self.get_logger().info(f'코너 충돌 위험! 거리 확보 백스텝 (거리: {d:.2f}m)', throttle_duration_sec=1.0)
            
            # 구간 B: 진열대 앞 정지선 도달 및 목표 간격 유지 구간
            elif 0.45 < d <= 0.65:
                twist.linear.x = 0.0   
                self.get_logger().info(f'안정적 정지 완료 (간격: {d:.2f}m 유지 중)', throttle_duration_sec=1.5)
            
            # 구간 C: 관성 제어 및 급정거 대응을 위한 선제적 감속 대기 구간
            elif 0.65 < d <= 0.95:
                twist.linear.x = 0.0
                self.get_logger().info(f'코너 감속 대기 중', throttle_duration_sec=1.0)
            
            # 구간 D: 1m 이상 거리 발생 시 선형 비례 및 조향 계수를 결합한 추종 구간
            elif d > 1.0:
                base_speed = min(0.55, 0.45 * (d - 1.0))
                twist.linear.x = base_speed * speed_factor
                self.get_logger().info(f'[코너링 제어] 거리: {d:.2f}m, 회전각: {angle_degree:.1f}°, 튜닝 속도: {twist.linear.x:.2f}m/s', throttle_duration_sec=1.0)
            
            # 구간 E: 완충 데드존 제어
            else:
                twist.linear.x = 0.0

        else:
            # 타겟 유실 시 예외 처리 및 정지 명령 부여
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