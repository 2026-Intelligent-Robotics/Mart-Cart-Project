# person_teleop.launch.py
# 사람 모델을 키보드로 조종하는 launch 파일
#
# 설치 (없는 경우):
#   sudo apt install ros-humble-teleop-twist-keyboard
#
# 복사 위치:
#   ~/tiago_ws/src/Mart-Cart-Project/tiago_simulation/tiago_gazebo/launch/
#
# 실행:
#   ros2 launch tiago_gazebo person_teleop.launch.py
#
# 키 조작 (터미널 포커스 유지 필요):
#   i: 전진    ,: 후진
#   j: 좌회전  l: 우회전
#   k: 정지

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    person_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='person_teleop',
        output='screen',
        emulate_tty=True,      # 키보드 입력 활성화
        remappings=[
            ('cmd_vel', '/person/cmd_vel'),  # 사람 모델 토픽으로 연결
        ],
        parameters=[{
            'speed': 0.5,   # 이동 속도 (m/s)
            'turn': 0.5,    # 회전 속도 (rad/s)
        }]
    )

    return LaunchDescription([person_teleop])
