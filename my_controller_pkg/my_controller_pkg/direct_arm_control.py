import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# 현재 관절 상태를 저장할 전역 변수 (간단한 테스트를 위해 사용)
# [joint1, joint2, joint3, joint4]
current_joint_positions = [0.0, 0.0, 0.0, 0.0]

class DirectArmController(Node):

    def __init__(self):
        super().__init__('direct_arm_controller_node')

        # === 퍼블리셔: ros2_control의 컨트롤러에 직접 명령 ===
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory', # 이 토픽 이름은 'ros2 control list_controllers' 결과와 일치해야 함
            10)

        # === 서브스크라이버: 조이스틱 입력 ===
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        # === 서브스크라이버: 현재 로봇 팔 상태 ===
        # 현재 위치를 알아야 다음 위치를 계산할 수 있음
        self.joint_state_sub = self.create_subscription(
            JointState, # 토픽 타입
            '/joint_states', # 토픽 이름
            self.joint_state_callback,
            10)

        self.get_logger().info("MoveIt Servo 우회! ros2_control 직접 제어 노드 시작.")
        self.get_logger().info("RB/LB 버튼으로 Joint2를 움직여보세요.")

        # 컨트롤러 매핑 (LB, RB 버튼만 사용)
        self.button_map = { 'joint2_up': 5, 'joint2_down': 4 }
        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

    def joint_state_callback(self, msg):
        # /joint_states 토픽으로부터 현재 관절 각도를 받아와 저장
        global current_joint_positions
        # msg.name 순서에 맞게 position을 재정렬
        try:
            for i, name in enumerate(self.arm_joint_names):
                idx = msg.name.index(name)
                current_joint_positions[i] = msg.position[idx]
        except ValueError:
            pass # 아직 모든 조인트 정보가 오지 않았을 수 있음

    def joy_callback(self, msg):
        global current_joint_positions
        
        # 목표 위치를 현재 위치로 초기화
        target_positions = list(current_joint_positions)
        
        # RB 버튼: Joint2를 0.1 라디안 증가
        if msg.buttons[self.button_map['joint2_up']] == 1:
            target_positions[1] += 0.1 # joint2에 해당하는 인덱스는 1
            self.get_logger().info(f"Joint2 UP! 목표: {target_positions[1]:.2f}")
            self.send_trajectory_goal(target_positions)
            
        # LB 버튼: Joint2를 0.1 라디안 감소
        elif msg.buttons[self.button_map['joint2_down']] == 1:
            target_positions[1] -= 0.1 # joint2에 해당하는 인덱스는 1
            self.get_logger().info(f"Joint2 DOWN! 목표: {target_positions[1]:.2f}")
            self.send_trajectory_goal(target_positions)

    def send_trajectory_goal(self, positions):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.arm_joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = 1 # 1초 안에 도달

        traj_msg.points.append(point)
        self.arm_pub.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DirectArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()