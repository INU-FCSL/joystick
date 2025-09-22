import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# 메시지 타입 임포트
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand

class FullController(Node):

    def __init__(self):
        super().__init__('full_controller_node')

        # === 파라미터 선언 ===
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('arm_joint_speed', 0.2) # 로봇 팔 관절 속도
        self.declare_parameter('deadzone', 0.1)        # 조이스틱 데드존

        # === 퍼블리셔 ===
        self.base_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # === 액션 클라이언트 (그리퍼용) ===
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        # === 서브스크라이버 ===
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # === 클래스 변수 초기화 ===
        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.current_joint_positions = [0.0] * len(self.arm_joint_names)
        self.last_sent_positions = list(self.current_joint_positions)

        # === 조이스틱 매핑 (Xbox 컨트롤러 기준) ===
        self.axis_map = {
            'base_linear': 1,   # 왼쪽 스틱 상/하
            'base_angular': 0,  # 왼쪽 스틱 좌/우
            'joint1': 3,        # 오른쪽 스틱 좌/우
            'joint2': 4,        # 오른쪽 스틱 상/하
            'joint3': 6,        # D-pad 좌/우
            'joint4': 7,        # D-pad 상/하
        }
        self.button_map = {
            'gripper_open': 5,  # RB
            'gripper_close': 4, # LB
            'reset_home' : 3,  # Y
        }

        self.get_logger().info("🚀 터틀봇 통합 제어 노드가 시작되었습니다.")
        self.get_logger().info("왼쪽 스틱: 베이스 | 오른쪽 스틱/D-pad: 팔 | LB/RB: 그리퍼")


    def joint_state_callback(self, msg):
        # 현재 관절 위치 업데이트
        try:
            for i, name in enumerate(self.arm_joint_names):
                idx = msg.name.index(name)
                self.current_joint_positions[i] = msg.position[idx]
        except ValueError:
            pass # 일부 조인트 정보가 아직 없을 수 있음

    def joy_callback(self, msg):
        # 파라미터 값 가져오기
        max_lin_vel = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        max_ang_vel = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        arm_speed = self.get_parameter('arm_joint_speed').get_parameter_value().double_value
        deadzone = self.get_parameter('deadzone').get_parameter_value().double_value

        # --- 1. 터틀봇 본체 제어 ---
        base_twist_msg = Twist()
        base_twist_msg.linear.x = max_lin_vel * msg.axes[self.axis_map['base_linear']]
        base_twist_msg.angular.z = max_ang_vel * msg.axes[self.axis_map['base_angular']]
        self.base_pub.publish(base_twist_msg)

        if msg.buttons[self.button_map['reset_home']] == 1:
            home_position = [0.0] * len(self.arm_joint_names)
            self.send_trajectory_goal(home_position)
            self.get_logger().info("🦾 초기 자세로 복귀합니다.")
        
        # Y 버튼이 눌리지 않았을 때만 일반 조작 실행
        else:
            # --- 2. 로봇 팔 제어 ---
            target_positions = list(self.current_joint_positions)
            deltas = [0.0] * len(self.arm_joint_names)

            # 오른쪽 스틱 입력 처리
            if abs(msg.axes[self.axis_map['joint1']]) > deadzone:
                deltas[0] = -arm_speed * msg.axes[self.axis_map['joint1']]
            if abs(msg.axes[self.axis_map['joint2']]) > deadzone:
                deltas[1] = arm_speed * msg.axes[self.axis_map['joint2']]

            # D-pad 입력 처리
            if abs(msg.axes[self.axis_map['joint3']]) > deadzone:
                deltas[2] = -arm_speed * msg.axes[self.axis_map['joint3']]
            if abs(msg.axes[self.axis_map['joint4']]) > deadzone:
                deltas[3] = arm_speed * msg.axes[self.axis_map['joint4']]

            # 목표 위치 계산 및 전송
            position_changed = False
            for i in range(len(self.arm_joint_names)):
                if deltas[i] != 0.0:
                    target_positions[i] += deltas[i]
                    position_changed = True

            if position_changed and target_positions != self.last_sent_positions:
                self.send_trajectory_goal(target_positions)
                self.last_sent_positions = list(target_positions)
                self.get_logger().info(f"팔 목표: {[f'{p:.2f}' for p in target_positions]}")


            # --- 3. 그리퍼 제어 ---
            if msg.buttons[self.button_map['gripper_open']] == 1:
                self.send_gripper_goal(0.01) # 열리는 위치 (최대값에 가깝게)
                self.get_logger().info("그리퍼 열기 명령")

            elif msg.buttons[self.button_map['gripper_close']] == 1:
                self.send_gripper_goal(-0.01) # 닫히는 위치 (최소값에 가깝게)
                self.get_logger().info("그리퍼 닫기 명령")

        

    def send_trajectory_goal(self, positions):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = 1 # 1초 안에 도달하도록 설정
        traj_msg.points.append(point)
        self.arm_pub.publish(traj_msg)

    def send_gripper_goal(self, position):
        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('그리퍼 액션 서버를 찾을 수 없습니다.')
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = -1.0 # 힘 제한 없음

        self.gripper_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FullController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드가 종료되었습니다.')
    finally:
        stop_msg = Twist()
        node.base_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()