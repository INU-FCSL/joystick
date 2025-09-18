# ====================
# 편집자 : 박정우
# 최종 수정일  : 2025-09-18
# 작업 상태 : 초안
# 역할 & 발행되는 토픽 : 조이스틱으로 TB3 제어
# ====================


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from control_msgs.action import GripperCommand

class XboxFullController(Node):

    def __init__(self):
        super().__init__('xbox_full_controller_node')

        # === 파라미터 선언 ===
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 1.5) # rad/s
        self.declare_parameter('arm_vel_scale', 0.2)     # 로봇 팔 속도 스케일
        self.declare_parameter('deadzone', 0.1) # 데드존 파라미터 추가

        # === 퍼블리셔(Publisher) 선언 ===
        # 1. 차량 주행용
        self.base_twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # 2. 로봇 팔 제어용 (MoveIt Servo)
        self.arm_twist_pub = self.create_publisher(Twist, '/servo_node/delta_twist_cmds', 10)

        # === 액션(Action) 클라이언트 선언 ===
        # 3. 그리퍼 제어용
        self.gripper_action_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')

        # === 서브스크라이버(Subscriber) 선언 ===
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.get_logger().info("Xbox 통합 제어 노드가 준비되었습니다.")

        # === 2단계에서 확인한 컨트롤러 매핑 정보 (이 부분을 자신의 컨트롤러에 맞게 수정하세요!) ===
        self.axis_map = {
            'base_linear': 1,   # 왼쪽 스틱 상/하
            'base_angular': 0,  # 왼쪽 스틱 좌/우
            'arm_y': 3,         # 오른쪽 스틱 좌/우 (- left, + right)
            'arm_z': 4,         # 오른쪽 스틱 상/하 (- down, + up)
            'arm_x': 7,         # D-pad 상/하 (- forward, + backward)
            'arm_roll': 6,      # D-pad 좌/우 (- roll left, + roll right)
        }
        self.button_map = {
            'gripper_open': 0,  # A 버튼
            'gripper_close': 1, # B 버튼
            'arm_pitch_up': 5,  # RB 버튼
            'arm_pitch_down': 4,# LB 버튼
        }
        self.button_name_map = {
            0: 'A', 1: 'B', 2: 'X', 3: 'Y',
            4: 'LB', 5: 'RB',
            6: 'View', 7: 'Menu', 8: 'Xbox',
            9: 'Left Stick Click', 10: 'Right Stick Click'
        }
        
    def joy_callback(self, msg):
        for i, value in enumerate(msg.buttons):
            if value == 1: # 버튼이 눌렸을 때 (값이 1일 때)
                # 이름표(button_name_map)에 있는 이름이면 그 이름을, 없으면 인덱스 번호를 출력
                button_name = self.button_name_map.get(i, f"Unknown Button {i}")
                self.get_logger().info(f"Button Pressed: {button_name} (Index: {i})")
        
        # 파라미터 값 가져오기
        max_lin_vel = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        max_ang_vel = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        arm_vel_scale = self.get_parameter('arm_vel_scale').get_parameter_value().double_value

        # --- 1. 차량 주행 제어 (왼쪽 스틱) ---
        base_twist_msg = Twist()
        base_twist_msg.linear.x = max_lin_vel * msg.axes[self.axis_map['base_linear']]
        base_twist_msg.angular.z = max_ang_vel * msg.axes[self.axis_map['base_angular']]
        self.base_twist_pub.publish(base_twist_msg)

        # --- 2. 로봇 팔 제어 (오른쪽 스틱, D-pad, 어깨 버튼) ---
        arm_twist_msg = Twist()
        # 선형 이동 (X, Y, Z)
        arm_twist_msg.linear.x = arm_vel_scale * msg.axes[self.axis_map['arm_x']]  # D-pad 상/하
        arm_twist_msg.linear.y = -arm_vel_scale * msg.axes[self.axis_map['arm_y']] # 오른쪽 스틱 좌/우 (방향 반전)
        arm_twist_msg.linear.z = arm_vel_scale * msg.axes[self.axis_map['arm_z']]  # 오른쪽 스틱 상/하
        
        # 회전 이동 (Roll, Pitch, Yaw)
        arm_twist_msg.angular.x = arm_vel_scale * msg.axes[self.axis_map['arm_roll']] # D-pad 좌/우
        # Pitch (어깨 버튼으로 제어)
        if msg.buttons[self.button_map['arm_pitch_up']] == 1:
            arm_twist_msg.angular.y = arm_vel_scale
        elif msg.buttons[self.button_map['arm_pitch_down']] == 1:
            arm_twist_msg.angular.y = -arm_vel_scale
            
        self.arm_twist_pub.publish(arm_twist_msg)
        
        # --- 3. 그리퍼 제어 (A, B 버튼) ---
        # A 버튼: 그리퍼 열기
        if msg.buttons[self.button_map['gripper_open']] == 1:
            self.send_gripper_goal(0.025) # 열릴 때의 목표 위치 (단위: 미터)
            self.get_logger().info("Gripper Open 명령 전송")

        # B 버튼: 그리퍼 닫기
        elif msg.buttons[self.button_map['gripper_close']] == 1:
            self.send_gripper_goal(-0.015) # 닫힐 때의 목표 위치
            self.get_logger().info("Gripper Close 명령 전송")

    def send_gripper_goal(self, position):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = -1.0 # 최대 힘 사용

        self.gripper_action_client.wait_for_server()
        self.gripper_action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = XboxFullController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드가 종료되었습니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()