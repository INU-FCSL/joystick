# ====================
# 편집자 : 박정우
# 최종 수정일  : 2025-09-18
# 작업 상태 : 초안
# 역할 & 발행되는 토픽 : 조이스틱으로 TB3 제어
# ====================


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped
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
        
        # === QOS 설정 ====
        arm_qos = QoSProfile(depth=10)
        arm_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        arm_qos.durability  = QoSDurabilityPolicy.VOLATILE

        # 실제 실행 중인 토픽명 확인 후 교체: /servo_server/delta_twist_cmds 가 일반적
        self.servo_topic = '/servo_node/delta_twist_cmds'  # 필요시 '/servo_node/delta_twist_cmds'
        self.arm_twist_pub = self.create_publisher(TwistStamped, self.servo_topic, arm_qos)

        # === 액션(Action) 클라이언트 선언 ===
        # 3. 그리퍼 제어용
        self.gripper_action_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.get_logger().info("Xbox 통합 제어 노드가 준비되었습니다.")

        self.last_arm_cmd = TwistStamped()
        self.last_arm_cmd.header.frame_id = 'base_link'  # Servo config의 command frame에 맞추세요
        self.deadzone = float(self.get_parameter('deadzone').get_parameter_value().double_value)

        self.arm_pub_rate = 100.0
        self.arm_timer = self.create_timer(1.0 / self.arm_pub_rate, self.publish_arm_cmd)


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


    def apply_deadzone(self, val: float) -> float:
        return 0.0 if abs(val) < self.deadzone else val

    def publish_arm_cmd(self):
        # 타이머에서 최신 명령을 지속 발행
        self.last_arm_cmd.header.stamp = self.get_clock().now().to_msg()
        self.arm_twist_pub.publish(self.last_arm_cmd)

    def joy_callback(self, msg: Joy):
        # (버튼 로그 출력 부분 그대로)

        # 파라미터 가져오기
        max_lin_vel = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        max_ang_vel = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        arm_vel_scale = self.get_parameter('arm_vel_scale').get_parameter_value().double_value

        # --- 1) 베이스 제어 (deadzone 적용) ---
        base_twist_msg = Twist()
        lx = self.apply_deadzone(msg.axes[self.axis_map['base_linear']])
        la = self.apply_deadzone(msg.axes[self.axis_map['base_angular']])
        base_twist_msg.linear.x  = max_lin_vel * lx
        base_twist_msg.angular.z = max_ang_vel * la
        self.base_twist_pub.publish(base_twist_msg)

        # --- 2) 팔 제어 (TwistStamped 생성 대신 상태에 반영) ---
        # 선형 이동
        ax = self.apply_deadzone(msg.axes[self.axis_map['arm_x']])   # D-pad 상/하
        ay = self.apply_deadzone(msg.axes[self.axis_map['arm_y']])   # 오른쪽 스틱 좌/우
        az = self.apply_deadzone(msg.axes[self.axis_map['arm_z']])   # 오른쪽 스틱 상/하

        self.last_arm_cmd.twist.linear.x = arm_vel_scale * ax
        self.last_arm_cmd.twist.linear.y = -arm_vel_scale * ay  # 보통 좌/우 감각 반전
        self.last_arm_cmd.twist.linear.z = arm_vel_scale * az

        # 회전 이동
        aroll = self.apply_deadzone(msg.axes[self.axis_map['arm_roll']])  # D-pad 좌/우
        self.last_arm_cmd.twist.angular.x = arm_vel_scale * aroll

        pitch = 0.0
        if msg.buttons[self.button_map['arm_pitch_up']] == 1:
            pitch = arm_vel_scale
        elif msg.buttons[self.button_map['arm_pitch_down']] == 1:
            pitch = -arm_vel_scale
        self.last_arm_cmd.twist.angular.y = pitch
        # 필요하면 yaw도 버튼/축으로 배정해서 self.last_arm_cmd.twist.angular.z 업데이트

        # --- 3) 그리퍼 제어 (그대로) ---
        if msg.buttons[self.button_map['gripper_open']] == 1:
            self.send_gripper_goal(0.025)
            self.get_logger().info("Gripper Open 명령 전송")
        elif msg.buttons[self.button_map['gripper_close']] == 1:
            self.send_gripper_goal(-0.015)
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
