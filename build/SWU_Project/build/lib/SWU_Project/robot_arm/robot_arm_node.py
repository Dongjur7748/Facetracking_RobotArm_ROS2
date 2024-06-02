import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import serial
import time

class RobotArmNode(Node):
    def __init__(self):
        super().__init__('robot_arm_node')
        self.subscription = self.create_subscription(Point, 'face_coordinates', self.control_robot_arm, 10)
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # 시리얼 포트 설정
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2)  # 시리얼 포트가 초기화될 시간을 줌
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
            rclpy.shutdown()
            return

        self.camera_width = self.declare_parameter('camera_width', 640).value
        self.camera_height = self.declare_parameter('camera_height', 480).value
        self.camera_radius = self.declare_parameter('camera_radius', 10).value
        self.goal_tolerance = self.declare_parameter('goal_tolerance', 0.05).value
        self.planning_time = self.declare_parameter('planning_time', 2.0).value
        self.p_gain_yaw = self.declare_parameter('p_gain_yaw', 0.1).value  # 수정: 더 큰 값
        self.p_gain_pitch = self.declare_parameter('p_gain_pitch', 0.1).value  # 수정: 더 큰 값
        self.initial_pose = [float(i) for i in self.declare_parameter('initial_pose', [90, 90, 90, 90, 45, 10]).value]

        self.current_js_state = JointState()
        self.current_js_state.position = self.initial_pose  # 초기 위치 설정
        self.center_x = self.camera_width / 2.0
        self.center_y = self.camera_height / 2.0
        self.bounding_box_updated = False
        self.camera_current_x = 0
        self.camera_current_y = 0

        self.sub_js = self.create_subscription(JointState, 'current_state', self.callback_joint_state_current, 10)
        self.sub_box = self.create_subscription(Point, 'face_coordinates', self.callback_bounding_box, 10)
        
        if self.initial_pose:
            self.set_initial_pose()

    def set_initial_pose(self):
        initial_js = JointState()
        initial_js.position = self.initial_pose
        while rclpy.ok() and not self.goal_reached(self.current_js_state, initial_js, self.goal_tolerance):
            initial_js.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(initial_js)
            rclpy.spin_once(self)
            time.sleep(0.2)

    def callback_joint_state_current(self, msg):
        self.current_js_state = msg

    def callback_bounding_box(self, msg):
        self.camera_current_x = msg.x - self.center_x
        self.camera_current_y = msg.y - self.center_y
        self.box_area = msg.z
        self.bounding_box_updated = True

    def control_robot_arm(self, msg):
        delta_x = self.center_x - self.camera_current_x
        delta_y = self.center_y - self.camera_current_y

        if self.bounding_box_updated and (abs(delta_x) > self.camera_radius or abs(delta_y) > self.camera_radius):
            self.plan_joint_to_delta(delta_x, delta_y)
            self.bounding_box_updated = False

    def plan_joint_to_delta(self, delta_x, delta_y):
        js = JointState()
        js.position = list(self.current_js_state.position)
        js.header.stamp = self.get_clock().now().to_msg()

        index_joint_x = 0
        index_joint_y = 1

        # 디버그 메시지 추가
        print(f"Current joint state positions: {js.position}")
        print(f"Delta X: {delta_x}, Delta Y: {delta_y}")

        if len(js.position) <= index_joint_x or len(js.position) <= index_joint_y:
            print(f"Index out of range: len(js.position)={len(js.position)}, index_joint_x={index_joint_x}, index_joint_y={index_joint_y}")
            return

        # PID gain 조정
        p_x = self.p_gain_yaw * delta_x
        p_y = self.p_gain_pitch * delta_y

        js.position[index_joint_x] -= p_x
        js.position[index_joint_y] += p_y

        # 각도를 정수로 변환하여 전송
        command = ' '.join([str(int(angle)) for angle in js.position]) + '\n'
        print(f"Sending command: {command}")
        self.serial_port.write(command.encode())
        self.serial_port.flush()  # 데이터를 확실히 전송하기 위해 flush 호출

    def goal_reached(self, current, goal, threshold):
        if not current.position:
            return False

        return all(abs(goal.position[i] - current.position[i]) <= threshold for i in range(len(current.position)))

def main(args=None):
    rclpy.init(args=args)
    node = RobotArmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
