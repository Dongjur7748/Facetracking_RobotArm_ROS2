import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import serial

class RobotArmNode(Node):
    def __init__(self):
        super().__init__('robot_arm_node')
        self.subscription = self.create_subscription(Point, 'face_coordinates', self.control_robot_arm, 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # 라즈베리파이에 연결된 아두이노 시리얼 포트
        self.declare_parameters()

    def declare_parameters(self):
        self.camera_width = self.declare_parameter('camera_width', 640).value
        self.camera_height = self.declare_parameter('camera_height', 480).value
        self.camera_radius = self.declare_parameter('camera_radius', 10).value
        self.goal_tolerance = self.declare_parameter('goal_tolerance', 0.05).value
        self.planning_time = self.declare_parameter('planning_time', 2.0).value
        self.p_gain_yaw = self.declare_parameter('p_gain_yaw', 0.001).value
        self.p_gain_pitch = self.declare_parameter('p_gain_pitch', 0.001).value
        self.initial_pose = self.declare_parameter('initial_pose', [0, 40, 180, 170, 0, 73]).value

        self.current_js_state = [0, 40, 180, 170, 0, 73]
        self.center_x = self.camera_width / 2.0
        self.center_y = self.camera_height / 2.0
        self.bounding_box_updated = False
        self.camera_current_x = 0
        self.camera_current_y = 0

        self.sub_js = self.create_subscription(Point, 'face_coordinates', self.callback_bounding_box, 10)
        
    def callback_bounding_box(self, msg):
        self.camera_current_x = msg.x
        self.camera_current_y = msg.y
        self.box_area = msg.z
        self.bounding_box_updated = True

    def control_robot_arm(self, msg):
        delta_x = self.center_x - self.camera_current_x
        delta_y = self.center_y - self.camera_current_y

        if self.bounding_box_updated and (abs(delta_x) > self.camera_radius or abs(delta_y) > self.camera_radius):
            self.plan_joint_to_delta(delta_x, delta_y)
            self.bounding_box_updated = False

    def plan_joint_to_delta(self, delta_x, delta_y):
        index_joint_x = 0
        index_joint_y = 3

        p_x = self.p_gain_yaw * (1 - self.box_area)
        p_y = self.p_gain_pitch * (1 - self.box_area)

        self.current_js_state[index_joint_x] -= delta_x * p_x
        self.current_js_state[index_joint_y] += delta_y * p_y

        command = f"{int(self.current_js_state[0])} {int(self.current_js_state[1])} {int(self.current_js_state[2])} {int(self.current_js_state[3])} {int(self.current_js_state[4])} {int(self.current_js_state[5])}\n"
        self.serial_port.write(command.encode())

def main(args=None):
    rclpy.init(args=args)
    node = RobotArmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
