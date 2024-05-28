import rclpy
from rclpy.node import Node
import serial

class ArduinoControlNode(Node):
    def __init__(self):
        super().__init__('arduino_control_node')
        self.serial_port = self.declare_parameter('serial_port', '/dev/ttyUSB0').value
        self.baud_rate = self.declare_parameter('baud_rate', 115200).value
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        self.get_logger().info('Arduino control node started.')
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.ser.write(b'Hello Arduino\n')
        response = self.ser.readline().decode('utf-8').strip()
        if response:
            self.get_logger().info(f'Arduino says: {response}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
