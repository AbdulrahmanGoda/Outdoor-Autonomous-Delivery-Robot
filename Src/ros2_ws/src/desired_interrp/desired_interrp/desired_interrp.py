import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import struct
import math

class DesiredInterrpNode(Node):
    def __init__(self):
        super().__init__('desired_interrp')
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            '/desired_theta',
            self.listener_callback,
            10)
            
        self.get_logger().info('Node initialized')

    def listener_callback(self, msg):
        data = msg.data
        if len(data) != 8:
            self.get_logger().info(f'Invalid data length: {len(data)}. Expected 16.')
            return

        theta_bytes = data[0:4]
        distance_bytes = data[4:8]

        try:
            theta = struct.unpack('<f', bytes(theta_bytes))[0]
            distance = struct.unpack('<f', bytes(distance_bytes))[0]
            
       
            
        except struct.error as e:
            self.get_logger().info(f'Error unpacking float: {e}')
            return

        self.get_logger().info(f'Desire: theta={theta}, distance={distance}.')

def main(args=None):
    rclpy.init(args=args)
    node = DesiredInterrpNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
