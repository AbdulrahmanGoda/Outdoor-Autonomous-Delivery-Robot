import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from nav_msgs.msg import Odometry
import struct
import math

class PoseToOdomNode(Node):
    def __init__(self):
        super().__init__('pose_to_odom')
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            '/current_pose',
            self.listener_callback,
            10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.diff_cont_odom_publisher = self.create_publisher(Odometry, '/diff_cont/odom', 10)
        self.get_logger().info('Pose to Odometry node initialized')

    def listener_callback(self, msg):
        data = msg.data
        if len(data) != 16:
            self.get_logger().info(f'Invalid data length: {len(data)}. Expected 16.')
            return

        header = data[0]
        if header != 0xAA:
            self.get_logger().info(f'Invalid header: 0x{header:02X}. Expected 0xAA.')
            return

        x_bytes = data[1:5]
        y_bytes = data[5:9]
        theta_bytes = data[9:13]

        checksum_bytes = data[0:13]
        checksum = 0
        for b in checksum_bytes:
            checksum ^= b

        if checksum != data[13]:
            self.get_logger().info(f'Checksum mismatch: computed 0x{checksum:02X}, received 0x{data[13]:02X}')
            return

        if data[14] != 0xEE:
            self.get_logger().info(f'Invalid dummy byte: 0x{data[14]:02X}. Expected 0xEE.')
            return

        if data[15] != 0xEF:
            self.get_logger().info(f'Invalid terminator: 0x{data[15]:02X}. Expected 0xEF.')
            return

        try:
            x = struct.unpack('<f', bytes(x_bytes))[0]
            y = struct.unpack('<f', bytes(y_bytes))[0]
            theta = struct.unpack('<f', bytes(theta_bytes))[0]
        except struct.error as e:
            self.get_logger().info(f'Error unpacking float: {e}')
            return

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion (yaw around Z-axis)
        qz = math.sin(theta / 2)
        qw = math.cos(theta / 2)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        self.odom_publisher.publish(odom_msg)
        self.diff_cont_odom_publisher.publish(odom_msg)
        self.get_logger().info(f'Published odometry: x={x}, y={y}, theta={theta}')

def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdomNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()