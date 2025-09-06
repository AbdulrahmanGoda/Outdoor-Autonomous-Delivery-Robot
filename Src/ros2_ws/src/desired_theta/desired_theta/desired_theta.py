import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension

class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')
        # Topic name remains 'desired_theta'
        self.publisher_ = self.create_publisher(UInt8MultiArray, 'desired_theta', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 1  # Starting value

    def timer_callback(self):
        msg = UInt8MultiArray()
        
        # Set up layout for a 1D array of eight numbers
        dim = MultiArrayDimension()
        dim.label = "desired_theta"
        dim.size = 8
        dim.stride = 8
        msg.layout.dim.append(dim)
        msg.layout.data_offset = 0

        # Create an array with eight consecutive numbers, each incremented by 1.
        msg.data = [
            self.i,
            (self.i + 1) % 256,
            (self.i + 2) % 256,
            (self.i + 3) % 256,
            (self.i + 4) % 256,
            (self.i + 5) % 256,
            (self.i + 6) % 256,
            (self.i + 7) % 256
        ]
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        
        # Increment by 8 for the next message, wrapping around at 256.
        self.i = (self.i + 8) % 256

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
