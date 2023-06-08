import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from novatel_oem7_msgs.msg import RAWIMU


class IMU_Converter(Node):

    def __init__(self):
        super().__init__('imu_converter_node')
        self.subscription = self.create_subscription(RAWIMU, '/novatel_top/rawimu', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Imu, '/imu', 5)

    def listener_callback(self, msg):
        data = Imu()
        data.header = msg.header
        data.linear_acceleration = msg.linear_acceleration
        data.angular_velocity = msg.angular_velocity
        self.publisher_.publish(data)



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = IMU_Converter()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()