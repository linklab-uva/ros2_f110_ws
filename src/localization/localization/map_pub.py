import rclpy
from nav2_msgs.srv import LoadMap
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.cli = self.create_client(LoadMap, '/map_server/load_map')
        timer_period = 5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LoadMap.Request()
        self.req.map_url = "/home/deepracer/depend_ws/src/localization/maps/map2.yaml"

        self.future = self.cli.call_async(self.req)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

