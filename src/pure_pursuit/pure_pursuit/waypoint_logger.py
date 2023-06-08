import rclpy
from rclpy.node import Node
import numpy as np
import atexit
import csv
import tf2_ros
from geometry_msgs.msg import Quaternion
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Odometry

class WaypointLogger(Node):

    def __init__(self):

        super().__init__('waypoint_logger')
        self.home = expanduser('~')
        self.file = open(self.home+'/raceline1.csv', 'w')
        self.pose_sub = self.create_subscription(Odometry, '/pf/pose/odom', self.save_waypoint, 1)
        self.writer = csv.writer(self.file)

    def save_waypoint(self, data):
        euler = self.euler_from_quaternion(data.pose.pose.orientation)
        speed = LA.norm(np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]),2)
        if data.twist.twist.linear.x>0.:
            print(data.twist.twist.linear.x)

        printdata = [data.pose.pose.position.x, data.pose.pose.position.y, euler[2], speed]
        print(printdata)
        self.writer.writerow(printdata)

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    waypoint_logger = WaypointLogger()
    rclpy.spin(waypoint_logger)
    waypoint_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
