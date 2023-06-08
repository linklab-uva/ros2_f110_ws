import rclpy
from rclpy.node import Node
import sys
import os
import math
import csv

from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
from geometry_msgs.msg import PoseStamped

class FindNearestPose(Node):


    def __init__(self):
        super().__init__('find_nearest_pose')
        self.declare_parameter('trajectory_file_path')
        self.trajectory = self.get_parameter('trajectory_file_path').value

        self.plan = []

        if not self.plan:
            self.get_logger().info('obtaining trajectory')
            self.construct_path()
        self.odom_sub = self.create_subscription(PoseStamped, 'pf/pose', self.odom_callback, 1)

        self.min_index_pub = self.create_publisher(Int64, 'ppc/index_nearest_point', 1)
        self.min_pose_pub  = self.create_publisher(PoseStamped, 'ppc/visualize_nearest_point', 1)

    def construct_path(self):
        file_path = os.path.expanduser(self.trajectory)

        with open(file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter = ',')
            for waypoint in csv_reader:
                self.plan.append(waypoint)

        for index in range(0, len(self.plan)):
            for point in range(0, len(self.plan[index])):
                self.plan[index][point] = float(self.plan[index][point])

    def odom_callback(self, data):
        min_index      = Int64()
        curr_x         = data.pose.position.x
        curr_y         = data.pose.position.y
        min_index.data = self.find_nearest_point(curr_x, curr_y)
        self.min_index_pub.publish(min_index)

        pose                 = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.plan[min_index.data][0]
        pose.pose.position.y = self.plan[min_index.data][1]
        self.min_pose_pub.publish(pose)

    def find_nearest_point(self, curr_x, curr_y):
        ranges = []
        for index in range(0, len(self.plan)-1):
            eucl_x = math.pow(curr_x - self.plan[index][0], 2)
            eucl_y = math.pow(curr_y - self.plan[index][1], 2)
            eucl_d = math.sqrt(eucl_x + eucl_y)
            ranges.append(eucl_d)
        return(ranges.index(min(ranges)))


def main(args=None):
    rclpy.init(args=args)
    pf = FindNearestPose()
    rclpy.spin(pf)
    pf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
