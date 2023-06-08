import rclpy
from rclpy.node import Node
import sys
import os
import math
import csv

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Int64
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker

class FindNearestGoal(Node):

    def __init__(self):
        super().__init__('find_nearest_goal')
        self.declare_parameter('trajectory_file_path')
        self.trajectory = self.get_parameter('trajectory_file_path').value

        self.adaptive_lookahead  = 'false'
        ang_lookahead_dist  = 2.0

        self.plan         = []
        self.frame_id     = 'map'
        self.seq          = 0
        self.ang_goal_pub = self.create_publisher(PoseStamped, 'ppc/ang_goal', 1)
        self.vel_goal_pub = self.create_publisher(PoseStamped, 'ppc/vel_goal', 1)
        self.path_pub = self.create_publisher(MarkerArray, 'ppc/path_pub', 1)
        self.plan_size    = 0
        if not self.plan:
            self.get_logger().info('obtaining trajectory')
            self.construct_path()
        self.nearest_point_sub = self.create_subscription(Int64, 'ppc/index_nearest_point', self.pose_callback, 1)


        # use adaptive lookahead as reported

        self.brake_lookahead        = 1.5
        self.caution_lookahead      = 0.75
        self.unrestricted_lookahead = 2.00

        if self.adaptive_lookahead != 'true':
            self.ang_lookahead_dist = 8
            self.vel_lookahead_dist = ang_lookahead_dist * 2
        else:
            self.ang_lookahead_dist = 100
            self.vel_lookahead_dist = 200


    def construct_path(self):
        file_path = os.path.expanduser(self.trajectory)

        with open(file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter = ',')
            for waypoint in csv_reader:
                self.plan.append(waypoint)

        for index in range(0, len(self.plan)):
            for point in range(0, len(self.plan[index])):
                self.plan[index][point] = float(self.plan[index][point])

        self.plan_size = len(self.plan)

        path_array = MarkerArray()
        path_array.markers = []

        for i in range (self.plan_size):
            Pose_maker = Pose()
            Pose_maker.orientation.x = self.plan[i][0]
            Pose_maker.orientation.y = self.plan[i][1]
            Pose_maker.orientation.z = self.plan[i][2]
            #Pose_maker.pose.orientation.w = self.plan[i][3]
            marker_point = Marker()
            marker_point.header.frame_id = 'map'
            marker_point.ns = "est_pose_" + str(i)
            marker_point.id = 42 + i
            marker_point.type = Marker.SPHERE
            marker_point.action = Marker.ADD
            marker_point.pose = Pose_maker
            marker_point.scale.x, marker_point.scale.y, marker_point.scale.z = .5, .5, .5
            
            path_array.markers.append(marker_point)

        self.path_pub.publish(path_array)


    def dist_callback(self, data):

        if data.data == 'brake':
            self.ang_lookahead_dist = int(self.brake_lookahead * 100)
        elif data.data == 'caution':
            self.ang_lookahead_dist = int(self.caution_lookahead * 100)
        else:
            self.ang_lookahead_dist = int(self.unrestricted_lookahead * 100)

        self.vel_lookahead_dist = self.ang_lookahead_dist * 2

    def pose_callback(self, data):
        # ang_lookahead_dist = int(rclpy.get_param('lookahead_dist'))

        # pose_index = data.data + ang_lookahead_dist

        # if pose_index >= plan_size - 1:
        #     pose_index = pose_index - plan_size - 2

        pose_index = int((data.data + self.ang_lookahead_dist) % self.plan_size)

        if (type(self.plan[pose_index][0]) != float):
            print(type(self.plan[pose_index][0]))
            print(self.plan[pose_index])
        goal                    = PoseStamped()
        goal.header.stamp       = self.get_clock().now().to_msg()
        goal.header.frame_id    = self.frame_id
        goal.pose.position.x    = self.plan[pose_index][0]
        goal.pose.position.y    = self.plan[pose_index][1]
        goal.pose.orientation.z = self.plan[pose_index][2]
        #goal.pose.orientation.w = self.plan[pose_index][3]

        self.ang_goal_pub.publish(goal)

        # pose_index = data.data + vel_lookahead_dist

        # if pose_index > plan_size:
        #     pose_index = pose_index - plan_size

        pose_index = int((data.data + self.vel_lookahead_dist) % self.plan_size)
        # rclpy.loginfo('current index: {}/{}'.format(pose_index, plan_size))

        goal                    = PoseStamped()
        goal.header.stamp       = self.get_clock().now().to_msg()
        goal.header.frame_id    = self.frame_id
        goal.pose.position.x    = self.plan[pose_index][0]
        goal.pose.position.y    = self.plan[pose_index][1]
        goal.pose.orientation.z = self.plan[pose_index][2]
        #goal.pose.orientation.w = self.plan[pose_index][3]

        self.seq = self.seq + 1

        self.vel_goal_pub.publish(goal)

def main(args=None):
    rclpy.init(args=args)
    pf = FindNearestGoal()
    rclpy.spin(pf)
    pf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
