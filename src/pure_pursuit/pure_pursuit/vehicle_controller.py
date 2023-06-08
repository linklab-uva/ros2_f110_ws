import rclpy
from rclpy.node import Node
import math
import sys
import tf2_ros
from sensor_msgs.msg import LaserScan
# import tf2_geometry_msgs
import numpy as np

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
import copy


# car name

class VehicleController(Node):

    def __init__(self):

        super().__init__('vehicle_controller')


        self.speedup = 0
        self.speedup_done = 0
        self.csv_vel = False
        self.velocity_from_file = 20
        self.straight_dist = 6
        self.vel_input = 20
        self.car_name = "car_6"
        self.use_ackermann_model = 'false'
        self.adaptive_lookahead = 'false'


        self.ang_goal_x = 0.0
        self.ang_goal_y = 0.0

        self.vel_goal_x = 0.0
        self.vel_goal_y = 0.0

        # steering map constants

        self.ANGLE_TURN_MAX = 100.0
        self.ANGLE_TURN_SWITCH_A = 50.0
        self.ANGLE_TURN_SWITCH_B = 25.0
        self.ANGLE_TURN_SWITCH_C = 12.0
        self.ANGLE_TURN_MIN = 0.0


        # throttle map constants

        # SPEED_TURN_MAX      = 0.50
        # SPEED_TURN_SWITCH_A = 0.65
        # SPEED_TURN_SWITCH_B = 0.75
        # SPEED_TURN_SWITCH_C = 0.85
        # SPEED_TURN_MIN      = 1.0

        self.SPEED_TURN_MAX = 85.0
        self.SPEED_TURN_SWITCH_A = 65.0
        self.SPEED_TURN_SWITCH_B = 75.0
        self.SPEED_TURN_SWITCH_C = 85.0
        self.SPEED_TURN_MIN = 35.0

        # command to steering map constants

        self.ANGLE_RANGE_A = 45.0  # 60.0
        self.ANGLE_RANGE_B = 30.0
        self.ANGLE_RANGE_C = 15.0
        self.ANGLE_RANGE_D = 5.0

        # velocity control

        self.MAX_VEL_GOAL_DIST = 6.0

        # vehicle physical parameters

        self.WHEELBASE_LEN = 0.5

        # adaptive speed control based on lookahead distance

        self.SCALE_VEL_BRAKE = 0.65
        self.SCALE_VEL_CAUTION = 0.80
        self.SCALE_VEL_UNRESTRICTED = 1.00
        self.SCALE_VEL_NO_ADAPTIVE_LOOKAHEAD = 1.00

        # interprocess control string

        self.GOAL_RIGHT = "goal_to_right"
        self.GOAL_LEFT = "goal_to_left"
        self.GOAL_ON_LINE = "goal_on_line"

        # get_logger msg types

        self.MSG_A = "goal at {}m"
        self.MSG_B = "goal at {}m bearing {} {}"
        self.MSG_GOAL = "recieved new goal: ({}, {})"

        # command publisher
        self.pub_goal_arrow = self.create_publisher(Marker, 'goal_data_arrow', 10)
        self.command_pub = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 1)

        # deviation publisher

        # deviation_pub = rclpy.Publisher('/{}/deviation'.format(car_name), Float64, queue_size = 1)

        # get front axle coordinates w.r.t map

            # lookahead speed scaler

        self.lookahead_state = 'caution'

        self.front_axle = Pose()
        self.front_axle.position.x = 0.325
        self.front_axle.orientation.w = 1.0

        self.pose_sub = self.create_subscription(PoseStamped, 'pf/pose', self.vehicle_controller, 1)

        if self.adaptive_lookahead == 'true':
            self.adaptive_lookahead = self.create_subscription(String, 'adaptive_lookahead', self.dist_callback, 1)

        self.ang_sub = self.create_subscription(PoseStamped, 'ppc/ang_goal', self.ang_pose_callback, 1)
        self.vel_sub = self.create_subscription(PoseStamped, 'ppc/vel_goal', self.vel_pose_callback, 1)
    # pure pursuit node



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

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q


    def vehicle_controller(self, data):

        command = AckermannDriveStamped()

        # data = listener.transformPose('car_6_base_link', data)

    #     log_dev = Float64()
        curr_x = data.pose.position.x
        curr_y = data.pose.position.y

        heading = self.euler_from_quaternion(data.pose.orientation)
        heading = heading[2]

        # begin test (include wheel base)

        # begin logging range

    #     log_range = math.sqrt(math.pow(ang_goal_x - curr_x, 2) + math.pow(ang_goal_y - curr_y, 2))

        # end logging range

        if self.use_ackermann_model == 'true':

            front_axle_x = (self.WHEELBASE_LEN * math.cos(heading)) + curr_x
            front_axle_y = (self.WHEELBASE_LEN * math.sin(heading)) + curr_y

            # self.get_logger().info('axle shift: {}'.format(math.sqrt(math.pow(curr_x - front_axle_x, 2) + math.pow(curr_y - front_axle_y, 2))))
            # self.get_logger().info('front axle: {}, {}'.format(round(front_axle_x, 2), round(front_axle_y, 2)))

            curr_x = front_axle_x
            curr_y = front_axle_y

        # end test

        eucl_d = math.sqrt(math.pow(self.ang_goal_x - curr_x, 2) + math.pow(self.ang_goal_y - curr_y, 2))

        # begin curvature test

        # curvature = math.degrees(2.0*(abs(ang_goal_x) - abs(curr_x))/(math.pow(eucl_d, 2)))
        # curvature = math.degrees(2.0*(abs(ang_goal_y) - abs(curr_y))/(math.pow(eucl_d, 2)))
        curvature = math.degrees(2.0*(abs(self.ang_goal_x - curr_x))/(math.pow(eucl_d, 2)))

        # print("Curvature 1: " + str(curvature) + " | Curvature 2: " + str(curvature2))

        steering_angle = math.atan(curvature * self.WHEELBASE_LEN)

        # self.get_logger().info('steering from curvature: {}'.format(math.degrees(steering_angle)))

        # end curvature test

        theta = math.atan2(self.ang_goal_y - curr_y, self.ang_goal_x - curr_x)

        proj_x = eucl_d * math.cos(heading) + curr_x
        proj_y = eucl_d * math.sin(heading) + curr_y

        proj_eucl_shift = math.sqrt(
            math.pow(proj_x - self.ang_goal_x, 2) + math.pow(proj_y - self.ang_goal_y, 2))

        angle_error = math.acos(
            1 - (math.pow(proj_eucl_shift, 2)/(2 * math.pow(eucl_d, 2))))
        angle_error = math.degrees(angle_error)

        goal_sector = (self.ang_goal_x - curr_x)*(proj_y - curr_y) - \
            (self.ang_goal_y - curr_y)*(proj_x - curr_x)

        if goal_sector > 0:
            goal_sector = self.GOAL_RIGHT
        elif goal_sector < 0:
            goal_sector = self.GOAL_LEFT
        else:
            goal_sector = self.GOAL_ON_LINE

        # begin logging deviation

    #     log_deviation = log_range * math.cos(math.radians(angle_error))

    #     if goal_sector < 0:
    #         log_deviation = -1.0 * log_deviation

    #     log_dev.data = log_deviation

    #     deviation_pub.publish(log_dev)

        # end logging deviation

        if goal_sector == self.GOAL_ON_LINE:
            msg = self.MSG_A.format(round(eucl_d, 2))
        else:
            msg = self.MSG_B.format(round(eucl_d, 2), round(
                angle_error, 2), goal_sector)
        # self.get_logger().info(msg)

        '''
        if angle_error > ANGLE_RANGE_A:
            command.drive.steering_angle = ANGLE_TURN_MAX
            command.drive.speed          = SPEED_TURN_MAX
        elif angle_error <= ANGLE_RANGE_A and angle_error > ANGLE_RANGE_B:
            command.drive.steering_angle = ANGLE_TURN_SWITCH_A
            command.drive.speed          = SPEED_TURN_SWITCH_A
        elif angle_error <= ANGLE_RANGE_B and angle_error > ANGLE_RANGE_C:
            command.drive.steering_angle = ANGLE_TURN_SWITCH_B
            command.drive.speed          = SPEED_TURN_SWITCH_B
        elif angle_error <= ANGLE_RANGE_C and angle_error > ANGLE_RANGE_D:
            command.drive.steering_angle = ANGLE_TURN_SWITCH_C
            command.drive.speed          = SPEED_TURN_SWITCH_C
        else:
            command.drive.steering_angle = ANGLE_TURN_MIN
            command.drive.speed          = SPEED_TURN_MIN
        '''

        # full P-control for angle and speed

        if angle_error > self.ANGLE_RANGE_A:
            angle_error = self.ANGLE_RANGE_A

        command.drive.steering_angle = angle_error/self.ANGLE_RANGE_A * 80/57
        # command.drive.speed          = 1.0 - (angle_error/ANGLE_RANGE_A) # * (SPEED_TURN_MIN - SPEED_TURN_MAX)
        # command.drive.speed          = command.drive.speed + SPEED_TURN_MAX

        if goal_sector == self.GOAL_RIGHT:
            command.drive.steering_angle = -1.0 * command.drive.steering_angle

        command.drive.steering_angle = math.degrees(command.drive.steering_angle)

        command.drive.steering_angle = command.drive.steering_angle * 0.0034

        if command.drive.steering_angle > 0.34:
            command.drive.steering_angle = 0.34
        elif command.drive.steering_angle < -0.34:
            command.drive.steering_angle = -0.34

        print("Steering Angle: " + str(command.drive.steering_angle))
        # velocity control node

        vel_eucl_d = math.sqrt(
            math.pow(self.vel_goal_x - curr_x, 2) + math.pow(self.vel_goal_y - curr_y, 2))


        if self.use_ackermann_model == 'true':
            command.drive.speed = (vel_eucl_d/(self.MAX_VEL_GOAL_DIST -
                            self.WHEELBASE_LEN)) * self.SPEED_TURN_MIN
        elif self.csv_vel == True:
            command.drive.speed = self.velocity_from_file
        else:
            command.drive.speed = 0.7
            #print("**SPEED IS " + str(command.drive.speed) + "**")

        # if adaptive_lookahead == 'true':
        #     if lookahead_state == 'brake':
        #         command.drive.speed = SCALE_VEL_BRAKE * command.drive.speed
        #     elif lookahead_state == 'caution':
        #         command.drive.speed = SCALE_VEL_CAUTION * command.drive.speed
        #     else:
        #         command.drive.speed = SCALE_VEL_UNRESTRICTED * command.drive.speed
        # else:
        #     command.drive.speed = SCALE_VEL_NO_ADAPTIVE_LOOKAHEAD * command.drive.speed

        # if command.drive.speed > SPEED_TURN_MAX:
        #     command.drive.speed = SPEED_TURN_MAX
        # if command.drive.speed < SPEED_TURN_MIN:
        #     command.drive.speed = SPEED_TURN_MIN

        # command.drive.speed = 35
        '''
        global speedup
        global speedup_done
        if (ang_goal_y >= 4.2 and speedup != 1):
            speedup = 1
        elif (ang_goal_y <= 0.5 and speedup == 1):
            speedup = 0
            speedup_done = 0
        if speedup == 1 and speedup_done == 0:
            print("**SPEEDUP**")
            speedup_done = 1
            command.drive.speed += 10
        '''

        self.command_pub.publish(command)
        # print(command)

        # publish goal arrow
        goal_arrow = Marker()
        goal_arrow.header = copy.deepcopy(data.header)
        goal_arrow.id = 0
        goal_arrow.type = 0  # defined as arrow
        goal_arrow.action = 0  # add or modify
        goal_arrow.pose.position.x = curr_x
        goal_arrow.pose.position.y = curr_y
        q = self.quaternion_from_euler(0, 0, (heading + math.radians(command.drive.steering_angle * 100)))
        # TODO should apply to orientation of car pose
        goal_arrow.pose.orientation.x = q[0]
        goal_arrow.pose.orientation.y = q[1]
        goal_arrow.pose.orientation.z = q[2]
        goal_arrow.pose.orientation.w = q[3]
        goal_arrow.scale.x = 1.0
        goal_arrow.scale.y = 0.1
        goal_arrow.scale.z = 0.0
        goal_arrow.color.g = 1.0
        goal_arrow.color.a = 1.0

        self.pub_goal_arrow.publish(goal_arrow)

    # relative pose callback


    def ang_pose_callback(self, data):

        self.ang_goal_x = data.pose.position.x
        self.ang_goal_y = data.pose.position.y


    def vel_pose_callback(self, data):

        self.vel_goal_x = data.pose.position.x
        self.vel_goal_y = data.pose.position.y
        self.velocity_from_file = data.pose.orientation.z


    def dist_callback(self, data):

        self.lookahead_state = data.data

def main(args=None):
    rclpy.init(args=args)
    vc = VehicleController()
    rclpy.spin(vc)
    vc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()

