#! /usr/bin/env python

import rospy
import threading

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import atan2
import tf.transformations as tftr
from numpy import matrix, cos, arctan2, sqrt, pi, sin, cos
import numpy as np

from traj_generator import TrajectoryGenerator

class Task2:

    def __init__(self):
        self.lock = threading.Lock()
        self.RATE = rospy.get_param('/rate', 50)

        self.dt = 0.0
        self.time_start = 0.0
        self.end = False

        self.pose_init = [0.0, 0.0, 0.0]
        self.flag = True

        "Desired values setup"
        # rotation matrix [4x4] from `world` frame to `body`
        self.bTw = tftr.euler_matrix(-np.pi, 0.0, 0.0, 'rxyz')

        # in `world` frame
        self.A = rospy.get_param('/A', 90.0)    # [degrees]
        self.pose_des = rospy.get_param('/pose_des', [0.0, 0.0, 0.0])
             
        # in 'body' frame
        self.pose_des = self.transform_pose(self.pose_des)
        print(self.pose_des.T)

        self.rot_z_des = 0.0

        "ROS stuff"
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=False)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odometry_callback)

        self.traj_gen = TrajectoryGenerator()
        self.point_num = 0.1
        self.is_nice_orientation = False

    def transform_pose(self, pose_w):
        # in 'body' frame
        pose_des = self.bTw * np.matrix([pose_w[0], pose_w[1], pose_w[2], 0.0]).T
        return pose_des[:3]
            
    def odometry_callback(self, msg):
        self.lock.acquire()

        # read current robot state
        cur_position = msg.pose.pose.position
        cur_q = msg.pose.pose.orientation
        cur_rpy = tftr.euler_from_quaternion((cur_q.x, cur_q.y, cur_q.z, cur_q.w))  # roll pitch yaw
        cur_rot_z = cur_rpy[2]
        self.lock.release()

        """Using if-else statement check the orientation of the robot. It is trajectory-related (see traj_generator.py)! 
        Think! I advice you to use a Flag (False-True) here. Check def __init__. 
        Only if the orientation of the robot is 'nice' you can switch to the main controller.
        
        if not self.is_nice_orientation:
            ...

        !!!!!!! HINT!!!!!!!! Don't forget to update your point!
            """

    def spin(self):
        rospy.loginfo('Task started!')
        rate = rospy.Rate(self.RATE)

        time_step = 5.0
        self.end = False

        time_prev = 0.0
        self.time_start = rospy.get_time()
        while not rospy.is_shutdown():
            t = rospy.get_time() - self.time_start
            self.dt = t - time_prev
            time_prev = t

            #print('time: {:3.3f} dt: {:3.3f}\n'.format(t, self.dt))
            rate.sleep()
        rospy.loginfo('Task completed!')


if __name__ == "__main__":
    rospy.init_node('task2_node')
    task1 = Task2()
    task1.spin()