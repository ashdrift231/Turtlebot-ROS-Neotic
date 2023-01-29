#! /usr/bin/env python3

from re import T
import rospy
import threading

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import tf.transformations as tftr
from numpy import matrix, cos, arctan2, sqrt, pi, sin, cos
import numpy as np
import matplotlib.pyplot as plt

class Task1:

    def __init__(self):
        self.lock = threading.Lock()
        self.RATE = rospy.get_param('/rate', 50)

        self.dt = 0.0
        self.time_start = 0.0
        self.end = False

        self.pose_init = [0.0, 0.0, 0.0]
        self.flag = True

        self.dist2goal = 0
        self.error_angle = 0
        self.dist2goal_prev = 0
        self.error_angle_prev = 0
        self.countor = 0
        self.do = True

        self.X = []
        self.Y = []

        "Desired values setup"
        # rotation matrix [4x4] from `world` frame to `body`
        self.bTw = tftr.euler_matrix(-np.pi, 0.0, 0.0, 'rxyz')

        # in `world` frame
        self.A = rospy.get_param('/A', 90.0)    # [degrees]
        self.pose_des = rospy.get_param('/pose_des', [0.5, 0.0, 2.0])
             
        # in 'body' frame
        self.pose_des = self.transform_pose(self.pose_des)
        print(self.pose_des.T)
        self.rot_z_des = 0.0

        "ROS stuff"
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=False)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odometry_callback)

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

        if self.flag:
            self.zero_pose = [cur_position.x, cur_position.y, cur_position.z]
            self.flag = False

        '''CALCULATE ERRORS HERE AND DISTANCE TO GOAL'''
        e_x = self.pose_des[0] - cur_position.x
        e_y = self.pose_des[1] - cur_position.y
        self.error_angle = -arctan2(e_y, e_x) + cur_rot_z

        self.dist2goal = sqrt(e_x**2 + e_y**2)
        
        if self.error_angle > np.pi:
            self.error_angle = -arctan2(e_y,e_x) + cur_rot_z - 2*np.pi
        elif self.error_angle < -np.pi:
            self.error_angle = -arctan2(e_y,e_x) + cur_rot_z + 2*np.pi
        else:
            self.error_angle = -arctan2(e_y,e_x) + cur_rot_z

        self.X.append(cur_position.x)
        self.Y.append(cur_position.y)


        # set pid controller
        velocity = Twist()
        self.Kp_Linear = 0.2 * self.dist2goal * cos(self.error_angle)
        self.Kp_Angular = -0.2 * self.error_angle
        self.Kd_Linear = 0.09 * (self.dist2goal*cos(self.error_angle) - self.dist2goal_prev*cos(self.error_angle_prev))/self.dt
        self.Kd_Angular = 0.3 *(self.error_angle - self.error_angle_prev)/self.dt
        
        self.Ki_Linear = 0.02 * self.dist2goal * cos(self.error_angle)*(self.dt)
        self.Ki_Angular = 0.05 * self.error_angle * self.dt
        
        velocity.linear.x =  self.Kp_Linear + self.Kd_Linear +self.Ki_Linear
        velocity.angular.z = self.Kp_Angular + self.Kd_Angular + self.Ki_Angular
        
        self.pub_cmd_vel.publish(velocity)
        
        self.dist2goal_prev = self.dist2goal
        self.error_angle_prev = self.error_angle
        
        print("ERROR", e_x, e_y)
        print("ORIENTATION", self.error_angle)
        
        print(self.countor)
        if self.dist2goal < 0.09 and self.do == True:
            self.do = False
            self.countor += 1
        self.lock.release()
    
    def spin(self):
        rospy.loginfo('Task started!')
        rate = rospy.Rate(self.RATE)

        time_step = 20.0
        self.end = False

        time_prev = 0.0
        self.time_start = rospy.get_time()
        while not rospy.is_shutdown():
            t = rospy.get_time() - self.time_start
            self.dt = t - time_prev
            time_prev = t

            points = [self.transform_pose([3.0, 0.0, 0.0]),
            self.transform_pose([3.0, 3.0, 0.0]),
            self.transform_pose([0.0, 3.0, 0.0]),
            self.transform_pose([0.0, 0.0, 0.0])]
            

            if self.countor == len(points):
                break
            
            self.pose_des = points[self.countor]
            self.do = True
            '''if t > 5 or t < 40:
                self.pose_des = self.transform_pose([2.0, 0.0, 0.0])
            elif t > 40 and t < 80:
                self.pose_des = self.transform_pose([2.0, 2.0, 0.0])
            elif t > 80 and t < 120:
                self.pose_des = self.transform_pose([0.0, 2.0, 0.0])
            elif t > 120 and t < 160:
                self.pose_des = self.transform_pose([0.0, 0.0, 0.0])
            else:
                break'''

                #self.pose_des = self.transform_pose([0.0, 0.0, 0.0])
            #elif t < 130:
                #self.pose_des= self.transform_pose([1.0, 1.0,0.0])
            #else:
                #self.pose_des= self.transform_pose([0.0, 0.0, 0.0])
                
                
            #self.pose_des = self.transform_pose(1.0. 1.0, 0.00 # change position

            #self.pose_des = self.transform_pose([5.0, 0.0, 0.0])
                
            #print('time: {:3.3f} dt: {:3.3f}\n'.format(t, self.dt))
            rate.sleep()
        plt.scatter(self.X,self.Y)
        plt.show()
        rospy.loginfo('Task completed!')


if __name__ == "__main__":
    rospy.init_node('task1_node')
    task1 = Task1()
    task1.spin()