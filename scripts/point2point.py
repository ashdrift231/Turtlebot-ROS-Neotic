"""
The code accomplishes a task for a robot using a Python node.
Controlling the robot's mobility is necessary to get it into the proper pose (position and orientation) in a 3D space.

The script imports a bunch of ROS modules and packages, including geometry msgs.msg, nav msgs.msg, threading, and rospy.
The math and numpy libraries are also utilized.

Several variables and parameters required for the task are initialized by the Task1 class.
It includes a constructor named __init__() that, among other things, initializes a lock and sets the ROS node's rate.

Another method in the class called transform pose() accepts a position from the "world" frame and returns it in the "body" frame.
It does this by changing the pose from one frame to the next using a rotation matrix (self.bTw).

Every time the robot's odometry data is updated, the odometry callback() method is called.
This technique reads the robot's current orientation and position, calculates its errors relative to the ideal posture, and sets the robot's velocity to move in the desired direction.The ROS topic /cmd vel receives the velocity's publication.

The task is launched by calling the spin() method.
The program executes a loop that fixes the desired posture to the value [3.0, 0.0, 0.0]
while it waits for the odometry callback() function to be called and update the robot's position and orientation.
The velocity needed to move the robot into the appropriate position is then calculated and published to /cmd vel.
Until the node is turned off or the task is finished, this loop keeps going.

The script as a whole uses a straightforward control method to get a robot into the required stance.
It computes the error in the robot's location and orientation using the odometry data and bases the robot's velocity on this error.
When the robot assumes the desired stance, the mission is finished.
"""
#! /usr/bin/env python3

# import required libraries
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

# define the Task1 class
class Task1:
    # define the constructor
    def __init__(self):
        # create a lock object to prevent simultaneous access to shared resources
        self.lock = threading.Lock()
        # get the publishing rate from the parameter server
        self.RATE = rospy.get_param('/rate', 50)
        # set the initial delta time and time start values
        self.dt = 0.0
        self.time_start = 0.0
        # set the end flag to False
        self.end = False
        # set the initial position of the robot
        self.pose_init = [0.0, 0.0, 0.0]
        # set the flag variable to True
        self.flag = True

        # set up the desired values
        # create a rotation matrix from 'world' frame to 'body' frame
        self.bTw = tftr.euler_matrix(-np.pi, 0.0, 0.0, 'rxyz')
        # set the desired pose in 'world' frame
        self.A = rospy.get_param('/A', 90.0)
        self.pose_des = rospy.get_param('/pose_des', [0.5, 0.0, 2.0])
        # transform the desired pose to 'body' frame
        self.pose_des = self.transform_pose(self.pose_des)
        print(self.pose_des.T)
        self.rot_z_des = 0.0

        # set up the controllers
        #self.pose_controller = PID([0.1, 0.0, 0.0], 
        #                           [0.0, 0.0, 0.0],
        #                           [0.1, 0.0, 0.0])
        #self.orientation_controller = PID(5.0, 2.0, 0.0)

        # set up ROS publishers and subscribers
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=False)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odometry_callback)

    # define the transform_pose method
    def transform_pose(self, pose_w):
        # transform the pose in 'world' frame to 'body' frame
        pose_des = self.bTw * np.matrix([pose_w[0], pose_w[1], pose_w[2], 0.0]).T
        return pose_des[:3]

    # define the odometry_callback method
    def odometry_callback(self, msg):
        """
        Callback function that is called each time a new odometry message is received.
        """
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
        error_angle = self.pose_des[2] - cur_rot_z

        dist2goal = sqrt(e_x**2 + e_y**2)
        
        if cur_rot_z > np.pi:
            error_angle = -atan2(e_y,e_x) + cur_rot_z - 2*np.pi
            
        else:
            error_angle = -arctan2(e_y,e_x) + cur_rot_z

        # set pd controller
        velocity = Twist()
        velocity.linear.x = 0.35*dist2goal*cos(error_angle)
        velocity.angular.z = 0.29*error_angle
        self.pub_cmd_vel.publish(velocity)
        print("ERROR", e_x, e_y)
        print("ORIENTATION", error_angle)
        self.lock.release()
    
    def spin(self):
        """
        Main loop function that executes the task and controls the robot.
        """
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
            
            self.pose_des = self.transform_pose([3.0, 0.0, 0.0])
                
            #print('time: {:3.3f} dt: {:3.3f}\n'.format(t, self.dt))
            rate.sleep()
        rospy.loginfo('Task completed!')


if __name__ == "__main__":
    rospy.init_node('task1_node')
    task1 = Task1()
    task1.spin()
