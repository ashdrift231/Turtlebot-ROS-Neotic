#! /usr/bin/env python3

#ROS stuff
from re import T
import rospy
import threading

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from math import atan2
import tf.transformations as tftr
from numpy import matrix, cos, arctan2, sqrt, pi, sin, cos
import numpy as np

from ControlRobot import RobotControl

pub_ = None

regions_= {
        'front' : 0,
        'frontr':0,
        'rarer':0,
        'rare' : 0,
        'right': 0,
        'rarel':0,
        'frontl':0,
        'left' : 0,
}

state_ = 0
sate_dict_ = {
    0:'finding wall',
    1:'turn left',
    2:'following the wall'
}

def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    return msg

def follow_the_wall():
    global regions_
    
    msg = Twist()
    msg.linear.x = 0.5
    return msg

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print (state, state_dict_[state])
        state_ = state

def laser_cb(msg):
    global regions_
    regions_={
        'front' : min(min(msg.ranges[0:22]),min(msg.ranges[336:359])),
        'frontr':min(min(msg.ranges[23:68]),10),
        'right': min(min(msg.ranges[69:114]),10),
        'rarer': min(min(msg.ranges[115:160]),10),
        'rare' : min(min(msg.ranges[161:206]),10),
        'rarel': min(min(msg.ranges[207:252]),10),
        'left': min(min(msg.ranges[253:298]),10),
        'frontl': min(min(msg.ranges[299:335]),10),
    }
    
def take_action():
    global regions_
    regions = regions_
    
    
    d = 0.5

    if regions_['front']>d and regions_['frontl'] > d and regions_['frontr']>d and regions_['left']>d and regions_['rare']>d and regions_['rarel']>d and regions_['rarer']>d and regions_['right']>d:
        #no wall
        change_state(0)
    #elif regions_['front'] < d and regions_['frontl']>d and regions_['frontr']>d and regions_['left']>d and regions_['rare']>d and regions_['rarel']>d and regions_['rarer']>d and regions_['right']>d:
        



def spin():
    global pub_

    rospy.init_node('Wall_follow')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub_ = rospy.Subscriber('/scan', LaserScan, laser_cb)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)    
        rate.sleep()

            
if __name__ == '__main__':
    spin()