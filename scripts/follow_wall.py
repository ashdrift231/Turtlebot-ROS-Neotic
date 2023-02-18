"""
The aforementioned node uses data from the LaserScan sensor to construct a wall-following behavior for a mobile robot.
The robot's motion is managed by a Twist message that includes commands for linear and angular velocity in a two-dimensional plane.
Finding the wall, turning left, and following the wall make up the three states of the behavior.

The script subscribes to the /scan topic in order to receive LaserScan messages that contain measurements of the distance between the robot and nearby obstructions.
The robot's proximity to a wall on its left side—which is supposed to be the one that needs to be followed—is calculated using the distances.

The clbk laser callback function processes the LaserScan data and changes the global regions_ variable with the minimum distance values in the robot's five specified surrounding regions (right, front-right, front, front-left, and left).
The next step is to decide what to do based on the current state and the distance measurements by calling the take action function.

The robot travels forward and turns to the right while in the find wall state until it senses a wall to the left.
The robot spins to the left in the turn left state until it is parallel to the wall on its left.
The robot uses a proportional control law in the follow the wall state to advance while keeping a constant distance from the wall to its left.
The take action function controls state transitions by examining the most recent distance readings and selecting the appropriate state in accordance with the preset conditions.
The wall-following behavior can be activated or deactivated using the wall follower switch ROS service, which controls the state machine.
"""

#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False
pub_ = None
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[270:306]), 10),
        'fright': min(min(msg.ranges[307:343]), 10),
        'front':  min(min(msg.ranges[0:21]), min(msg.ranges[344:359])),
        'fleft':  min(min(msg.ranges[22:58]), 10),
        'left':   min(min(msg.ranges[59:95]), 10),
    }
    
    take_action()

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print (state, state_dict_[state])
        state_ = state

def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    d = 1.5
    
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

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

def main():
    global pub_, active_
    
    rospy.init_node('Follow_wall')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        
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
    main()
