#!/usr/bin/env python

'''
The control module inputs the estimated variables x-hat, y-hat and theta-hat.
Its objective is to guarantee that the robot follow an intended path using
predefined routines: "GoXYTheta" and "FollowLine".

This module also knows the linear reference and angular velocities. 

Then, it outputs the wheel's velocity. 
'''

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist, Pose

rospy.init_node('base_controller')

control_pub = rospy.Publisher("control", Twist, queue_size=50)

curr_pose = []
estimated_pose = []

def update_pose(data):
    curr_pose = data.pose

def update_state(data):
    estimated_pose = data

state_sub = rospy.Subscriber("estimated_state", Point, update_state)

odom_sub = rospy.Subscriber("odom", Odometry, update_pose)

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(60.0)

while not rospy.is_shutdown():
    current_time = rospy.Time.now() 

    print curr_pose
    # check if pose is 0,0
    # GoXYTheta: send the robot somewhere
    # FollowLine: if the robot is close to a wall, follow a line

    last_time = current_time
    r.sleep()