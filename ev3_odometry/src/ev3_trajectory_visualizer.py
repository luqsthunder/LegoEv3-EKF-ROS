#!/usr/bin/env python

"""
Source code from: https://answers.ros.org/question/278616/how-to-create-a-publisher-about-trajectory-path-then-show-it-in-rviz/
"""

import rospy
import tf

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

path = Path()

def odom_cb(data):
    global path
    path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path.poses.append(pose)
    path_pub.publish(path)

rospy.init_node('path_node')

odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
path_pub = rospy.Publisher('/odom_trajectory', Path, queue_size=10)

if __name__ == '__main__':
    print('[INFO] Publishing trajectory to /lego_trajectory...')
    rospy.spin()
