#!/usr/bin/env python
"""
Odometry performed over wheels data collected from ev3_driver.
"""

import math
import rospy
import tf
from math import sin, cos, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def vel_from_wheel_distances(left_distance, right_distance):

if __name__ == '__main__':
    # Init node and Odometry publisher
    rospy.init_node('odometry_publisher')
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
    # A tf broadcast publishes transformations between frames
    odom_broadcaster = tf.TransformBroadcaster()
    # Instantiating pose
    x = 0.0
    y = 0.0
    th = 0.0
    # Instantiating velocities
    vx = 0.0
    vy = 0.0
    vth = 0.0
    # Time variables
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    r = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        # compute odometry in a typical way given the velocities of the robot
        dt = (current_time - last_time).to_sec()
        delta_x = (vx * cos(th) - vy * sin(th)) * dt
        delta_y = (vx * sin(th) + vy * cos(th)) * dt
        delta_th = vth * dt

        x += delta_x
        y += delta_y
        th += delta_th

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        odom_pub.publish(odom)

        last_time = current_time
        r.sleep()
