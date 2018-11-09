#!/usr/bin/env python
'''
The control module inputs the estimated variables x_hat, y_hat and theta_hat.
Its objective is to guarantee that the robot follow an intended path using
predefined routines: "GoXYTheta" and "FollowLine".

This module also knows the linear reference and angular velocities.

Then, it outputs the wheels velocity.
'''
import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Pose2D, PoseWithCovarianceStamped

"""
Object of type geometry_msgs/PoseWithCovarianceStamped, received from topic
/ev3/odom_combined. It is the current estimated pose for EV3 brick wrt. origin.
"""
estimated_pose = None

"""
Object of type geometry_msgs/Pose. It is the target pose wrt. origin.
"""
target_pose = None      # Representes the target pose for EV3 brick

def set_target_pose(lin_vel, x_target, y_target, theta_target):
    """
    Updates target_pose for robot.

    @params
    lin_vel : float
        Target velocity in m/s for the travel.
    x_target : float
        Target x position wrt. origin frame.
    y_target : float
        Target y position wrt. origin frame.
    theta_target : float
        Target theta angle wrt. x axis.

    @returns nothing
    """

def update_estimated_pose():
    """
    """

def update_target_pose():
    """
    """

if __name__ == "__main__":
    """
    Initing ROS node and wheels velocity publisher
    """
    rospy.init_node('base_controller')
    control_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(20)

    """
    estimated_pose is as an instance of goemetry_msgs/PoseWithCovarianceStamped
    in order to match robot_pose_ekf output.
    """
    estimated_pose = PoseWithCovarianceStamped()
    estimated_pose.header.frame_id = "world"
    estimated_pose.header.stamp = rospy.Time.now()
    """
    In the other hand, as target_pose does not need a header it can be simply
    an instance of geometry_msgs/Pose2D.
    """
    target_pose = Pose2D()
    target_pose.x = 0.0
    target_pose.y = 0.0
    target_pose.theta = 0.0

    """
    Subscribe to robot_pose_ekf output topic.
    """
    state_sub = rospy.Subscriber("odom_combined",
                                 PoseWithCovarianceStamped, update_pose)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        print current_time

        # check if pose is 0,0
        # GoXYTheta: send the robot somewhere
        # FollowLine: if the robot is close to a wall, follow a line
        rate.sleep()
