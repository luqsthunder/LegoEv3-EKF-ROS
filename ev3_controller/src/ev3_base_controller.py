#!/usr/bin/env python
'''
The control module inputs the estimated variables x_hat, y_hat and theta_hat.
Its objective is to guarantee that the robot follow an intended path using
predefined routines: "set_target_pose" and "make_a_square".

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
Object of type geometry_msgs/Pose2D. It will be converted from an object of type
geometry_msgs/PoseWithCovarianveStamped, received from topic /ev3/odom_combined.
It is the current estimated pose for EV3 brick wrt. origin.
"""
estimated_pose = None

"""
Object of type geometry_msgs/Pose2D. It is the target pose wrt. origin.
"""
target_pose = None      # Representes the target pose for EV3 brick

"""
Pose array to be targeted cyclically. Must be init depending on desired movement.
"""
pose_array = None

"""
Integer that describes movement state.
"""
pose_index = 0

"""
Object of type geometry_msgs/Twist. These are the velocities  will be sent to
EV3 brick, wrt Lego local frame.
"""
twist_cmd = None

def update_estimated_pose(message):
    """
    Callback for odom_combined topic. Updates estimated_pose data.

    @params
    message : geometry_msgs/PoseWithCovarianveStamped
        Pose calculation for odometry and feature observation combined togheter.
    """
    global estimated_pose

    # Extracting 2D coordinates from 3D pose
    x_hat = message.pose.pose.position.x
    y_hat = message.pose.pose.position.y

    """
    Orientation given by message.pose.pose.orientation has form of a Quaternion.
    We need to convert it to Euler form.
    """
    quat = message.pose.pose.orientation
    # In order to use tf.transformations, we need explicit form of quat
    explicit_quat = [quat.x, quat.y, quat.z, quat.w]
    euler = tf.transformations.euler_from_quaternion(explicit_quat)
    theta_hat = euler[2]  # theta is the Yaw angle

    # Finally, store updated estimation for 2D pose
    estimated_pose.x = x_hat
    estimated_pose.y = y_hat
    estimated_pose.theta = theta_hat

def create_pose2d(x, y, theta):
    """
    Returns a new geometry_msgs/Pose2D object.

    @params
    x : float
        x position in m.
    y : float
        y position in m.
    theta : float
        z orientation in rad.

    @returns
    geometry_msgs/Pose2D
        Pose with given parameters.
    """
    pose = Pose2D()
    pose.x = x
    pose.y = y
    pose.theta = theta
    return pose

def set_target_pose(new_target_pose):
    """
    Update target_pose with a geometry_msgs/Pose2D object.

    @params
    new_target_pose : geometry_msgs/Pose2D
        New pose to be assumed as target_pose.
    """
    global target_pose
    target_pose = new_target_pose

def update_state(linear_vel=0.05, angular_vel=math.pi/8,
                     d_position=0.01, d_theta=0.01 ):
    """
    Calculates linear and angular velocities given current estimated_pose
    and target_pose. Movement velocities are constant and passed as parameter.
    Results are update to twist_cmd.

    @params
    lin_vel : float
        Linear velocity in m/s for movement.
    ang_vel : float
        Angular velocity in rad/s for movement.
    d_pos : float
        Minimal linear difference in m so target is considered reached.
    d_theta : float
        Minimal angular difference in rad so target is considered reached.

    @returns
    bool
        If target_pose is reached or not
    """
    global estimated_pose, target_pose, twist_cmd
    # Assume that pose is not reached yet
    reached_target = False
    # Calculate differences
    delta_x = target_pose.x - estimated_pose.x
    delta_y = target_pose.y - estimated_pose.y
    delta_theta = target_pose.theta - estimated_pose.theta

    # Case 1: Position is not reached yet...
    if abs(delta_x) > d_position or abs(delta_y) > d_position:
        # Calculate angle between current and target position
        delta_alpha = math.atan2(delta_x, delta_y)
        twist_cmd.linear.x = linear_vel
        # Angular direction can be clockwise or counterclockwise
        angular_direction = 0
        if delta_alpha != 0:
            angular_direction = - delta_alpha / abs(delta_alpha)
        # Defining angular velocity
        twist_cmd.angular.z = angular_direction * angular_vel
        rospy.logdebug("[DEBUG] Update stepped into case 1")
    # Case 2: Position is reached, but orientation is not...
    elif abs(delta_theta) > d_theta:
        twist_cmd.linear.x = 0
        angular_direction = 0
        if delta_theta != 0:
            angular_direction = delta_theta / abs(delta_theta)
        twist_cmd.angular.z = angular_direction * angular_vel
        rospy.logdebug("[DEBUG] Update stepped into case 2")
    # Case 3: Position and orientation are reached...
    else:
        twist_cmd.linear.x = 0
        twist_cmd.angular.z = 0
        reached_target = True
        rospy.logdebug("[DEBUG] Update stepped into case 3")

    # Debugging messages
    rospy.logdebug("[DEBUG] Desired pose: {}, {}, {}".format(
        target_pose.x,
        target_pose.y,
        target_pose.theta))
    rospy.logdebug("[DEBUG] Current pose: {}, {}, {}".format(
        estimated_pose.x,
        estimated_pose.y,
        estimated_pose.theta))
    return reached_target

def init_square(side_length = 1.0):
    """
    Set predefined poses for square movement.

    @params
    side_length : float
        Lenght of square in m.
    """
    global pose_array, pose_index
    half_side = side_length / 2.0
    # Fill pose_array with vertices of square
    pose_array = []
    """
    For testing purposes, we have following orientations around Z axis wrt. X:
    0     = [0, 0, 0, 0]
    pi/2  = [0, 0, 0.707, 0.707]
    pi    = [0, 0, 1, 0]
    -pi/2 = [0, 0, -0.707, 0.707]
    """
    pose_array.append( create_pose2d(  half_side,  half_side,  math.pi) )
    pose_array.append( create_pose2d( -half_side,  half_side,  -math.pi/2 ) )
    pose_array.append( create_pose2d( -half_side, -half_side,  0 ) )
    pose_array.append( create_pose2d(  half_side, -half_side,  math.pi/2 ) )
    # Start with first pose as target
    pose_index = 0
    set_target_pose( pose_array[pose_index] )

if __name__ == "__main__":
    # Init ROS node and wheels velocity publisher
    rospy.init_node('base_controller', log_level=rospy.DEBUG)
    control_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(1)   # All system runs at 20 Hz

    # Instatiate poses
    estimated_pose = Pose2D()
    target_pose = Pose2D()
    twist_cmd = Twist()

    # Init square configurations
    init_square()

    # Subscribe to robot_pose_ekf output topic.
    state_sub = rospy.Subscriber("odom_combined",
                                 PoseWithCovarianceStamped,
                                 update_estimated_pose)

    while not rospy.is_shutdown():
        # If pose is reached, target next one
        if update_state():
            pose_index = (pose_index + 1) % len(pose_array)
            set_target_pose( pose_array[pose_index] )

        # Publish control Twist to cmd_vel
        control_pub.publish(twist_cmd)

        # Wait next iteration
        rate.sleep()
