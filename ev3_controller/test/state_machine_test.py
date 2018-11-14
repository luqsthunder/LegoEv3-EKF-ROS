import rospy
import math
from math import sin, cos, pi
import tf
from geometry_msgs.msg import Quaternion, Pose, PoseWithCovarianceStamped

"""
This test routine fakes an odometry, sending poses to /odom_combined topic.
It's purpose is to test ev3_controller state machine responses, simulating a
robot_pose_ekf output.
"""

def create_pose_with_covariance_msg(x, y, theta):
    """
    Create and returns a message of robot_pose_ekf output class.

    @params
    x : float
        x position in m.
    y : float
        y position in m.
    theta : float
        z orientation in rad.

    @returns
    geometry_msgs/PoseWithCovarianceStamped
        Pose with positions and orientation set.
    """
    msg = PoseWithCovarianceStamped()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    # Following line returns a np.array
    quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    # Orientation is a Quaternion object
    msg.pose.pose.orientation.x = quat[0]
    msg.pose.pose.orientation.y = quat[1]
    msg.pose.pose.orientation.z = quat[2]
    msg.pose.pose.orientation.w = quat[3]
    return msg

if __name__ == '__main__':
    pub = rospy.Publisher('/odom_combined', PoseWithCovarianceStamped, queue_size=1)
    rospy.init_node('state_machine_tester')
    # Half side length of square to be performed
    half_side = 0.5
    # Array that will contain poses messages to be faked
    msg_array = []
    msg_array.append( create_pose_with_covariance_msg(  half_side,  half_side,  math.pi) )
    msg_array.append( create_pose_with_covariance_msg( -half_side,  half_side,  -math.pi/2 ) )
    msg_array.append( create_pose_with_covariance_msg( -half_side, -half_side,  0 ) )
    msg_array.append( create_pose_with_covariance_msg(  half_side, -half_side,  math.pi/2 ) )
    # Time in seconds between two messages
    period = rospy.Duration(5)
    # Iterate over pose array
    for (i, msg) in enumerate(msg_array):
        # Check if roscore is still active
        if rospy.is_shutdown():
            break
        # Publish and wait next message time
        rospy.loginfo("Publishing message of index {}".format(i))
        pub.publish(msg)
        rospy.sleep(period)
