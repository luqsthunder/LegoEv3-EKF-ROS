import rospy
import math
from math import sin, cos, pi
import tf
from geometry_msgs.msg import Quaternion,Pose,PoseWithCovarianceStamped


def create_pose_with_covariance_msg(x, y, theta):
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
    period = rospy.Duration(5)
    half_side = 0.5

    msg_array = []
    msg_array.append( create_pose_with_covariance_msg(  half_side,  half_side,  math.pi) )
    msg_array.append( create_pose_with_covariance_msg( -half_side,  half_side,  -math.pi/2 ) )
    msg_array.append( create_pose_with_covariance_msg( -half_side, -half_side,  0 ) )
    msg_array.append( create_pose_with_covariance_msg(  half_side, -half_side,  math.pi/2 ) )

    for (i, msg) in enumerate(msg_array):
        if rospy.is_shutdown():
            break

        rospy.loginfo("Publishing message of index {}".format(i))
        pub.publish(msg)
        rospy.sleep(period)
