#!/usr/bin/env python

"""
Source from:
https://answers.ros.org/question/11135/plotting-a-markerarray-of-spheres-with-rviz/
http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes
"""
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math

def create_marker(x, y, z, sx, sy, sz):
    marker = Marker()
    marker.header.frame_id = "/odom"
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = sx
    marker.scale.y = sy
    marker.scale.z = sz
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    return marker

def start_publishing(rate, l, d):
    topic = 'visualization_marker_array'
    publisher = rospy.Publisher(topic, MarkerArray, queue_size=1)

    rospy.init_node('marker_manager')

    # Generate markers
    markerArray = MarkerArray()
    markerArray.markers.append(create_marker(-l, -d, 0.1,  0.1, 0.4, 0.2))
    markerArray.markers.append(create_marker( l,  d, 0.1,  0.1, 0.4, 0.2))
    markerArray.markers.append(create_marker(-d,  l, 0.1,  0.4, 0.1, 0.2))
    markerArray.markers.append(create_marker( d, -l, 0.1,  0.4, 0.1, 0.2))

    # Set markers ID
    id = 0
    for i in range(0, len(markerArray.markers)):
       markerArray.markers[i].id = id
       id += 1

    # Frequency of publishing
    hz = rospy.Rate(rate)

    # Start publishing
    while not rospy.is_shutdown():
       # Publish the MarkerArray
       publisher.publish(markerArray)
       hz.sleep()

if __name__ == '__main__':
    start_publishing(20, 1, 0.6)
