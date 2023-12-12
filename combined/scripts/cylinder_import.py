#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def spawn_cylinder():
    rospy.init_node('spawn_cylinder_node', anonymous=True)
    marker_pub = rospy.Publisher('/collision_object', Marker, queue_size=10)

    cylinder_marker = Marker()
    cylinder_marker.header.frame_id = "base_box"  # Change the frame_id as needed
    cylinder_marker.header.stamp = rospy.Time.now()
    cylinder_marker.ns = "cylinder"
    cylinder_marker.id = 0
    cylinder_marker.type = Marker.CYLINDER
    cylinder_marker.action = Marker.ADD

    # Set the pose of the cylinder (adjust as needed)
    cylinder_marker.pose.position.x = 1.0
    cylinder_marker.pose.position.y = 1.0
    cylinder_marker.pose.position.z = 0.9
    cylinder_marker.pose.orientation.x = 0.0
    cylinder_marker.pose.orientation.y = 0.0
    cylinder_marker.pose.orientation.z = 0.0
    cylinder_marker.pose.orientation.w = 1.0

    # Set the scale of the cylinder (radius and height)
    cylinder_marker.scale.x = 0.022
    cylinder_marker.scale.y = 0.022
    cylinder_marker.scale.z = 0.180

    # Set the color of the cylinder (adjust as needed)
    cylinder_marker.color.a = 1.0  # Alpha (transparency)
    cylinder_marker.color.r = 1.0  # Red
    cylinder_marker.color.g = 0.0  # Green
    cylinder_marker.color.b = 0.0  # Blue

    # Lifetime of the marker (0 indicates forever)
    cylinder_marker.lifetime = rospy.Duration(0)

    # Publish the marker
    marker_pub.publish(cylinder_marker)

    rospy.spin()

if __name__ == '__main__':
    try:
        spawn_cylinder()
    except rospy.ROSInterruptException:
        pass
