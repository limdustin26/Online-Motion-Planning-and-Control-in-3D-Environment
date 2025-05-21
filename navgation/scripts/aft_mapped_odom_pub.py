#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np

# Constant delta translation (aft_mapped to base_footprint)
aft_mapped_to_base_footprint = np.array([-0.2143, 0, -0.494])

def odom_callback(msg):
    """
    Callback to handle incoming odometry messages and publish the transformed odometry.
    """
    # Create a new Odometry message
    new_odom_msg = Odometry()

    # Set the header
    new_odom_msg.header.stamp = rospy.Time.now()
    new_odom_msg.header.frame_id = "camera_init"  # Set the new frame_id
    new_odom_msg.child_frame_id = "base_footprint"  # Set the new child_frame_id

    # Apply the delta to the translation from the original message
    new_odom_msg.pose.pose.position.x = msg.pose.pose.position.x + aft_mapped_to_base_footprint[0]
    new_odom_msg.pose.pose.position.y = msg.pose.pose.position.y + aft_mapped_to_base_footprint[1]
    new_odom_msg.pose.pose.position.z = msg.pose.pose.position.z + aft_mapped_to_base_footprint[2]

    # Set the rotation (no change needed, we use the same rotation)
    new_odom_msg.pose.pose.orientation = msg.pose.pose.orientation

    # Optionally, you can copy over twist information if available
    new_odom_msg.twist = msg.twist

    # Publish the transformed odometry message
    odom_publisher.publish(new_odom_msg)

if __name__ == "__main__":
    rospy.init_node("aft_mapped_to_base_footprint_odom_publisher")

    # Create a Publisher for the new odometry message
    odom_publisher = rospy.Publisher("/odometry/filtered", Odometry, queue_size=10)

    # Subscribe to the original odometry topic
    rospy.Subscriber("/aft_mapped_to_init", Odometry, odom_callback)

    rospy.loginfo("Publishing odometry from /aft_mapped_to_init to /odometry/filtered")

    rospy.spin()