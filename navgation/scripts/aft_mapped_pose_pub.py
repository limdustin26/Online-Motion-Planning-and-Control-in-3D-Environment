#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

# Constant delta translation (aft_mapped to base_footprint)
aft_mapped_to_base_footprint = np.array([-0.2143, 0, -0.494])

def odom_callback(msg):
    """
    Callback to handle incoming odometry messages and publish the transformed PoseWithCovarianceStamped.
    """
    # Create a new PoseWithCovarianceStamped message
    new_pose_msg = PoseWithCovarianceStamped()

    # Set the header
    new_pose_msg.header.stamp = rospy.Time.now()
    new_pose_msg.header.frame_id = "map"  # Set the new frame_id

    # Apply the delta to the translation from the original message
    new_pose_msg.pose.pose.position.x = msg.pose.pose.position.x + aft_mapped_to_base_footprint[0]
    new_pose_msg.pose.pose.position.y = msg.pose.pose.position.y + aft_mapped_to_base_footprint[1]
    new_pose_msg.pose.pose.position.z = msg.pose.pose.position.z + aft_mapped_to_base_footprint[2]

    # Set the rotation (no change needed, we use the same rotation)
    new_pose_msg.pose.pose.orientation = msg.pose.pose.orientation

    # Set the covariance (if required, adjust values based on your application)
    new_pose_msg.pose.covariance = msg.pose.covariance

    # Publish the transformed PoseWithCovarianceStamped message
    pose_publisher.publish(new_pose_msg)

if __name__ == "__main__":
    rospy.init_node("aft_mapped_to_base_footprint_pose_publisher")

    # Create a Publisher for the new PoseWithCovarianceStamped message
    pose_publisher = rospy.Publisher("/aloam_pose", PoseWithCovarianceStamped, queue_size=10)

    # Subscribe to the original odometry topic
    rospy.Subscriber("/aft_mapped_to_init", Odometry, odom_callback)

    rospy.loginfo("Publishing PoseWithCovarianceStamped from /aft_mapped_to_init to /odometry/aloam")

    rospy.spin()
