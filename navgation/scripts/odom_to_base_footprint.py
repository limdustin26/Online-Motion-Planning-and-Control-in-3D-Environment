# 
#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

def odom_callback(msg):
    """
    Callback to handle incoming odometry messages and broadcast the transform.
    """
    transform = TransformStamped()

    # Set the header
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = msg.header.frame_id  # "odom"
    transform.child_frame_id = msg.child_frame_id    # "base_link"

    # Set the translation
    transform.transform.translation.x = msg.pose.pose.position.x
    transform.transform.translation.y = msg.pose.pose.position.y
    transform.transform.translation.z = msg.pose.pose.position.z

    # Set the rotation
    transform.transform.rotation = msg.pose.pose.orientation

    # Broadcast the transform
    tf_broadcaster.sendTransform(transform)

if __name__ == "__main__":
    rospy.init_node("odom_to_base_link_broadcaster")

    # Create a TF broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Subscribe to the odometry topic
    rospy.Subscriber("/ackermann_steering_controller/odom", Odometry, odom_callback)

    rospy.loginfo("Broadcasting odom -> base_link transform from /ackermann_steering_controller/odom")
    rospy.spin()
