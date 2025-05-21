#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg

def publish_static_tf():
    # Initialize the ROS node
    rospy.init_node('camera_init_to_odom_static_tf')

    # Create a TransformBroadcaster
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Create a static transform message
    static_transform = geometry_msgs.msg.TransformStamped()

    # Set the frame names
    static_transform.header.frame_id = "camera_init"
    static_transform.child_frame_id = "odom"

    # Set the translation (x, y, z)
    static_transform.transform.translation.x = -0.2143  # Set the desired x translation   , base_link to autokit_base_link + autokit_base_link to rslidar 
    static_transform.transform.translation.y = 0  # Set the desired y translation
    # static_transform.transform.translation.y = -0.0016881  # Set the desired y translation
    # static_transform.transform.translation.z = -0.494  # Set the desired z translation, autokit base_link - 
    static_transform.transform.translation.z = -0.494  # Set the desired z translation, set odom height to roughly wheel height

    # static_transform.transform.translation.z = -0.5337016  # Set the desired z translation, autokit base_link - 

    # Set the rotation (quaternion: x, y, z, w)
    static_transform.transform.rotation.x = 0.0  # Set the desired rotation x
    static_transform.transform.rotation.y = 0.0  # Set the desired rotation y
    static_transform.transform.rotation.z = 0.0  # Set the desired rotation z
    static_transform.transform.rotation.w = 1.0  # Set the desired rotation w (no rotation)

    # Set the timestamp for the transform
    static_transform.header.stamp = rospy.Time.now()

    # Publish the static transform
    static_broadcaster.sendTransform(static_transform)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_static_tf()
    except rospy.ROSInterruptException:
        pass
