#!/usr/bin/env python

import rospy
import tf2_ros
import tf.transformations as tf_trans
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class MapOdomTfPublisher:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.odom_sub = rospy.Subscriber('/kiss/odometry', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        try:
            # Lookup the latest available transform from odom to base_footprint
            # Use the timestamp from the incoming message
            transform = self.tf_buffer.lookup_transform("odom", "base_footprint", msg.header.stamp, rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: %s", e)
            return

        # Extract components from both transforms
        # From the odometry message (map -> base_footprint)
        map_to_base_trans = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        map_to_base_rot = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

        # From the TF lookup (odom -> base_footprint)
        odom_to_base_trans = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ]
        odom_to_base_rot = [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ]

        # Convert both transforms to 4x4 matrices
        mat_map_to_base = tf_trans.concatenate_matrices(
            tf_trans.translation_matrix(map_to_base_trans),
            tf_trans.quaternion_matrix(map_to_base_rot)
        )

        mat_odom_to_base = tf_trans.concatenate_matrices(
            tf_trans.translation_matrix(odom_to_base_trans),
            tf_trans.quaternion_matrix(odom_to_base_rot)
        )

        # Compute inverse of odom_to_base (base_to_odom)
        mat_base_to_odom = tf_trans.inverse_matrix(mat_odom_to_base)

        # Compute map->odom transform: map->base * base->odom
        mat_map_to_odom = tf_trans.concatenate_matrices(mat_map_to_base, mat_base_to_odom)

        # Extract translation and quaternion from the resulting matrix
        t = tf_trans.translation_from_matrix(mat_map_to_odom)
        q = tf_trans.quaternion_from_matrix(mat_map_to_odom)

        # Prepare and send transform
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = msg.header.stamp
        transform_stamped.header.frame_id = "map"
        transform_stamped.child_frame_id = "odom"
        transform_stamped.transform.translation.x = t[0]
        transform_stamped.transform.translation.y = t[1]
        transform_stamped.transform.translation.z = t[2]
        transform_stamped.transform.rotation.x = q[0]
        transform_stamped.transform.rotation.y = q[1]
        transform_stamped.transform.rotation.z = q[2]
        transform_stamped.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(transform_stamped)

if __name__ == '__main__':
    rospy.init_node('map_to_odom_tf_publisher')
    node = MapOdomTfPublisher()
    rospy.spin()