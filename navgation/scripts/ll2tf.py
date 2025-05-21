#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Point
import numpy as np


class LLToTFNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node("ll2tf_node")

        # TF Broadcaster and Listener
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        self.publish_inverted_tf = rospy.get_param("~publish_inverted_tf", False)
        self.use_2d_mode = rospy.get_param("~2d_mode", True)

        # Subscribe to PoseWithCovarianceStamped topic
        self.pose_subscription = rospy.Subscriber(
            "/aloam_pose", PoseWithCovarianceStamped, self.pose_callback, queue_size=10
        )

    def pose_callback(self, msg):
        try:
            # Look up the latest transform from odom to base_link
            (trans, rot) = self.tf_listener.lookupTransform(
                "odom", "base_footprint", rospy.Time(0)
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Could not get odom to base_footprint transform: {e}")
            return

        current_pose = msg.pose.pose.position
        
        # Convert PoseWithCovarianceStamped to Transform
        map_to_base_link = TransformStamped()
        map_to_base_link.header.stamp = rospy.Time.now()
        map_to_base_link.header.frame_id = "map"

        # Set translation and rotation
        map_to_base_link.transform.translation.x = msg.pose.pose.position.x
        map_to_base_link.transform.translation.y = msg.pose.pose.position.y
        map_to_base_link.transform.translation.z = msg.pose.pose.position.z
        map_to_base_link.transform.rotation = msg.pose.pose.orientation

        if self.use_2d_mode:
            map_to_base_link.transform.translation.z = 0.01
            euler = tf.transformations.euler_from_quaternion(
                [
                    map_to_base_link.transform.rotation.x,
                    map_to_base_link.transform.rotation.y,
                    map_to_base_link.transform.rotation.z,
                    map_to_base_link.transform.rotation.w,
                ]
            )
            quat = tf.transformations.quaternion_from_euler(0, 0, euler[2])
            map_to_base_link.transform.rotation.x = quat[0]
            map_to_base_link.transform.rotation.y = quat[1]
            map_to_base_link.transform.rotation.z = quat[2]
            map_to_base_link.transform.rotation.w = quat[3]

        if self.publish_inverted_tf:
            inverted_quat = tf.transformations.quaternion_inverse(
                [
                    map_to_base_link.transform.rotation.x,
                    map_to_base_link.transform.rotation.y,
                    map_to_base_link.transform.rotation.z,
                    map_to_base_link.transform.rotation.w,
                ]
            )
            translation = [
                -map_to_base_link.transform.translation.x,
                -map_to_base_link.transform.translation.y,
                -map_to_base_link.transform.translation.z,
            ]
            inverted_translation = tf.transformations.quaternion_multiply(
                tf.transformations.quaternion_multiply(inverted_quat, translation + [0]),
                tf.transformations.quaternion_conjugate(inverted_quat),
            )[:3]

            self.tf_broadcaster.sendTransform(
                inverted_translation,
                inverted_quat,
                rospy.Time.now(),
                "map",
                "base_footprint",
            )
        else:
            try:
                map_to_odom_trans, map_to_odom_rot = self.chain_transforms(
                    map_to_base_link, (trans, rot)
                )
                self.tf_broadcaster.sendTransform(
                    map_to_odom_trans,
                    map_to_odom_rot,
                    rospy.Time.now(),
                    "odom",
                    "map",
                )
            except Exception as e:
                rospy.logerr(f"Failed to compute map to odom transform: {e}")

    def chain_transforms(self, map_to_base_link, odom_to_base_link):
        base_link_to_odom = tf.transformations.inverse_matrix(
            tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(odom_to_base_link[0]),
                tf.transformations.quaternion_matrix(odom_to_base_link[1]),
            )
        )

        map_to_base_link_matrix = tf.transformations.concatenate_matrices(
            tf.transformations.translation_matrix(
                [
                    map_to_base_link.transform.translation.x,
                    map_to_base_link.transform.translation.y,
                    map_to_base_link.transform.translation.z,
                ]
            ),
            tf.transformations.quaternion_matrix(
                [
                    map_to_base_link.transform.rotation.x,
                    map_to_base_link.transform.rotation.y,
                    map_to_base_link.transform.rotation.z,
                    map_to_base_link.transform.rotation.w,
                ]
            ),
        )

        map_to_odom_matrix = tf.transformations.concatenate_matrices(
            map_to_base_link_matrix, base_link_to_odom
        )

        translation = tf.transformations.translation_from_matrix(map_to_odom_matrix)
        rotation = tf.transformations.quaternion_from_matrix(map_to_odom_matrix)

        return translation, rotation


if __name__ == "__main__":
    try:
        node = LLToTFNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
