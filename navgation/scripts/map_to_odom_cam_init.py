#!/usr/bin/env python3

import rospy
import tf2_ros as tf
from geometry_msgs.msg import TransformStamped
import tf.transformations
import numpy as np

class MapToOdomPublisher:
    def __init__(self):
        # Initialize the node
        rospy.init_node('map_to_odom_tf')

        # Create a tf listener and broadcaster
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Create a ROS timer to call the transform computation regularly
        rospy.Timer(rospy.Duration(0.02), self.timer_callback)  # 100 Hz timer

        # Define the static offset for aft_mapped -> base_footprint
        self.aft_mapped_to_base_footprint = np.array([-0.2143, 0.0, -0.494])

    def publish_map_to_odom_tf(self):
        """Compute and broadcast the map -> odom transformation."""
        try:
            # Get the camera_init -> aft_mapped transform
            # camera_init_to_aft_mapped_trans, camera_init_to_aft_mapped_rot = \
            #     self.tf_listener.lookupTransform('map', 'aft_mapped', rospy.Time(0))

            # Get the camera_init -> map transform
            camera_init_to_map_trans, camera_init_to_map_rot = \
                self.tf_listener.lookupTransform('camera_init', 'map', rospy.Time(0))

            # Compute camera_init -> base_footprint
            camera_init_to_base_footprint_trans = camera_init_to_aft_mapped_trans + self.aft_mapped_to_base_footprint
            camera_init_to_base_footprint_trans_mat = tf.transformations.translation_matrix(camera_init_to_base_footprint_trans)
            camera_init_to_base_footprint_rot_mat = tf.transformations.quaternion_matrix(camera_init_to_aft_mapped_rot)
            camera_init_to_base_footprint = tf.transformations.concatenate_matrices(
                camera_init_to_base_footprint_trans_mat, camera_init_to_base_footprint_rot_mat
            )

            # Compute the camera_init -> map transform matrix
            camera_init_to_map_trans_mat = tf.transformations.translation_matrix(camera_init_to_map_trans)
            camera_init_to_map_rot_mat = tf.transformations.quaternion_matrix(camera_init_to_map_rot)
            camera_init_to_map = tf.transformations.concatenate_matrices(
                camera_init_to_map_trans_mat, camera_init_to_map_rot_mat
            )

            # Compute map -> camera_init (inverse of camera_init -> map)
            map_to_camera_init = np.linalg.inv(camera_init_to_map)

            # Compute map -> base_footprint
            map_to_base_footprint = np.dot(map_to_camera_init, camera_init_to_base_footprint)

            # Get odom -> base_footprint from TF
            odom_to_base_trans, odom_to_base_rot = \
                self.tf_listener.lookupTransform('odom', 'base_footprint', rospy.Time(0))
            odom_to_base_trans_mat = tf.transformations.translation_matrix(odom_to_base_trans)
            odom_to_base_rot_mat = tf.transformations.quaternion_matrix(odom_to_base_rot)
            odom_to_base = tf.transformations.concatenate_matrices(
                odom_to_base_trans_mat, odom_to_base_rot_mat
            )

            # Compute map -> odom as map -> base_footprint * (odom -> base_footprint)^-1
            base_to_odom = np.linalg.inv(odom_to_base)
            map_to_odom = np.dot(map_to_base_footprint, base_to_odom)

            # Publish the map -> odom transform
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "map"  # Parent frame
            transform.child_frame_id = "odom"  # Child frame

            # Fill in translation from the matrix
            transform.transform.translation.x = map_to_odom[0, 3]
            transform.transform.translation.y = map_to_odom[1, 3]
            transform.transform.translation.z = map_to_odom[2, 3]

            # Fill in rotation (quaternion) from the matrix
            quat = tf.transformations.quaternion_from_matrix(map_to_odom)
            transform.transform.rotation.x = quat[0]
            transform.transform.rotation.y = quat[1]
            transform.transform.rotation.z = quat[2]
            transform.transform.rotation.w = quat[3]

            # Publish the transform
            self.tf_broadcaster.sendTransformMessage(transform)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"Failed to compute transformations: {str(e)}")

    def timer_callback(self, event):
        """Timer callback to compute and broadcast map -> odom at regular intervals."""
        self.publish_map_to_odom_tf()

if __name__ == '__main__':
    # Instantiate and run the node
    node = MapToOdomPublisher()

    # Use rospy.spin() to keep the node alive and processing callbacks
    rospy.spin()
