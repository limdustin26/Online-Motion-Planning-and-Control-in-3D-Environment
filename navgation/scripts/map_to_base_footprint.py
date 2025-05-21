#!/usr/bin/env python3

import rospy
import tf2_ros as tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion, Pose, Twist
import tf.transformations
import numpy as np

class OdomToBaseFootprintPublisher:
    def __init__(self):
        # Initialize the node
        rospy.init_node('map_to_base_footprint_tf')

        # Create a tf listener and broadcaster
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.odom_publisher = rospy.Publisher('/odometry/filtered', Odometry, queue_size=10)

        # Create a ROS timer to call the transform computation regularly
        rospy.Timer(rospy.Duration(0.02), self.timer_callback)  # 100 Hz timer

    def publish_odom_to_base_footprint_tf(self):
        """Compute and broadcast the odom -> base_footprint transformation."""
        try:
            # Extract map to base_footprint from base_pose_ground_truth


            # Invert odom to base_footprint to get base_footprint to odom
            camera_init_to_aft_mapped_trans, camera_init_to_aft_mapped_rot = \
                self.tf_listener.lookupTransform('camera_init', 'aft_mapped', rospy.Time(0))
            
            camera_init_to_odom_trans, camera_init_to_odom_rot = \
                self.tf_listener.lookupTransform('camera_init', 'map', rospy.Time(0))
            
            # aft_mapped_to_base_footprint = np.array([-0.2143, -0.0016881, -0.5337016])
            aft_mapped_to_base_footprint = np.array([-0.2143, 0, -0.494])


            camera_init_to_base_footprint_trans = camera_init_to_aft_mapped_trans + aft_mapped_to_base_footprint
            camera_init_to_base_footprint_trans = tf.transformations.translation_matrix(camera_init_to_base_footprint_trans)
            camera_init_to_base_footprint_rot = tf.transformations.quaternion_matrix(camera_init_to_aft_mapped_rot)
            camera_init_to_base_footprint = tf.transformations.concatenate_matrices(camera_init_to_base_footprint_trans,camera_init_to_base_footprint_rot)

            camera_init_to_odom_trans = tf.transformations.translation_matrix(camera_init_to_odom_trans)
            camera_init_to_odom_rot = tf.transformations.quaternion_matrix(camera_init_to_odom_rot)
            camera_init_to_odom = tf.transformations.concatenate_matrices(camera_init_to_odom_trans,camera_init_to_odom_rot)
            odom_to_camera_init = np.linalg.inv(camera_init_to_odom)


            odom_to_base_footprint = tf.transformations.concatenate_matrices(odom_to_camera_init,camera_init_to_base_footprint)
            
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "map"  # Parent frame
            transform.child_frame_id = "base_footprint"  # Child frame

            # Fill in translation from the matrix
            transform.transform.translation.x = odom_to_base_footprint[0, 3]
            transform.transform.translation.y = odom_to_base_footprint[1, 3]
            transform.transform.translation.z = odom_to_base_footprint[2, 3]

            # Fill in rotation (quaternion) from the matrix
            quat = tf.transformations.quaternion_from_matrix(odom_to_base_footprint)
            transform.transform.rotation.x = quat[0]
            transform.transform.rotation.y = quat[1]
            transform.transform.rotation.z = quat[2]
            transform.transform.rotation.w = quat[3]

            # Publish the transform
            self.tf_broadcaster.sendTransformMessage(transform)

             # Create and publish the odometry message
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "map"
            odom.child_frame_id = "base_footprint"

            # Set the position from the transformation matrix
            odom.pose.pose.position.x = odom_to_base_footprint[0, 3]
            odom.pose.pose.position.y = odom_to_base_footprint[1, 3]
            odom.pose.pose.position.z = odom_to_base_footprint[2, 3]
            odom.pose.pose.orientation = Quaternion(*quat)

            # Optionally, set the velocity (linear and angular)
            odom.twist.twist.linear.x = 0.0  # Replace with actual velocity if available
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = 0.0

            # Publish the odometry message
            self.odom_publisher.publish(odom)


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"Failed to compute transformations: {str(e)}")

        

    def timer_callback(self, event):
        """Timer callback to compute and broadcast map to odom at regular intervals."""
        self.publish_odom_to_base_footprint_tf()

if __name__ == '__main__':
    # Instantiate and run the node
    node = OdomToBaseFootprintPublisher()

    # Use rospy.spin() to keep the node alive and processing callbacks
    rospy.spin()
