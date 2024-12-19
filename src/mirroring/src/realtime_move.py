#!/usr/bin/env python
import rospy
from geometry_msgs.msg import (
    PoseStamped,
    PointStamped,
)
from utils.utils import *

from trac_ik_python.trac_ik import IK

import tf2_ros

from tf2_geometry_msgs import do_transform_pose, do_transform_point
from cam_transform import load_ar_trans, tuck


def pose_callback(msg, publisher):

    hand_rel_base = do_transform_point(msg, transform)

    print("Given Point")
    print(msg)
    print("Trans Point")
    print(hand_rel_base)

    # Hyperparameters for offsetting the hand position
    offset_x = -0.1
    offset_y = 0.2
    offset_z = 0.05

    # Offset the hand position
    # hand_rel_base.point.x += offset_x
    # hand_rel_base.point.y += offset_y
    # hand_rel_base.point.z += offset_z

    publisher.publish(hand_rel_base)


def main():
    # Initialize the ROS node
    rospy.init_node("cam_transform", anonymous=True)

    # Create the static transform for the camera_location frame (broadcast only once)
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Create a TransformStamped message for the camera_location frame
    global transform
    transform = load_ar_trans()
    transform.child_frame_id = "camera_location"  # name new frame
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "base"  # Parent frame (e.g., "base")

    # Broadcast the static transform (this will be broadcast only once)
    tf_broadcaster.sendTransform(transform)

    # Create a publisher for the /hand_point_base topic
    publisher = rospy.Publisher("/hand_pose_base", PointStamped, queue_size=1)

    # Create a subscriber for the /hand_pose topic
    rospy.Subscriber("/hand_pose", PointStamped, pose_callback, publisher)

    # Keep the node running
    rospy.spin()


if __name__ == "__main__":
    main()
