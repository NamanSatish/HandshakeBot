#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_msgs.msg import Constraints, JointConstraint
from geometry_msgs.msg import PoseStamped, TransformStamped, Transform, Vector3, PointStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys

import sys
import argparse
import numpy as np
import rospkg
import roslaunch

from paths.trajectories import LinearTrajectory, CircularTrajectory
from paths.paths import MotionPath
from paths.path_planner import PathPlanner
from controllers.controllers import ( 
    PIDJointVelocityController, 
    FeedforwardJointVelocityController
)
from utils.utils import *

from trac_ik_python.trac_ik import IK

import rospy
import tf2_ros
import intera_interface
from moveit_msgs.msg import DisplayTrajectory, RobotState
from sawyer_pykdl import sawyer_kinematics

from tf2_geometry_msgs import do_transform_pose, do_transform_point


def pose_callback(msg, publisher):
    # Use the existing transform from camera_location to base
    try:
        # Now apply the transform to the incoming point
        transformed_point = do_transform_point(msg, transform)

        # Publish the transformed point
        publisher.publish(transformed_point)

        # Output the transformed point to the console
        print("Given Point:", msg)
        print("Transformed Point:", transformed_point)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr("Transform error: %s" % e)



def main():
    # Initialize the ROS node
    rospy.init_node('arm_controller', anonymous=True)

    # Create the static transform for the camera_location frame (broadcast only once)
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Create a TransformStamped message for the camera_location frame
    global transform
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "base"  # Parent frame (e.g., "base")
    transform.child_frame_id = "camera_location"  # Name of the new frame

    # Define the translation (position of the new frame in the base frame)
    transform.transform.translation.x = 0.9862231938792343
    transform.transform.translation.y = 0.6982211803285568
    transform.transform.translation.z = 0.4146117198281746
    transform.transform.rotation.x = -0.5157978911299365
    transform.transform.rotation.y = -0.4428540978134041
    transform.transform.rotation.z = 0.5088505215632815
    transform.transform.rotation.w = 0.5281135581109083

    # Broadcast the static transform (this will be broadcast only once)
    tf_broadcaster.sendTransform(transform)

    # Create a publisher for the /hand_point_base topic
    publisher = rospy.Publisher('/hand_pose_base', PointStamped, queue_size=10)

    # Create a subscriber for the /hand_pose topic
    rospy.Subscriber('/hand_pose', PointStamped, pose_callback, publisher)

    # Keep the node running
    rospy.spin()


if __name__ == '__main__':
    main()
