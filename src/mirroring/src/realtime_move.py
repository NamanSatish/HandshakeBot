#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_msgs.msg import Constraints, JointConstraint
from geometry_msgs.msg import (
    PoseStamped,
    TransformStamped,
    Transform,
    Vector3,
    PointStamped,
)
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys

import argparse
import rospkg
import roslaunch

from paths.trajectories import LinearTrajectory, CircularTrajectory
from paths.paths import MotionPath
from paths.path_planner import PathPlanner
from controllers.controllers import (
    PIDJointVelocityController,
    FeedforwardJointVelocityController,
)
from utils.utils import *

from trac_ik_python.trac_ik import IK

import tf2_ros
import intera_interface
from moveit_msgs.msg import DisplayTrajectory, RobotState
from sawyer_pykdl import sawyer_kinematics

from tf2_geometry_msgs import do_transform_pose, do_transform_point
from cam_transform import load_ar_trans, tuck


def pose_callback(msg, publisher):
    # ar_trans = TransformStamped()
    # ar_trans.header.frame_id = "base"
    # ar_trans.child_frame_id = "ar_marker_5"

    # ar_trans.transform.translation.x = 1.2470560459434177
    # ar_trans.transform.translation.y =0.325631514226347
    # ar_trans.transform.translation.z = 0.3113295629800095
    # ar_trans.transform.rotation.x = -0.5171229059977078
    # ar_trans.transform.rotation.y = -0.5753025430376476
    # ar_trans.transform.rotation.z = 0.45681815857381536
    # ar_trans.transform.rotation.w = 0.43923576136755066

    ar_trans = load_ar_trans(ar_trans_file="ar_trans.json")

    hand_rel_base = do_transform_point(msg.point, transform)

    print("Given Point")
    print(msg)
    print("Trans Point")
    print(hand_rel_base)

    publisher.publish(hand_rel_base)


"""
    restrict = Constraints()
    restrict.name = "constr"

    l2_joint = JointConstraint()
    l2_joint.joint_name = "right_j2"
    l2_joint.position = 0.
    l2_joint.tolerance_above = 0.6
    l2_joint.tolerance_below = 0.6

    l1_joint = JointConstraint()
    l1_joint.joint_name = "right_j1"
    l1_joint.position = 0.
    l1_joint.tolerance_above = 0.6
    l1_joint.tolerance_below = 0.6

    l0_joint = JointConstraint()
    l0_joint.joint_name = "right_j0"
    l0_joint.position = 0.
    l0_joint.tolerance_above = 0.6
    l0_joint.tolerance_below = 0.6

    restrict.joint_constraints = [l0_joint, l1_joint, l2_joint]



    try:
        
        group = MoveGroupCommander("right_arm")
        group.set_planner_id("RRTConnectkConfigDefault")
        group.set_goal_orientation_tolerance(100)
        group.set_goal_position_tolerance(0.05)
        group.set_path_constraints(restrict)
        
        group.set_pose_target(hand_rel_base)

        # TRY THIS
        # Setting just the position without specifying the orientation
        # group.set_position_target([0.5, 0.5, 0.0])

        # Plan IK
        plan = group.plan()
        
        user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")

        # Execute IK if safe
        if user_input == 'y':
            group.execute(plan[1])
        elif user_input == 'e':
            return 0
        elif user_input == 'n':
            rospy.signal_shutdown("reason")
            sys.exit()
        

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
"""


def main():
    # Initialize the ROS node
    rospy.init_node("arm_controller", anonymous=True)

    # Create the static transform for the camera_location frame (broadcast only once)
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Create a TransformStamped message for the camera_location frame
    global transform
    transform = load_ar_trans()
    transform.child_frame_id = "camera_location"  # name new frame
    # transform = TransformStamped()
    # transform.header.stamp = rospy.Time.now()
    # transform.header.frame_id = "base"  # Parent frame (e.g., "base")
    # transform.child_frame_id = "camera_location"  # Name of the new frame

    # # Define the translation (position of the new frame in the base frame)
    # transform.transform.translation.x = 0.9862231938792343
    # transform.transform.translation.y = 0.6982211803285568
    # transform.transform.translation.z = 0.4146117198281746
    # transform.transform.rotation.x = -0.5157978911299365
    # transform.transform.rotation.y = -0.4428540978134041
    # transform.transform.rotation.z = 0.5088505215632815
    # transform.transform.rotation.w = 0.5281135581109083

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
