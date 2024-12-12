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


def pose_callback(msg):

    print("Given Point")
    print(msg)

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
        
        #group.set_pose_target(hand_rel_base)

        # TRY THIS
        # Setting just the position without specifying the orientation
        group.set_position_target([msg.point.x, msg.point.y, msg.point.z])

        # Plan IK
        plan = group.plan()
        print(plan)
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

def main():
    # Initialize the ROS node
    rospy.init_node('arm_controller2', anonymous=True)


    # Create a subscriber for the /hand_pose topic
    rospy.Subscriber('/robot_point', PointStamped, pose_callback)

    # Keep the node running
    rospy.spin()


if __name__ == '__main__':
    main()
