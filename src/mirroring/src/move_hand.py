#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_msgs.msg import Constraints, JointConstraint, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import (
    PoseStamped,
    TransformStamped,
    Transform,
    Vector3,
    PointStamped,
    Pose
)
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
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


def pose_callback(msg):

    print("Given Point")
    print(msg)

    restrict = Constraints()
    restrict.name = "constr"

    j0_joint = JointConstraint()
    j0_joint.joint_name = "right_j0"
    j0_joint.position = 0.9
    j0_joint.tolerance_above = 0.6
    j0_joint.tolerance_below = 0.5

    j1_joint = JointConstraint()
    j1_joint.joint_name = "right_j1"
    j1_joint.position = 0.0
    j1_joint.tolerance_above = 1.5
    j1_joint.tolerance_below = 0.1

    j2_joint = JointConstraint()
    j2_joint.joint_name = "right_j2"
    j2_joint.position = 0.0
    j2_joint.tolerance_above = 0.1
    j2_joint.tolerance_below = 0.1

    j3_joint = JointConstraint()
    j3_joint.joint_name = "right_j3"
    j3_joint.position = 0.0
    j3_joint.tolerance_above = np.pi / 2
    j3_joint.tolerance_below = np.pi / 2



    #restrict.joint_constraints = [j0_joint, j1_joint, j2_joint, j3_joint]

    try:

        group = MoveGroupCommander("right_arm")
        scene = PlanningSceneInterface()
        group.set_planner_id("RRTConnectkConfigDefault")
        group.set_goal_orientation_tolerance(100)
        group.set_goal_position_tolerance(0.05)
        group.set_path_constraints(restrict)
        group.set_max_velocity_scaling_factor(0.5)



        




        # group.set_pose_target(hand_rel_base)

        # TRY THIS
        # Setting just the position without specifying the orientation
        group.set_position_target([msg.point.x, msg.point.y, msg.point.z])

        # Plan IK
        plan = group.plan()
        print(plan)
        #user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")

        # Execute IK if safe
        #if user_input == "y":
        group.execute(plan[1])
        #elif user_input == "e":
        #    return 0
        #elif user_input == "n":
        #    rospy.signal_shutdown("reason")
        #    sys.exit()

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def main():
    # Initialize the ROS node
    rospy.init_node("arm_controller2", anonymous=True)



    group = MoveGroupCommander("right_arm")
    scene = PlanningSceneInterface()


    #collision objects
    collision_obj = CollisionObject()
    collision_obj.header.frame_id = group.get_planning_frame()
    collision_obj.id = "box"
    primitive = SolidPrimitive()
    primitive.type = primitive.BOX
    primitive.dimensions = [0, 0, 0]
    #21, 28, 0
    #print(primitive.dimensions)
    primitive.dimensions[0] = 0.2
    primitive.dimensions[1] = 5
    primitive.dimensions[2] = 5

    box_pose = Pose()
    box_pose.orientation.w = 1.0
    box_pose.position.x = -0.21 - 0.1
    box_pose.position.y = 0
    box_pose.position.z = 0

    collision_obj.primitives.append(primitive)
    collision_obj.primitive_poses.append(box_pose)
    collision_obj.operation = collision_obj.ADD

    scene.add_object(collision_obj)
    group.attach_object(collision_obj.id, "base")
    rospy.sleep(4)


    # Create a subscriber for the /hand_pose topic
    rospy.Subscriber("/robot_point", PointStamped, pose_callback)

    # Keep the node running
    rospy.spin()


if __name__ == "__main__":
    main()
