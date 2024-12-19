#!/usr/bin/env python
import rospy
from moveit_msgs.msg import Constraints, JointConstraint, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import (
    PoseStamped,
    PointStamped,
    Pose,
)
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
import numpy as np

from utils.utils import *

# Import python queue for the path_points
import queue

PATH_SIZE = 5


def pose_callback(msg):
    waypt = Pose()
    waypt.position = msg.point
    waypt.orientation.w = 1.0
    waypt.orientation.x = 0.0
    waypt.orientation.y = 1.0
    waypt.orientation.z = 0.0

    # Print the time at which the point was recorded, and the time at which the point was received in seconds
    # print(msg.header.stamp)
    # print(rospy.Time.now())
    time_diff = rospy.Time.now() - msg.header.stamp
    # print(time_diff.to_sec())

    # Get the xyz of the previous point and current point
    if len(path_points) > 0:
        prev_x, prev_y, prev_z = (
            path_points[-1].position.x,
            path_points[-1].position.y,
            path_points[-1].position.z,
        )
        curr_x, curr_y, curr_z = msg.point.x, msg.point.y, msg.point.z

        # Compute the distance between the two points
        dist = np.sqrt(
            (curr_x - prev_x) ** 2 + (curr_y - prev_y) ** 2 + (curr_z - prev_z) ** 2
        )
        # If the distance is more than 0.03 append the point to the path_points
        if dist > 0.01:
            path_points.append(waypt)
    else:
        path_points.append(waypt)

    # path_points.append(waypt)
    if len(path_points) < PATH_SIZE:
        return 0
    try:
        plan, fraction = group.compute_cartesian_path(
            waypoints=path_points,
            eef_step=0.3,
            jump_threshold=0.0,
            avoid_collisions=True,
        )

        # Print if plan is empty
        if plan.joint_trajectory.points == []:
            print("Empty Plan")
            return 0
        else:
            print("Plan not empty")

        # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")

        # Execute IK if safe
        # if user_input == "y":
        group.execute(plan, wait=True)

        path_points.clear()
        # elif user_input == "e":
        #    return 0
        # elif user_input == "n":
        #    rospy.signal_shutdown("reason")
        #    sys.exit()

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def main():
    # Initialize the ROS node
    rospy.init_node("arm_controller", anonymous=True)

    global group
    group = MoveGroupCommander("right_arm")
    scene = PlanningSceneInterface()

    # collision objects
    collision_obj = CollisionObject()
    collision_obj.header.frame_id = group.get_planning_frame()
    collision_obj.id = "box"
    primitive = SolidPrimitive()
    primitive.type = primitive.BOX
    primitive.dimensions = [0, 0, 0]
    # 21, 28, 0
    # print(primitive.dimensions)
    primitive.dimensions[0] = 0.2
    primitive.dimensions[1] = 5
    primitive.dimensions[2] = 5

    box_pose = Pose()
    box_pose.orientation.w = 1.0
    box_pose.position.x = -0.46 - 0.1
    box_pose.position.y = 0
    box_pose.position.z = 0

    collision_obj.primitives.append(primitive)
    collision_obj.primitive_poses.append(box_pose)
    collision_obj.operation = collision_obj.ADD

    # scene.add_object(collision_obj)
    # group.attach_object(collision_obj.id, "base")

    primitive2 = SolidPrimitive()
    primitive2.type = primitive2.BOX
    primitive2.dimensions = [0, 0, 0]
    # 21, 28, 0
    # print(primitive.dimensions)
    primitive2.dimensions[0] = 5
    primitive2.dimensions[1] = 5
    primitive2.dimensions[2] = 0.2

    box_pose2 = Pose()
    box_pose2.orientation.w = 1.0
    box_pose2.position.x = 0
    box_pose2.position.y = 0
    box_pose2.position.z = 1

    collision_obj.primitives.append(primitive2)
    collision_obj.primitive_poses.append(box_pose2)
    # collision_obj2.operation = collision_obj2.ADD

    scene.add_object(collision_obj)
    group.attach_object(collision_obj.id, "base")

    # Create the restrict object
    global restrict
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

    j4_joint = JointConstraint()
    j4_joint.joint_name = "right_j4"
    j4_joint.position = -0.25
    j4_joint.tolerance_above = 0.2
    j4_joint.tolerance_below = 0.2

    restrict.joint_constraints = [j2_joint, j3_joint, j4_joint]

    rospy.sleep(4)

    global path_points
    path_points = []

    # Create the move group commander
    group.set_planner_id("RRTConnectkConfigDefault")
    group.set_max_velocity_scaling_factor(0.4)
    group.set_pose_reference_frame("base")
    group.set_goal_orientation_tolerance(0.1)

    # Create a subscriber for the /hand_pose topic
    rospy.Subscriber("/robot_point", PointStamped, pose_callback)

    # Keep the node running
    rospy.spin()


if __name__ == "__main__":
    main()
