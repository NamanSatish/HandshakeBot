#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_msgs.msg import Constraints
from geometry_msgs.msg import PoseStamped, TransformStamped, Transform, Vector3
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

from tf2_geometry_msgs import do_transform_pose


def tuck():
    """
    Tuck the robot arm to the start position. Use with caution
    """
    if input('Would you like to tuck the arm? (y/n): ') == 'y':
        rospack = rospkg.RosPack()
        path = rospack.get_path('sawyer_full_stack')
        launch_path = path + '/launch/custom_sawyer_tuck.launch'
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
        launch.start()
    else:
        print('Canceled. Not tucking the arm.')

def lookup_tag(tag_number):
    """
    Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
    You can use either this function or try starting the scripts/tag_pub.py script.  More info
    about that script is in that file.  

    Parametersar_pos
    ----------
    tag_number : int

    Returns
    -------
    3x' :obj:`numpy.ndarray`
        tag position
    """

    
    # TODO: initialize a tf buffer and listener as in lab 3
    
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    try:
        # TODO: lookup the transform and save it in trans

        # The rospy.Time(0) is the latest available 
        # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
        trans = tfBuffer.lookup_transform("base", f"ar_marker_{tag_number}", rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)
        print("Retrying ...")

    tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
    return np.array(tag_pos), trans

def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    #Get the transform 
    #ar_pos, ar_trans = lookup_tag(5)
    #print(ar_trans)
    
    # manually writing out the transform
    ar_trans = TransformStamped()
    ar_trans.header.frame_id = "base"
    ar_trans.child_frame_id = "ar_marker_5"


    ar_trans.transform.translation.x = 1.111721371448794
    ar_trans.transform.translation.y =0.34143258202829513
    ar_trans.transform.translation.z = 0.30799038163007
    ar_trans.transform.rotation.x = -0.5357408298438834
    ar_trans.transform.rotation.y = 0.5801812211635239
    ar_trans.transform.rotation.z = 0.4957531745456158
    ar_trans.transform.rotation.w = -0.3613866402834157
    # print(ar_pos)
    print("transform: \n", ar_trans)

    #manually set constraints
    #moveit_constraints = Constraints()

    #tuck()

    hand_poses_raw = [
        [0.5,0.5,0.3],
        [0.2, 0.5, 0.3],
        [0.2, 0.2, 0.3],
        [0.5, 0.2, 0.3] 
    ]

    hand_poses = []
    for pose in hand_poses_raw:
        hand_point_trans = PoseStamped()
        hand_point_trans.pose.position.x = pose[0]
        hand_point_trans.pose.position.y = pose[1]
        hand_point_trans.pose.position.z = pose[2]
        hand_point_trans.pose.orientation.x = 0
        hand_point_trans.pose.orientation.y = 1.0
        hand_point_trans.pose.orientation.z = 0
        hand_point_trans.pose.orientation.w = 0

        hand_rel_base = do_transform_pose(hand_point_trans, ar_trans)
        hand_poses.append(hand_rel_base.pose)

    
    '''target_poses = [
        [0.985, 0.207, 0.443, 0.5, 0.5, 0.5, 0.5],
        [0.985, 0.18, 0.403, 0.5, 0.5, 0.5, 0.5],
        [0.935, 0.18, 0.443, 0.5, 0.5, 0.5, 0.5],
        [0.935, 0.207, 0.443, 0.5, 0.5, 0.5, 0.5],
    ]'''

    # pos = [hand_rel_base.pose.position.x, hand_rel_base.pose.position.y, hand_rel_base.pose.position.z, 0, 1, 0, 0]
    while not rospy.is_shutdown():
        # input('Press [ Enter ]: ')
        
        # Construct the request
        request = GetPositionIKRequest()

        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        #link = "stp_022312TP99620_tip_1" # for amir

        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        

        # Set the desired orientation for the end effector HERE

        # request.ik_request.pose_stamped.pose.position.x = pos[0]
        # request.ik_request.pose_stamped.pose.position.y = pos[1]
        # request.ik_request.pose_stamped.pose.position.z = pos[2] 
        # request.ik_request.pose_stamped.pose.orientation.x = pos[3]
        # request.ik_request.pose_stamped.pose.orientation.y = pos[4]
        # request.ik_request.pose_stamped.pose.orientation.z = pos[5]
        # request.ik_request.pose_stamped.pose.orientation.w = pos[6]
        
        try:
            # Send the request to the service
            # response = compute_ik(request)
            
            # # Print the response HERE
            # print("hi: ", response)
            group = MoveGroupCommander("right_arm")
            group.set_planner_id("RRTConnectkConfigDefault")
            group.set_goal_orientation_tolerance(100)
            
            # Setting position and orientation target
            #print(hand_poses[0])
            #group.set_position_target([hand_poses[0].position.x, hand_poses[0].position.y, hand_poses[0].position.z ])
            # group.set_pose_target(request.ik_request.pose_stamped)
            group.set_pose_targets(hand_poses)

            # TRY THIS
            # Setting just the position without specifying the orientation
            # group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK
            plan = group.plan()
            
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                 group.execute(plan[1])
            elif user_input == 'n':
                sys.exit()
            #group.execute(plan[1])



        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
