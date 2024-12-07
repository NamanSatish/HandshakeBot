#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
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

    # ar_pos, ar_trans = lookup_tag(5)
    
    # manually writing out the transform
    ar_trans = TransformStamped()
    ar_trans.header.frame_id = "base"
    ar_trans.child_frame_id = "ar_marker_5"


    ar_trans.transform.translation.x = 1.1520635773864734
    ar_trans.transform.translation.y = 0.09403840735301512
    ar_trans.transform.translation.z = 0.3057204338258193
    ar_trans.transform.rotation.x = -0.5550855293497743
    ar_trans.transform.rotation.y = 0.4649585908687069
    ar_trans.transform.rotation.z = 0.4652051640233256
    ar_trans.transform.rotation.w = -0.5091932042455977
    # print(ar_pos)
    print("transform: \n", ar_trans)

    #tuck()

    hand_poses_raw = [
[0.2792954458227116, 0.20746030396144485, 0.6270000002634611],
[0.24614906862195018, 0.1985790580698161, 0.6520000002825395],
[0.2305477627607013, 0.18835894277739518, 0.6450000002643731],
[0.2178943275923651, 0.17291054853257334, 0.6270000003071394],
[0.1909910393610335, 0.16106525230357463, 0.6200000003094642],
[0.14055625862154905, 0.15921390534708685, 0.6460000002205467],
[0.11097308418590665, 0.18003259607286234, 0.6480000002630933],
[0.10325810592331426, 0.20527095118681957, 0.6550000002944559],
[0.10822435146626055, 0.17257472474368754, 0.6530000003145734],
[0.11648899890722152, 0.07787504074297669, 0.6570000004248486],
[0.08914648733988616, -0.03245027889705123, 0.6950000002763516],
[0.02647198196104959, -0.07895627006748025, 0.7300000002158563],
[-0.04365718476919022, -0.05517612564525457, 0.7510000001332132],
[-0.09597827025380092, 0.04001302723391594, 0.7620000000577821],
[-0.08090262237270539, 0.1601825111134928, 0.7530000001346298],
[-0.01078914234642985, 0.2493981723364537, 0.7390000000792593],
[0.09274029849907937, 0.26435344943814115, 0.6930000001808035],
[0.18321075617958604, 0.20754475265929645, 0.65800000028659],
[0.19973568897137775, 0.11878768307809145, 0.6590000004201225],
[0.1570671685047608, 0.038208126272829684, 0.6710000003634502],
[0.07620319488338892, -0.0074318084541192975, 0.6810000002824395],
[-0.011866743793299134, 0.014948482276233955, 0.6900000001883503],
[-0.04520379166685437, 0.10826905573207053, 0.6930000001543165],
[0.0273170809868344, 0.2022444462845896, 0.6730000001436685],
[0.11496056850127137, 0.22861632267795567, 0.6440000002081814],
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

            # Setting position and orientation target
            # group.set_position_target(pos[:3])
            # group.set_pose_target(request.ik_request.pose_stamped)
            group.set_pose_targets(hand_poses)

            # TRY THIS
            # Setting just the position without specifying the orientation
            # group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK
            plan = group.plan()
            # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            # if user_input == 'y':
            #     group.execute(plan[1])
            # elif user_input == 'g':
            #     break
            group.execute(plan[1])



        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
