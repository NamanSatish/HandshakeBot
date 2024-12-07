#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import sys

# Callback function to process PoseStamped messages
def pose_callback(msg, compute_ik):
    try:
        # Construct the IK service request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        request.ik_request.ik_link_name = "right_gripper_tip"
        request.ik_request.pose_stamped = msg

        # Call the IK service
        response = compute_ik(request)

        if response.error_code.val != response.error_code.SUCCESS:
            rospy.logerr("Inverse Kinematics failed")
            return

        # Plan and execute the trajectory
        group = MoveGroupCommander("right_arm")
        group.set_pose_target(msg.pose)

        plan = group.plan()
        if plan:
            group.execute(plan[1], wait=True)
            rospy.loginfo("Executed arm motion")
        else:
            rospy.logerr("Planning failed")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def main():
    # Initialize the ROS node
    rospy.init_node('arm_controller', anonymous=True)

    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    # Create a subscriber for the /hand_pose topic
    rospy.Subscriber('/hand_pose', PoseStamped, pose_callback, compute_ik)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()