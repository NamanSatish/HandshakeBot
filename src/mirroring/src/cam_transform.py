#!/usr/bin/env python
import json
import sys
import numpy as np
import rospy

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
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    try:
        # The rospy.Time(0) is the latest available 
        # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
        trans = tfBuffer.lookup_transform("base", f"ar_marker_{tag_number}", rospy.Time(0), rospy.Duration(10.0))
        #trans_inv = tfBuffer.lookup_transform(f"ar_marker_{tag_number}", "base",  rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)
        print("Retrying ...")

    tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
    return np.array(tag_pos), trans#, trans_inv

def main():
    # Get the transform 
    ar_pos, ar_trans = lookup_tag(5)
    # Convert the TransformStamped message to a dictionary
    ar_trans_dict = {
        'translation': {
            'x': ar_trans.transform.translation.x,
            'y': ar_trans.transform.translation.y,
            'z': ar_trans.transform.translation.z
        },
        'rotation': {
            'x': ar_trans.transform.rotation.x,
            'y': ar_trans.transform.rotation.y,
            'z': ar_trans.transform.rotation.z,
            'w': ar_trans.transform.rotation.w
        }
    }

    folder_path = "/cam_transforms/"
    # Save the dictionary to a JSON file
    with open(folder_path + 'ar_trans.json', 'w') as json_file:
        json.dump(ar_trans_dict, json_file, indent=4)
    print(ar_trans)
    print("------------------")
