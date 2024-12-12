#!/usr/bin/env python
import json
import sys
import argparse
import numpy as np
import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped


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
        trans = tfBuffer.lookup_transform(
            "base", f"ar_marker_{tag_number}", rospy.Time(0), rospy.Duration(10.0)
        )
        # trans_inv = tfBuffer.lookup_transform(f"ar_marker_{tag_number}", "base",  rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)
        print("Retrying ...")

    tag_pos = [getattr(trans.transform.translation, dim) for dim in ("x", "y", "z")]
    return np.array(tag_pos), trans  # , trans_inv


def load_ar_trans(ar_trans_file="ar_trans.json", directory="/home/HandshakeBot/src/"):
    """Load the AR tag transform from a JSON file."""
    with open(directory + ar_trans_file, "r") as ar_trans_json:
        ar_trans = TransformStamped()
        ar_trans.header.frame_id = ar_trans_json["header"]["frame"]
        ar_trans.child_frame_id = ar_trans_json["header"]["child_frame"]

        ar_trans.transform.translation.x = ar_trans_json["translation"]["x"]
        ar_trans.transform.translation.y = ar_trans_json["translation"]["y"]
        ar_trans.transform.translation.z = ar_trans_json["translation"]["z"]

        ar_trans.transform.rotation.x = ar_trans_json["rotation"]["x"]
        ar_trans.transform.rotation.y = ar_trans_json["rotation"]["y"]
        ar_trans.transform.rotation.z = ar_trans_json["rotation"]["z"]
        ar_trans.transform.rotation.w = ar_trans_json["rotation"]["w"]

    return ar_trans


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ar_tag", "-t", type=int, default=8, help="The AR tag number")

    args = parser.parse_args()

    handshake_directory = "/home/HandshakeBot/src/"

    rospy.init_node("camera_transform")

    # Get the transform
    ar_pos, ar_trans = lookup_tag(args.ar_tag)
    # Convert the TransformStamped message to a dictionary
    ar_trans_dict = {
        "translation": {
            "x": ar_trans.transform.translation.x,
            "y": ar_trans.transform.translation.y,
            "z": ar_trans.transform.translation.z,
        },
        "rotation": {
            "x": ar_trans.transform.rotation.x,
            "y": ar_trans.transform.rotation.y,
            "z": ar_trans.transform.rotation.z,
            "w": ar_trans.transform.rotation.w,
        },
        "header": {"frame": "base", "child_frame": f"ar_marker_{args.ar_tag}"},
    }

    # Save the dictionary to a JSON file
    with open(handshake_directory + "ar_trans.json", "w") as json_file:
        json.dump(ar_trans_dict, json_file, indent=4)
    print(ar_trans)
    print("------------------")


if __name__ == "__main__":
    main()
