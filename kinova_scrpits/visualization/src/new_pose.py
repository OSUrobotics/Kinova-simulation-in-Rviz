#!/usr/bin/env python


import rospy
import sys
import csv
import glob
import sys
import numpy as np

import tf

import os

from kinova_scripts.srv import New_pose


if __name__ == '__main__':

    rospy.init_node('new_pose', argv=sys.argv, disable_signals=True)

    listener = tf.TransformListener()

    while True:
        try:
            translation, rotation = listener.lookupTransform('j2s7s300_link_base', 'ee_frame_camera_flipped', rospy.Time())
            break  # once the transform is obtained move on
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue  # if it fails try again


    pose = translation + rotation
    print(pose)


    updated_pose = rospy.ServiceProxy('new_pose', New_pose)

    try:
        answer = updated_pose(pose)
    except rospy.ServiceException as e:
        rospy.logwarn('Service call failed for: {0}'.format(e))

    rospy.loginfo( '{0}'.format(answer.success))

    rospy.signal_shutdown('{0}'.format(answer.success))
