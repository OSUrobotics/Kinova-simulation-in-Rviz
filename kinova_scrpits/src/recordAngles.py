#! /usr/bin/env python


import rospy

from sensor_msgs.msg import JointState

import tf
import os
import numpy as np
import csv
import glob
import sys

def callback(msg):

    angles = msg.position
    print(angles)

    dir_path = os.path.dirname(os.path.realpath(__file__))


    file = open(dir_path + "/data/Angles_" + str(n) + ".csv", "w")
    wr = csv.writer(file, dialect='excel')

    wr.writerow(angles)
    
    file.close()

    rospy.signal_shutdown('done recording')
    

if __name__ == "__main__":

    n = float(sys.argv[1])
    
    rospy.init_node('readTransform', argv=sys.argv, disable_signals=True)

    # For simulator use
    # /move_group/fake_controller_joint_states

    # For real arm
    # '/j2s7s300_driver/out/joint_state'

    subscriber = rospy.Subscriber('/joint_states', JointState, callback)


    rospy.spin()
    # listener = tf.TransformListener()
    
    # while True:
    #     try:
    #         translation, rotation = listener.lookupTransform('j2s7s300_link_base', 'j2s7s300_end_effector', rospy.Time())
    #         break  # once the transform is obtained move on
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         continue  # if it fails try again

    # transform_mat = listener.fromTranslationRotation(translation, rotation)



    
