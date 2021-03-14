#! /usr/bin/env python


import rospy

import tf
import os
import numpy as np
import csv
import glob
import sys

dir_path = os.path.dirname(os.path.realpath(__file__))

if __name__ == "__main__":

    n = float(sys.argv[1])
    
    rospy.init_node('readTransform', argv=sys.argv)


    listener = tf.TransformListener()
    
    while True:
        try:
            translation, rotation = listener.lookupTransform('j2s7s300_link_base', 'j2s7s300_end_effector', rospy.Time())
            break  # once the transform is obtained move on
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue  # if it fails try again

    transform_mat = listener.fromTranslationRotation(translation, rotation)


    file = open(dir_path + "/data/TransformMatrix_" + str(n) + ".csv", "w")
    wr = csv.writer(file, dialect='excel')

    for i in range(len(transform_mat)):
        wr.writerow(transform_mat[i])
    
    file.close()

