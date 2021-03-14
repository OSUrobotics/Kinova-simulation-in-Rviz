#!/usr/bin/env python


import rospy

import tf
import os
import numpy as np
from math import pi
import csv
import glob
import sys




if __name__ == '__main__':

    ###################################################################################
    # Creates the camera, world, and the aruco_end_effector frames to TF to be used later 
    ###################################################################################

    rospy.init_node('camera_frame_tf')

    directory = os.path.dirname(os.path.realpath(__file__))

    camera_mat = np.zeros((4, 4))
    translation_mat = np.zeros((4, 4))
    rotation_mat = np.zeros((4, 4))

    # reads in transforms for the frames
    with open(directory + '/final_test/TranslationMatrix.csv') as f:
        reader = csv.reader(f)
        for j, row in enumerate(reader):
            for i, col in enumerate(row):
                translation_mat[j][i] = float(col)


    with open(directory + '/final_test/RotationMatrix.csv') as f:
        reader = csv.reader(f)
        for j, row in enumerate(reader):
            for i, col in enumerate(row):
                rotation_mat[j][i] = float(col)


    with open(directory + '/final_test/World_to_Camera_Transform_Matrix.csv') as f:
        reader = csv.reader(f)
        for j, row in enumerate(reader):
            for i, col in enumerate(row):
                camera_mat[j][i] = float(col)

    camera_transform = np.dot(camera_mat, np.dot(translation_mat, rotation_mat))
    camera_transform = np.linalg.inv(camera_transform)
    
    transform = np.dot(rotation_mat, translation_mat)
    transform = np.linalg.inv(transform)
    
    # setting up the world frame
    trans = tf.transformations.translation_from_matrix(transform)
    rot = tf.transformations.quaternion_from_matrix(transform)
    
    # setting up the camera frame
    trans2 = tf.transformations.translation_from_matrix(camera_transform)
    rot2 = tf.transformations.quaternion_from_matrix(camera_transform)
    
    # setting up the aruco_end_effector frame which is fixed and known
    trans3 = (0, 0, 0)
    rot3 = tf.transformations.quaternion_from_euler(-pi/2, pi, 0)

    br = tf.TransformBroadcaster()
    camera_world = tf.TransformBroadcaster()
    aruco_ee = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        br.sendTransform(tuple(trans), tuple(rot), rospy.Time.now(), 'world_frame', 'j2s7s300_link_base') # j2s7s300_end_effector
        camera_world.sendTransform(tuple(trans2), tuple(rot2), rospy.Time.now(), 'camera_frame', 'j2s7s300_link_base')
        aruco_ee.sendTransform(trans3, tuple(rot3), rospy.Time.now(), 'aruco_endeffector', 'j2s7s300_end_effector')
        rate.sleep()


