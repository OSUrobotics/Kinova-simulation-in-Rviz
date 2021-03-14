#! /usr/bin/env python


import rospy

import tf
import os
import numpy as np
from math import sin, cos, pi
import csv
import glob
import sys

from visualization_msgs.msg import Marker


directory = os.path.dirname(os.path.realpath(__file__))

def marker_setup():
    """sets up the corner and center markers that represent the world frame from
    from the perspective of the arm, red markers
    """

    markers = [0, 1, 2, 3, 4, 5, 6]
    poses = [[0, 0, -0.005], [0.30, 0.25, 0], [-0.30, 0.25, 0], [-0.30, -0.25, 0], [0.30, -0.25, 0], [0, 0, 0], [0, 0, 0]]
    scales = [[0.60, 0.50, 0.01], [0.036, 0.036, 0.01], [0.036, 0.036, 0.01], [0.036, 0.036, 0.01], [0.036, 0.036, 0.01], [0.036, 0.036, 0.01], [0.036, 0.036, 0.01]]
    colors = [[0, 1, 0, .5], [1, 0, 0, .5], [1, 0, 0, .5], [1, 0, 0, .5], [1, 0, 0, .5], [1, 0, 0, .5], [1, 0, 0, .5]]
    for i in range(len(markers)):
        markers[i] = Marker()
        markers[i].header.frame_id = '/world_frame'
        markers[i].type = markers[i].CUBE
        markers[i].id = i
        markers[i].action = markers[i].ADD
        markers[i].scale.x = scales[i][0] 
        markers[i].scale.y = scales[i][1]
        markers[i].scale.z = scales[i][2]
        markers[i].color.r = colors[i][0] 
        markers[i].color.g = colors[i][1]
        markers[i].color.b = colors[i][2]
        markers[i].color.a = colors[i][3]
        markers[i].pose.position.x = poses[i][0]
        markers[i].pose.position.y = poses[i][1]
        markers[i].pose.position.z = poses[i][2]

    return markers



        
def ee_palm():
    """gets the transform to go from the end effector to the palm Aruco marker"""

    ee_palm = np.zeros((4, 4))
    ee_palm_tran = np.zeros((4, 4))
    ee_palm_rot = np.zeros((4, 4))
    
    with open(directory + '/final_test/EE_to_Palm.csv') as f:
        reader = csv.reader(f)
        for j, row in enumerate(reader):
            for i, col in enumerate(row):
                ee_palm[j][i] = float(col)
    
    with open(directory + '/final_test/EE_to_Palm_Rotation_Matrix.csv') as f:
        reader = csv.reader(f)
        for j, row in enumerate(reader):
            for i, col in enumerate(row):
                ee_palm_rot[j][i] = float(col)
    
#################################################################################################################
    ###Test Stuff Ignore will be Removed Later #############
    # ee_palm_mat = np.dot(ee_palm_rot, ee_palm_tran)

    # rot_auruco_ee = tf.transformations.euler_matrix((-9.38*pi / 180), (-8.83*pi / 180), (0.83*pi / 180))
    # rot_auruco_ee = tf.transformations.euler_matrix(0, 0, 0)
    # rospy.logerr(rot_auruco_ee)
    
    # rot = tf.transformations.quaternion_from_euler((-9.38*pi / 180), (-8.83*pi / 180), (0.83*pi / 180))
    
    # ee_palm_mat = np.dot(ee_palm_tran, rot_auruco_ee)
    # rot = tf.transformations.quaternion_from_matrix(rot_auruco_ee)
    
    # rospy.logerr(tf.transformations.euler_from_matrix(ee_palm_rot))

    # ee_palm_mat = np.dot(np.linalg.inv(ee_palm_mat), rot_auruco_ee)
    # trans = tf.transformations.translation_from_matrix(ee_palm_mat)
    # rot = tf.transformations.quaternion_from_matrix(ee_palm_rot)
    # np.linalg.inv(
#################################################################################################################

    rot = tf.transformations.quaternion_from_matrix(np.linalg.inv(ee_palm))
    trans = tf.transformations.translation_from_matrix(np.linalg.inv(ee_palm))
    rot_ee = tf.transformations.quaternion_from_matrix(ee_palm)
    trans_ee = tf.transformations.translation_from_matrix(ee_palm)
    # rospy.logerr(trans)

    return trans, rot, trans_ee, rot_ee


if __name__ == '__main__':

    rospy.init_node('markers', argv=sys.argv)

    markers = marker_setup()
    
    trans, rot, trans_ee, rot_ee = ee_palm()

    # Modifies the last marker which is the palm aruco marker
    markers[6].header.frame_id = 'aruco_endeffector'  # the end effector flipped to align with the aruco marker
    markers[6].pose.position.x = trans[0]
    markers[6].pose.position.y = trans[1]
    markers[6].pose.position.z = trans[2]
    markers[6].pose.orientation.x = rot[0]
    markers[6].pose.orientation.y = rot[1]
    markers[6].pose.orientation.z = rot[2]
    markers[6].pose.orientation.w = rot[3]

    # Set up a publisher.  We're going to publish on a topic called balloon.
    publisher = rospy.Publisher('markers', Marker, queue_size=10)

    # Set a rate.  10 Hz is a good default rate for a marker moving with the Fetch robot.
    rate = rospy.Rate(10)
    palm_frame = tf.TransformBroadcaster()  # adds the palm to the tf 
    ee_frame_camera = tf.TransformBroadcaster()
    ee_frame_camera_flipped = tf.TransformBroadcaster()
    quan = tf.transformations.quaternion_from_euler(-pi/2, pi, 0)
    # Publish the marker at 10Hz.
    while not rospy.is_shutdown():
        for i in range(len(markers)):
            publisher.publish(markers[i])

        ee_frame_camera.sendTransform(tuple(trans_ee), tuple(rot_ee), rospy.Time.now(), 'ee_frame_camera', 'palm_frame_camera')
        palm_frame.sendTransform(tuple(trans), tuple(rot), rospy.Time.now(), 'palm_frame', 'aruco_endeffector')
        ee_frame_camera_flipped.sendTransform((0, 0, 0), tuple(quan), rospy.Time.now(), 'ee_frame_camera_flipped', 'ee_frame_camera')
        rate.sleep()