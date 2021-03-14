#! /usr/bin/env python


import rospy

import tf
from tf.transformations import quaternion_from_euler
import os
import numpy as np
import csv
import glob
import sys

from visualization_msgs.msg import Marker


class Markers:
    """ Places the aruco markers as seen by the overhead web camera"""

    def __init__(self):

        self.dir_path = os.path.dirname(os.path.realpath(__file__)) # current directory
        self.camera_pub = rospy.Publisher('camera_markers', Marker, queue_size=10)

        # self.arm_marker()
        self.marker_dict = {}
        self.marker_names = []
        self.markers = []

        self.palm_frame_camera = tf.TransformBroadcaster()
        self.palm_frame_trans = []
        self.palm_frame_quan = []

        self.file_number = rospy.get_param('file_number') # paramater set in launch file

        self.camera_marker()

        rate = rospy.Rate(10)

        # Publish the marker at 10Hz.
        while not rospy.is_shutdown():
            file_num = rospy.get_param('file_number')
            if file_num != self.file_number:  # checks to see if the parameter has changed and if so updates markers pose
                self.file_number = file_num
                self.camera_marker()
            
            self.palm_frame_camera.sendTransform(tuple(self.palm_frame_trans), tuple(self.palm_frame_quan), rospy.Time.now(), 'palm_frame_camera', '/world_frame')

            for i in range(len(self.markers)):
                self.camera_pub.publish(self.markers[i])

            rate.sleep()




    def camera_marker(self):
        self.readfile()
        self.marker_setup()
        

    def arm_marker(self):

        self.readfile()
        

    def marker_setup(self):
        """"  
        Sets up the markers which are the same size
        """
        
        self.markers[:] = self.marker_names[:]
        for i in range(len(self.markers)):

            # rospy.logerr(self.marker_dict[self.marker_names[i]])
            self.markers[i] = Marker()
            self.markers[i].header.frame_id = '/world_frame'
            self.markers[i].type = self.markers[i].CUBE
            self.markers[i].id = i
            self.markers[i].action = self.markers[i].ADD
            self.markers[i].scale.x = 0.036  # Markers are 3.6cm square
            self.markers[i].scale.y = 0.036
            self.markers[i].scale.z = 0.01

            self.markers[i].color.r = 0
            self.markers[i].color.g = 0
            self.markers[i].color.b = 1  # blue is camera frame
            self.markers[i].color.a = 1

            pose = self.marker_dict[self.marker_names[i]]

            self.markers[i].pose.position.x = pose[0]
            self.markers[i].pose.position.y = pose[1]
            self.markers[i].pose.position.z = pose[2]
            
            quan = quaternion_from_euler(np.pi * pose[3] / 180, np.pi * pose[4] / 180, np.pi * pose[5] / 180)
            self.markers[i].pose.orientation.x = quan[0]
            self.markers[i].pose.orientation.y = quan[1]
            self.markers[i].pose.orientation.z = quan[2]
            self.markers[i].pose.orientation.w = quan[3]
            

            if self.marker_names[i] == 'palm':
                self.palm_frame_trans = pose[:3]
                self.palm_frame_quan = quan

    def readfile(self):
        """Reads in a csv file marked with the file_number that has multiple aruco markers' pose and orientation
        first column has the name for the marker and the first row states the order
        """

        with open(self.dir_path + '/final_test/data_file_' + self.file_number + '.csv') as f:
            reader = csv.reader(f)
            next(reader)
            for j, row in enumerate(reader):
                data_list = []
                for i in range(1, len(row)):
                    data_list.append(float(row[i]))

                # stores the files in a dictionary and a list
                self.marker_names.append(str(row[0]))
                self.marker_dict[str(row[0])] = data_list








if __name__ == '__main__':

    rospy.init_node('camera_markers')

    marker = Markers()

    rospy.spin()