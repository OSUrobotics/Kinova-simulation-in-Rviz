#!/usr/bin/env python


import rospy
import sys
import csv
import glob
import sys
import numpy as np
import os
from sensor_msgs.msg import JointState
from kinova_scripts.srv import Distance
import kinova_msgs.msg



if __name__ == '__main__':
    ##########################################################################################
    # Reads the distance and sends them to the plannar to create
    ##########################################################################################

    try:  # get input from command line for which file to use
        n = sys.argv[1]
    except:
        n = 6

    rospy.init_node('distance', argv=sys.argv, disable_signals=True)

    # directory = os.path.dirname(os.path.realpath(__file__))

    stance = np.zeros((1, 4))

    # set the file number parameter so other programs update
    rospy.set_param('file_number', str(n))

    # get distance from Identify file
    # distance = data


    arm_angles = angles[0]
    print(arm_angles)
    print(arm_angles[:7])
    print(arm_angles[7:-3])


    # print(kinova_msgs.msg.SetFingersPositionGoal())
    rospy.wait_for_service('distance')

    set_distance = rospy.ServiceProxy('distance', Distance)

    # send the angles to the plannar to move the arm
    try:
        answer = set_angles(distance)
    except rospy.ServiceException as e:
        rospy.logwarn('Service call failed for: {0}'.format(e))

    rospy.loginfo('{0}'.format(answer.success))

    rospy.signal_shutdown('{0}'.format(answer.success))
