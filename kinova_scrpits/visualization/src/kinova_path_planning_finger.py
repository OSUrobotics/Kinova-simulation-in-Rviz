#! /usr/bin/env python

########################################################################################################################
# Author: Nuha Nishat
# Date: 1/30/20
#
# Modified By: Josh Campbell
# Date: 12/9/2020
# Edits to main to allow a service call to give joint angles
#
# Modified BY: Haonan Yuan
# Date: 03/02/2021
# Edits the move_gripper function, change the function name to go_to_finger_state
# Allow the Kinova arm and finger move to the goal state
# Add the kinova finger speed controller
########################################################################################################################


import rospy
import sys, os
import math
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf, math
import tf.transformations
import pdb
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String

from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, PlanningScene, PlanningSceneComponents, AllowedCollisionEntry, \
    AllowedCollisionMatrix
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene

from kinova_scripts.srv import Joint_angles, Joint_anglesResponse
from kinova_scripts.srv import New_pose, New_poseResponse
import time
import numpy as np


# move_group_python_interface_tutorial was used as reference

class MoveRobot():
    def __init__(self):
        # Initialize moveit commander and ros node for moveit
        moveit_commander.roscpp_initialize(sys.argv)

        # Initializing node
        rospy.init_node("move_kinova", anonymous=True)

        # Define robot using RobotCommander. Provided robot info such as
        # kinematic model and current joint state
        self.robot = moveit_commander.RobotCommander()

        # Setting the world
        self.scene = moveit_commander.PlanningSceneInterface()

        # Define the planning group for the arm you are using
        # You can easily look it up on rviz under the MotionPlanning tab
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.move_gripper = moveit_commander.MoveGroupCommander("gripper")
        rospy.loginfo(self.move_gripper.get_current_joint_values())

        # Set the precision of the robot
        rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.0)

        rospy.wait_for_service("/apply_planning_scene", 10.0)
        rospy.wait_for_service("/get_planning_scene", 10.0)

        self.apply_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        self.get_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        rospy.sleep(2)

        # To see the trajectory
        self.disp = moveit_msgs.msg.DisplayTrajectory()

        self.disp.trajectory_start = self.robot.get_current_state()

        self.rate = rospy.Rate(10)

        self.move_group.allow_replanning(1)

        self.joint_angles_service = rospy.Service('joint_angles', Joint_angles, self.joint_angles)

        # self.new_pose_service = rospy.Service('new_pose', New_pose, self.new_pose)

    # self.main()

    def set_planner_type(self, planner_name):
        if planner_name == "RRT":
            self.move_group.set_planner_id("RRTConnectkConfigDefault")
        if planner_name == "RRT*":
            self.move_group.set_planner_id("RRTstarkConfigDefault")
        if planner_name == "PRM*":
            self.move_group.set_planner_id("PRMstarkConfigDefault")

    def go_to_joint_state(self, joint_state):
        joint_goal = JointState()
        joint_goal.position = joint_state
        self.move_group.set_joint_value_target(joint_goal.position)

        self.plan = self.move_group.plan()
        self.move_group.go(wait=True)
        self.move_group.execute(self.plan, wait=True)

        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.sleep(2)

    def go_to_goal(self, ee_pose):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = ee_pose[0]
        pose_goal.position.y = ee_pose[1]
        pose_goal.position.z = ee_pose[2]

        rospy.logerr(len(ee_pose))

        if len(ee_pose) == 6:
            quat = tf.transformations.quaternion_from_euler(math.radians(ee_pose[3]), math.radians(ee_pose[4]),
                                                            math.radians(ee_pose[5]))
            pose_goal.orientation.x = quat[0]
            pose_goal.orientation.y = quat[1]
            pose_goal.orientation.z = quat[2]
            pose_goal.orientation.w = quat[3]

        else:
            pose_goal.orientation.x = ee_pose[3]
            pose_goal.orientation.y = ee_pose[4]
            pose_goal.orientation.z = ee_pose[5]
            pose_goal.orientation.w = ee_pose[6]

        self.move_group.set_pose_target(pose_goal)
        self.move_group.set_planning_time(20)
        rospy.sleep(2)
        self.move_group.go(wait=True)
        self.move_group.stop()

        self.move_group.clear_pose_targets()
        rospy.sleep(2)

    def go_to_finger_state(self, cmd, scaling_value=0.05):
        self.move_gripper.set_max_velocity_scaling_factor(scaling_value)
        if cmd == "Close":
            self.move_gripper.set_named_target("Close")
        elif cmd == "Open":
            self.move_gripper.set_named_target("Open")
        else:
            """gripper_goal = JointState()
            gripper_goal.position = cmd"""
            # set the velocity when finger close or open to the goal position
            # the parm is a scaling factor from 0 to 1 and the max velocity is set to 1 in kinova xarco file
            self.move_gripper.set_joint_value_target(cmd)

        # self.plan_gripper = self.move_gripper.plan()

        self.move_gripper.go(wait=True)
        """self.move_gripper.execute(self.plan_gripper, wait=True)"""
        self.move_gripper.stop()
        self.move_gripper.clear_pose_targets()
        # rospy.sleep(2)

    def display_trajectory(self):
        self.disp_pub = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory,
                                        queue_size=20)
        self.disp.trajectory.append(self.plan)
        print(self.disp.trajectory)
        self.disp_pub.publish(self.disp)

    def new_pose(self, request):
        self.main(0, request.pose)

        return New_poseResponse(True)

    def joint_angles(self, arm):
        """calls the main() function giving it the desired joint angles"""
        self.main(1, arm.angles)
        # self.distance_service = rospy.Service('distance', Distance, self.distance)
        return Joint_anglesResponse(True)

    # def distance(self, ):

    def main(self, trig, goal):

        # Set up path here

        # Pick planner
        self.set_planner_type("RRT")

        if trig == 1:
            # Draw a straight line in 90 deg
            rospy.loginfo('moving')
            joint_radians = goal[:7]  # angles for the arm
            # angles for the gripper this code currently  work
            # For the kinova, just control the angle of proximal finger, that's why choose [7:-3]
            finger_radians = goal[7:-3]
            speed = 0.1

            self.go_to_joint_state(joint_radians)
            if finger_radians[0] < 1:
                self.go_to_finger_state(finger_radians, speed)
            else:
                for i in range(1, 10):
                    stage = np.array([finger_radians[0]/(10-i), finger_radians[1]/(10-i), finger_radians[2]])
                    self.go_to_finger_state(stage, speed/(i+1))


        elif trig == 0:
            self.go_to_goal(list(goal))

        elif trig == 2:
            speed = goal[0]
            self.go_to_finger_state('close', speed)

    # self.move_gripper(angles)

    # rospy.loginfo('Going to first point')
    # self.go_to_goal([-0.1, -0.63, 0.2, 0, 180, 0])

    # rospy.loginfo('Moving down')
    # self.go_to_goal([-0.1, -0.63, 0.097, 0, 180, 0])

    # rospy.loginfo("Going to second point")
    # self.go_to_goal([0.1, -0.63, 0.097, 0, 180, 0])

    # rospy.loginfo('Moving Up')
    # self.go_to_goal([0.1, -0.63, 0.2, 0, 180, 0])


if __name__ == '__main__':
    MoveRobot()

    rospy.spin()