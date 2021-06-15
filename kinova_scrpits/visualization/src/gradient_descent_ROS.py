#! /usr/bin/env python

import rospy
import tf
import numpy as np
import csv
import os
import pylab as plt

from kinova_scripts.srv import Joint_angles

directory = os.path.dirname(os.path.realpath(__file__))

class RobotArm:
    def __init__(self):
        
        self.listener = tf.TransformListener()

        rospy.wait_for_service('joint_angles')
        self.update_arm_angles = rospy.ServiceProxy('joint_angles', Joint_angles)

        self.joint_angles = {}
        self.joint_lengths = {"joint1": 0, "joint2": 0.2755, "joint3": 0.205, "joint4": 0.205, "joint5": 0.2073,
                              "joint6": 0.1038, "joint7": 0.1038}
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        self.joint_min = {"joint1": -1744.444444, "joint2": 2.442222222, "joint3": -17444.44444, "joint4": 0.523333333,
                          "joint5": -1744.444444,
                          "joint6": 1.133888889, "joint7": -174.4444444}
        self.joint_max = {"joint1": 174.4444444, "joint2": 5.460111111, "joint3": 2.878333333, "joint4": 5.756666667,
                          "joint5": 174.4444444,
                          "joint6": 5.146111111, "joint7": 174.4444444}

        # Get joint angles from physical data and update angles within ROS
        self.get_joint_angles_from_physical_data()
        self.update_angles()

        # Determine the target endpoint based off of the diff in the End effector location and palm
        self.arm_endpoint = np.array([])
        # Sets the arm endpoint value
        self.get_current_ee_pose()

        # Actual Palm arUco marker location
        self.target_endpoint = np.array([])
        # Sets the target endpoint value
        self.get_goal_ee_pose()

        self.done_parts = []  # Stores final arm parts for plotting
        self.done_parts_dict = {}
        self.cost_values = []  # Holds each cost value made from changes in gradient descent

    def get_joint_angles_from_physical_data(self):
        """ Gets the joint angles saved within arm_cal/ with the physical robot arm data """
        print("\nGetting the physical arm joint angles...")
        with open(directory + '/final_test/test_data/Matrices/Angles_9.0.csv') as file:
            joint_data = csv.reader(file)
            idx = 0
            for row in joint_data:
                for j in row:
                    key = self.joint_names[idx]
                    self.joint_angles[key] = float(j)
                    print(key, ": ", self.joint_angles[key])
                    idx += 1
                    if idx == len(self.joint_names):
                        break
            file.close()

    def get_ee_transform_matrices(self):
        """ Get the saved End Effector to Palm Translation and Rotation matrices """
        translation_mat = np.zeros((4, 4))
        rotation_mat = np.zeros((4, 4))

        with open(directory + '/EE_to_Palm_Translation_Matrix.csv', newline='') as f:
            reader = csv.reader(f)
            for j, row in enumerate(reader):
                for i, col in enumerate(row):
                    translation_mat[j][i] = float(col)

        with open(directory + '/EE_to_Palm_Rotation_Matrix.csv', newline='') as f:
            reader = csv.reader(f)
            for j, row in enumerate(reader):
                for i, col in enumerate(row):
                    rotation_mat[j][i] = float(col)

        return translation_mat, rotation_mat

    def save_new_joint_angles(self,save_file):
        """ Save joint angle values to csv file """
        with open(directory + save_file, "w") as outfile:
            writer = csv.writer(outfile)
            writer.writerow(robot.joint_angles.values())
            outfile.close()


    def plot_cost(self):
        """ Plot cost value after each angle change from gradient descent """
        steps = np.arange(len(self.cost_values))
        plt.plot(steps, self.cost_values, '-o')
        plt.xlabel("Steps")
        plt.ylabel("Cost value")
        plt.title("Cost value per step using Gradient Descent")
        plt.show()

    def get_current_ee_pose(self):
        """ Get current physical end effector location """
        #self.arm_endpoint = #magic tf call that I can add
        while True:
            try:
                translation, rotation = self.listener.lookupTransform('world_frame', 'palm_frame', rospy.Time()) #j2s7s300_end_effector
                break  # once the transform is obtained move on
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue  # if it fails try again
        point = [translation[0], translation[1], translation[2]]
        self.arm_endpoint = np.array(point)
        # rospy.logerr(self.arm_endpoint)




    def get_goal_ee_pose(self):
        """ Get target end effector location based on arUco markers """
        #self.target_endpoint = #magic tf call that I can add ie the pose of the palm from camera aruco detection
        while True:
            try:
                translation, rotation = self.listener.lookupTransform('world_frame', 'palm_frame_camera', rospy.Time()) # ee_frame_camera_flipped
                break  # once the transform is obtained move on
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue  # if it fails try again
        point = [translation[0], translation[1], translation[2]]
        self.target_endpoint = np.array(point)
        # rospy.logerr(self.target_endpoint)


    def update_angles(self):
        """ Update joint angle values within ROS """
        joint_values = []
        for i in self.joint_names:
            joint_values.append(self.joint_angles[i])
        
        try:
            answer = self.update_arm_angles(joint_values)
        except rospy.ServiceException as e:
            rospy.logwarn('Service call failed for: {0}'.format(e))

        #service_call = # self.update_joints(joint_values)

    def cost(self, curr_xy):
        """
        Calculates the cost between the given point and the target point.
        :param curr_xy: Current endpoint of arm
        :return: dist: Distance between current endpoint and target point
        """
        dist = np.linalg.norm(curr_xy - self.target_endpoint)
        rospy.logerr(self.target_endpoint)
        rospy.logerr(curr_xy)
        rospy.logerr(dist)
        return dist

    def gradient(self, joint_key):
        """
        Calculates the gradient between the current endpoint and the new endpoint from a change in theta.
        :param joint_angle: Current joint component
        :return: Gradient from angle based on the change in endpoint cost
        """
        change = 0.3  # Amount to change theta by
        self.update_angles()
        self.get_current_ee_pose()  # Old endpoint value
        old_cost = self.cost(self.arm_endpoint)  # Old endpoint cost (to target)

        angle_value = self.joint_angles[joint_key] + change  # Change the angle
        self.joint_angles[joint_key] = angle_value

        self.update_angles()
        self.get_current_ee_pose()  # New endpoint value
        new_cost = self.cost(self.arm_endpoint)  # New endpoint cost (to target)

        gradient = (new_cost - old_cost) / change  # Calculate the gradient using the change in cost of x,y

        return gradient

    def reach_gradient(self):
        """Align the robot end point (palm) to the target point using gradient descent"""
        step_size = 0.05
        min_step_size = 0.001
        moved_closer = True
        while_loop_counter = 0
        max_steps = 100
        old_total_cost = 10
        epsilon = 0.005

        # While moved closer and not reached minimum step size
        while moved_closer and step_size > min_step_size:
            while_loop_counter += 1
            # Set a maximum number of steps per change to see progress - used for testing
            if while_loop_counter > max_steps:
                break
            new_total_cost = 0
            text = ""
            i = 0

            # Go through each joint within the arm
            for joint_key, joint_value in self.joint_angles.items():
                # Text to show for each joint change
                text += str(self.joint_names[i]) + " "
                i += 1

                # Old endpoint values
                old_value = joint_value

                # Update joints in ROS with current self.joint_angle values
                self.update_angles()
                self.get_current_ee_pose() # Old endpoint

                # Determine cost from current end effector to target
                old_cost = self.cost(self.arm_endpoint)

                # Gradient of old values
                gradient = self.gradient(joint_key)
                if gradient > 0:  # Determine direction of gradient
                    direction = 1
                else:
                    direction = -1

                # Determine new angle value based on gradient
                self.joint_angles[joint_key] = (old_value - direction * step_size)

                if self.joint_angles[joint_key] < self.joint_min[joint_key]:
                    self.joint_angles[joint_key] = self.joint_min[joint_key]
                elif self.joint_angles[joint_key] > self.joint_max[joint_key]:
                    self.joint_angles[joint_key] = self.joint_max[joint_key]

                # Update joint angle values within ROS and get new endpoint value
                self.update_angles()
                self.get_current_ee_pose()

                # Determine cost from current end effector to target
                new_cost = self.cost(self.arm_endpoint)

                # Determine the cost of
                if new_cost > old_cost:
                    self.joint_angles[joint_key] = old_value
                    new_total_cost += old_cost
                    text += ": No change \n"
                else:
                    text += ": Improved by " + str(direction * step_size) + "\n"
                    new_total_cost += new_cost

            # Display change of each joint through text
            print("Robot part changes: \n", text)
            self.cost_values += [new_total_cost]

            # Check if improved from previous position
            if old_total_cost < new_total_cost:
                step_size -= .01
                moved_closer = False
            else:
                moved_closer = True

            print("abs(old_total_cost - new_total_cost): ", abs(old_total_cost - new_total_cost))
            print("new_total_cost: ", new_total_cost)
            # If changes are less than epsilon, we stop
            if abs(old_total_cost - new_total_cost) < epsilon:
                break
            old_total_cost = new_total_cost

        # Save new joint angle values
        save_file = "/OptimizedAngles.csv"
        print("Saving new joint angles at ", save_file)
        self.save_new_joint_angles(save_file)

if __name__ == '__main__':
    rospy.init_node('gradient_descent')

    print("GRADIENT DESCENT...")
    robot = RobotArm()

    robot.update_angles()
    robot.get_current_ee_pose()
    before_endpoint = robot.arm_endpoint

    # Run Gradient Descent optimization to minimize error between physical and aruco end effector
    robot.reach_gradient()

    robot.update_angles()
    robot.get_current_ee_pose()
    after_endpoint = robot.arm_endpoint

    print("Before ENDPOINT: ", before_endpoint)
    print("After ENDPOINT: ", after_endpoint)
    print("target ENDPOINT: ", robot.target_endpoint)
    print("Before cost: ", robot.cost(before_endpoint))
    print("After cost: ", robot.cost(after_endpoint))

    # Plot cost over time from Gradient Descent algorithm
    print("After, Joint angles: ", robot.joint_angles.items())
    robot.plot_cost()
    print("Done :)")

    rospy.spin()