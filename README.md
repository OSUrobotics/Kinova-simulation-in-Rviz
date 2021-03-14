# Kinova-simulation-in-Rviz

This code reconstructs the environment of the Kinova arm simulation in Rviz.  

## Kinova_scrpits

In the folder, it contain two other folder "src" and "visualization"

* src
	* In this folder, we mainly want to run the "arm_calibration.py" to get the calibaration matrixs.

* visualization

	* In this folder, it contain the launch file, and test data. It is worth noting that the difference from the previous one is that the path planning file in the launch file is changed to "kinova_path_planning_finger.py". The previous one "kinova_path_planning.py" file cannot move the kinova finger.

## new_meshes

In this folder, add the Aruco Marker model of two finger(proximal and distal). Also redesign the finger and save to finger_marker_Dist1 or 2, finger_marker_Prox1 or 2, and keep the original finger design file.

## new_urdf

In this folder, rewrite all urdf files of j2s7s300 model. Mainly modify the finger part.

## Run Code

First, add the kinova_scrpits folder to the kinova_ws/src/kinova_ros/

Then, move the files in the new_mesh and new_urdf to the kinova_ws/src/kinova_ros/kinova_description/meshs or urdf and replace the old one. 


* Once all files are in the correct location make sure to catkin_make in the root directory.
	* cd ~/kinova_ws
	* catkin_make
* Open two other terminals to the workspace and source all three of them.
	* cd ~/kinova_ws
	* source devel/setup.bash
* In the first terminal run the kinova virtual launch file(if you do not contact with real kinova arm):
	* roslaunch j2s7s300_moveit_config j2s7s300_virtual_robot_demo.launch
* Once Rviz opens run the following in the second terminal:
	* roslaunch kinova_scripts rob514_visual.launch
*	In the third terminal run the following:
	* rosrun kinova_scripts joint_angles.py <#>
		* <#> : The file number, ie 9 for the example data

 
