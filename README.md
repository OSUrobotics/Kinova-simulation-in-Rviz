# Kinova-simulation-in-Rviz

This code reconstructs the environment of the Kinova arm simulation in Rviz.  

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

 
