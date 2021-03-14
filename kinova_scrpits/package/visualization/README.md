# Visualization of the Kinova Arm

This code recreates the overhead webcamera image using Rviz and moveIt.
* Inputs:
	* Recorded joint angles of the arm
	* Translation and Rotation matrices from arm calibration
	* Transform matrix to go from world frame to camera frame
	* Transform matrices for all the ArUco markers, in world frame
	* Transform matrix to go from end effector link to the palm ArUco marker
* Outputs:
	* Displays the arm, where it "thinks" it is, with a red marker on the palm to represent the ArUco marker
	* Displays a Green plane for the table top which is the world frame, and red markers at each corner and center to show the respective ArUco Marker
	* Displays blue markers to represent the ArUco markers location as detected with the camera
  
The different between the blue and red marker on the palm shows the error between what the web camera "sees" and where the arm "thinks" it is. 

## Run Code

This code will need to be added to their respective spots in the kinova_ws/src/kinova-ros/kinova_scripts/ package.
also edit the package.xml and CMakeList.txt files to include the service calls as seen in the ones provided.

If you want to change the file locations make sure to go through each script to modify the path.

* Once all files are in the correct location make sure to catkin_make in the root directory.
	* cd ~/kinova_ws
	* catkin_make
* Open 3 terminals to the workspace and source them.
	* cd ~/kinova_ws
	* source devel/setup.bash
* In the first terminal run the kinova virtual launch file:
	* roslaunch j2s7s300_moveit_config j2s7s300_demo.launch
* Once Rviz opens run the following in the second terminal:
	* roslaunch kinova_scripts rob514_visual.launch
* In Rviz load the saved setup:
	* located ~/kinova_ws/src/kinova-ros/kinova_scripts/src/rob514_finalVisual.rviz
*	In the third terminal run the following:
	* rosrun kinova_scripts joint_angles.py <#>
		* <#> : The file number, ie 9 for the example data

Rviz now should be displaying the recreation of what is seen in the webcam image.  If you want to change the data file just run the last command again with the number associated with the file.
