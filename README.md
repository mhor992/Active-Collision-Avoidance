# Safe Human-Robot Collaboration via Active Collision Avoidance
#### Github Repository: https://github.com/mhor992/Active-Collision-Avoidance.git

 This document outlines the entire process for implementing the Safe Human-Robot Collaboration via Avoidance Collision Avoidance robotic system.
 
 The robotic system enables the UR5e robotic model developed by Universal Robotics to avoid dynamic human movements that may result in collision.
 
 The system utilises Azure Kinect Depth sensors used for body tracking, MoveIt! to command the UR5e and a linux operating system linked with ROS.
 
## Implementation
 
 This system will only work with an Ubuntu Linux based system that does not use a virtual machine.
 
### Physical Setup

 Create a physical setup as shown in Experimental Setup => Experimental_Setup_1
 ![Experimental_Setup](https://github.com/mhor992/Active-Collision-Avoidance/assets/87846690/99af9676-9530-4cc6-88ee-9b53f41bac15)

 - This uses a UR5e robot with a workbench as its operating boundary zone
 - The setup utilises Azure Kinect cameras as seen on the ceiling. Setup for an ariel view across the entire workspace
 
### Software Dependency Installations
 - Install the following installations in seperate catkin workspaces and source from all
 
 1. Install Visual Studio Code
 Install for Ubuntu Linux
 https://code.visualstudio.com/download

 2. Install ROS Noetic.
 Follow the official documentation setup as per below
 http://wiki.ros.org/noetic/Installation/Ubuntu
 
 3. Install MoveIt! and build it from source
 Follow the official documentation setup as per below
 https://moveit.ros.org/install/source/

 4. Install Universal Robots - For ROS Noetic, build from source
 Follow the official documentation setup as per below
 https://github.com/ros-industrial/universal_robot
 
 5. Install the Universal_Robots_ROS_Driver
 Follow the official documentation setup as per below
 https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
 
 6. Install PyKinect Azure
 Follow the official documentation setup as per below
 https://github.com/ibaiGorordo/pyKinectAzure
 
 7. Install Visual Kinematics
 Follow the official documentation setup as per below
 https://github.com/dbddqy/visual_kinematics
 
 8. Install Active-Collision-Avoidance - This reposition
 #### Follow instructions as per below
 
	 1. Install from source
	 
  		cd $HOME
  		mkdir -p Safe_HRC/src
  		cd Safe_HRC/src
  		git clone https://github.com/mhor992/Active-Collision-Avoidance.git
  		cd ..
  		catkin build
  		source devel/setup.bash
 
	2. Alternatively, download zip file and extract
	
 9. Install remaining dependencies
	 pip install opencv-python
	 pip install numpy
 
 
## Modifications
 
 1. Modify robotic joint kinematics
	- Replace the contents of universal_robot/ur_description/config/ur5e/joint_limits.yaml with the contents found in Experimental_Setup/joint_limits.yaml
	- This replaces the joint limits with the corrected ones
	
 2. Ensure calibration of the depth camera
 #### Follow https://github.com/UniversalRobots/Universal_Robots_ROS_Driver to establish a connection with the PC and UR5e	
	 #Calibrate robot 
	 	roslaunch ur_calibration calibration_correction.launch \
				robot_ip:=[ROBOT_IP] target_filename:="${HOME}/my_robot_calibration.yaml"
    
	#Connect to robot
		roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=[ROBOT_IP] \
			kinematics_config:=$HOME/my_robot_calibration.yaml

	#Connect to UR5e using MoveIt!
		roslaunch ur5e_moveit_config moveit_planning_execution.launch
		
	- Using Visual Studio run body_tracking_ros_publisher and get_current_position simulatenously
	- Using the teach pendant move the robot and modify the transform_coords(x,y,z) function to calibrate the camera and robot to match
	
 4. Modify the boundary zones
	- Using get_current_pos and the teach pendant to move the robot, modify the maximum and minimum boundary zones for each x,y,z plane
	
## Running the robotic system
	# For physical setup
	Run the following code in a new terminal below
		roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=[ROBOT_IP] \
			kinematics_config:=$HOME/my_robot_calibration.yaml
		roslaunch ur5e_moveit_config moveit_planning_execution.launch
		roslaunch ur5e_moveit_config moveit_rviz.launch

	
	Open visual studio and run body_tracking_ros_publisher.py and then UR5e_dynamic_obstacle_avoidance.py
	
	# For simulation setup 
	roslaunch ur_gazebo ur5e_bringup.launch
	roslaunch ur5e_moveit_config moveit_planning_execution.launch sim:=true
	roslaunch ur5e_moveit_config moveit_rviz.launch

	
	#Change target positions and locations by changing the target1 and target2 variables
