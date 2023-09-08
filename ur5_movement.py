#!/usr/bin/env python3
# This makes the ur5 move to a position 1, wait 2 seconds and then move to position 2.
# To execute this code ensure MoveIt!, Universal Robots, Ros, and this directory is sourced.
# Ensure this code is executable by running chmod +x "NameOfCode.py"
# 
# roslaunch ur_gazebo ur5_bringup.launch
# roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=true
# roslaunch ur5_moveit_config moveit_rviz.launch
# ./ur5_movement.py

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
from math import pi
from moveit_commander import conversions

rospy.init_node('ur5_movement', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Define first target pose
target_pose_1 = geometry_msgs.msg.Pose()
target_pose_1.position.x = 0.5
target_pose_1.position.y = 0.0
target_pose_1.position.z = 0.9
target_pose_1.orientation.x = 0.0
target_pose_1.orientation.y = 0.0
target_pose_1.orientation.z = 0.0
target_pose_1.orientation.w = 1.0

# Set first target pose
move_group.set_pose_target(target_pose_1)
testplan = move_group.plan()
plan_1 = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()

# Print message for first pose completion
rospy.loginfo("First pose completed. Moving to the next pose.")

# Pause for 2 seconds
rospy.sleep(2)

# Define second target pose
target_pose_2 = geometry_msgs.msg.Pose()
target_pose_2.position.x = 0.8
target_pose_2.position.y = 0.0
target_pose_2.position.z = 0.0
target_pose_2.orientation.x = 0.0
target_pose_2.orientation.y = 0.0
target_pose_2.orientation.z = 0.0
target_pose_2.orientation.w = 1.0

# Set second target pose
move_group.set_pose_target(target_pose_2)
plan_2 = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()

# Print message for second pose completion
rospy.loginfo("Second pose completed.")
