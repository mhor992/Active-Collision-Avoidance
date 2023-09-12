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
import math
import csv
from moveit_commander import conversions

rospy.init_node('ur5e_physical_movement', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

rospy.loginfo("STARTING")

############################################FUNCTIONS###############################################################

#Checks to see if the robot is moving within the required boundary zones
def WithinBoundary():
    #Manually set boundary constraints
    minimum_x_boundary = 0
    maximum_x_boundary = 0.825
    minimum_y_boundary = -0.550
    maximum_y_boundary = 0.5
    minimum_z_boundary = -0.435
    maximum_z_boundary = 0.370

    current_pose_x = move_group.get_current_pose().pose.position.x
    current_pose_y = move_group.get_current_pose().pose.position.y
    current_pose_z = move_group.get_current_pose().pose.position.z

    if (current_pose_x < minimum_x_boundary) or (current_pose_x > maximum_x_boundary):
        rospy.loginfo("Out of X bounds X:{} Y:{} Z:{}".format(current_pose_x, current_pose_y,current_pose_z))
        return False
    elif (current_pose_y < minimum_y_boundary) or (current_pose_y > maximum_y_boundary):
        rospy.loginfo("Out of Y bounds X:{} Y:{} Z:{}".format(current_pose_x, current_pose_y,current_pose_z))
        return False
    elif (current_pose_z < minimum_z_boundary) or (current_pose_z > maximum_z_boundary):
        rospy.loginfo("Out of Z bounds X:{} Y:{} Z:{}".format(current_pose_x, current_pose_y,current_pose_z))
        return False
    else:
        return True

#Checks to see if the robot has reached the goal position
def WithinTarget():
    threshold = 0.1

    current_pose_x = move_group.get_current_pose().pose.position.x
    current_pose_y = move_group.get_current_pose().pose.position.y
    current_pose_z = move_group.get_current_pose().pose.position.z

    distance = math.sqrt((current_pose_x - target_pose.position.x )**2 + (current_pose_y - target_pose.position.y)**2 + (current_pose_z - target_pose.position.z)**2)
    if distance < threshold:
        rospy.loginfo("Current Position X:{} Y:{} Z:{}".format(current_pose_x, current_pose_y,current_pose_z))
        return True
    else:
        return False

###########################################################################################################################3

#For data saving
position_data = []

# Define first target pose
target_pose = geometry_msgs.msg.Pose()
target_pose.position.x = 0.5
target_pose.position.y = -0.5
target_pose.position.z = 0.5
target_pose.orientation.x = 0.0
target_pose.orientation.y = 0.0
target_pose.orientation.z = 0.0
target_pose.orientation.w = 1.0

# Set first target pose
move_group.set_pose_target(target_pose)
plan = move_group.plan()

move_group.set_max_velocity_scaling_factor(0.1)

plan_test = move_group.plan()
path = plan_test[1].joint_trajectory.points

# Turn check to True
checkBoundary = True
checkFinished = False
# While the robot is within the boundary zones move
plan = move_group.go(wait=False)
while(checkBoundary is True and checkFinished is False):
    # Check position
    #checkBoundary = WithinBoundary()
    checkFinished = WithinTarget()
    current_pose_x = move_group.get_current_pose().pose.position.x
    current_pose_y = move_group.get_current_pose().pose.position.y
    current_pose_z = move_group.get_current_pose().pose.position.z
    position_data.append([current_pose_x, current_pose_y, current_pose_z])
    rospy.sleep(0.25)
# Stop the robot and clear pose targets
move_group.stop()
move_group.clear_pose_targets()

# After your loop, save the position data to a CSV file
with open('position_simulated.csv', 'a', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    
    # Write a header row with column names
    csv_writer.writerow(['X', 'Y', 'Z'])
    
    # Write the position data
    csv_writer.writerows(position_data)

# Print message for first pose completion
rospy.loginfo("First pose completed. Moving to the next pose.")


########################################Reset Position######################################################
# Define Second target pose
target_pose = geometry_msgs.msg.Pose()
target_pose.position.x = 0.5
target_pose.position.y = 0.5
target_pose.position.z = 0.5
target_pose.orientation.x = 0.0
target_pose.orientation.y = 0.0
target_pose.orientation.z = 0.0
target_pose.orientation.w = 1.0

# Set first target pose
move_group.set_pose_target(target_pose)
plan = move_group.plan()

plan = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()

rospy.loginfo("Second pose completed. Moving to the next pose.")