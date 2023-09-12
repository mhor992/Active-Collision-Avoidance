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

rospy.loginfo("Start")

#Refresh the planning scene
scene.remove_world_object("back_wall")
scene.remove_world_object("left_wall")
scene.remove_world_object("right_wall")
scene.remove_world_object("bottom_wall")
scene.remove_world_object("top_wall")

rospy.sleep(1)

#Add the collision zones
back_wall = geometry_msgs.msg.PoseStamped()
back_wall.header.frame_id = robot.get_planning_frame()
back_wall.pose.position.x = 0.4
back_wall.pose.position.y = 0
back_wall.pose.position.z = 0.5
scene.add_box("back_wall", back_wall, (0.1, 2, 1))

left_wall = geometry_msgs.msg.PoseStamped()
left_wall.header.frame_id = robot.get_planning_frame()
left_wall.pose.position.x = 0
left_wall.pose.position.y = 0.75
left_wall.pose.position.z = 0.5
scene.add_box("left_wall", left_wall, (1.7, 0.1, 1))

right_wall = geometry_msgs.msg.PoseStamped()
right_wall.header.frame_id = robot.get_planning_frame()
right_wall.pose.position.x = 0
right_wall.pose.position.y = -0.75
right_wall.pose.position.z = 0.5
scene.add_box("right_wall", right_wall, (1.7, 0.1, 1))

bottom_wall = geometry_msgs.msg.PoseStamped()
bottom_wall.header.frame_id = robot.get_planning_frame()
bottom_wall.pose.position.x = 0
bottom_wall.pose.position.y = 0
bottom_wall.pose.position.z = -0.3
scene.add_box("bottom_wall", bottom_wall, (2, 2, 0.1))

top_wall = geometry_msgs.msg.PoseStamped()
top_wall.header.frame_id = robot.get_planning_frame()
top_wall.pose.position.x = 0
top_wall.pose.position.y = 0
top_wall.pose.position.z = 0.8
scene.add_box("top_wall", top_wall, (2, 2, 0.1))

rospy.sleep(1)

####################################SAFETY FUNCTIONS###############################################################

#Checks to see if the robot is moving within the required boundary zones
def WithinBoundary():
    #Manually set boundary constraints
    minimum_x_boundary = -0.7
    maximum_x_boundary = -0.11
    minimum_y_boundary = -0.5
    maximum_y_boundary = 0.490
    minimum_z_boundary = 0.18
    maximum_z_boundary = 0.7

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
def WithinTarget(target_pose):
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
############################################### DATA STORAGE FUNCTIONS ######################################################
def AppendData():
    current_pose_x = move_group.get_current_pose().pose.position.x
    current_pose_y = move_group.get_current_pose().pose.position.y
    current_pose_z = move_group.get_current_pose().pose.position.z
    position_data.append([current_pose_x, current_pose_y, current_pose_z])
############################################### MOVEMENT FUNCTIONS ######################################################
def MoveToPosition(target_pose):

    move_group.set_pose_target(target_pose)
    plan = move_group.plan()
    move_group.set_max_velocity_scaling_factor(0.03)
    # Turn check to True
    checkBoundary = True
    checkFinished = False

    plan = move_group.go(wait=False)
    while(checkBoundary is True and checkFinished is False):
        # Check position
        checkBoundary = WithinBoundary()
        checkFinished = WithinTarget(target_pose)
        AppendData()
        rospy.sleep(0.25)
    # Stop the robot and clear pose targets
    move_group.stop()
    move_group.clear_pose_targets()
#########################################################################################################################

#For data saving
position_data = []

# Define the first target pose
pose_1 = geometry_msgs.msg.Pose()
pose_1.position.x = -0.56
pose_1.position.y = 0.1
pose_1.position.z = 0.21
pose_1.orientation.x = move_group.get_current_pose().pose.orientation.x
pose_1.orientation.y = move_group.get_current_pose().pose.orientation.y
pose_1.orientation.z = move_group.get_current_pose().pose.orientation.z
pose_1.orientation.w = move_group.get_current_pose().pose.orientation.w

# Define the second target pose
pose_2 = geometry_msgs.msg.Pose()
pose_2.position.x = -0.56
pose_2.position.y = -0.1
pose_2.position.z = 0.21
pose_2.orientation.x = move_group.get_current_pose().pose.orientation.x
pose_2.orientation.y = move_group.get_current_pose().pose.orientation.y
pose_2.orientation.z = move_group.get_current_pose().pose.orientation.z
pose_2.orientation.w = move_group.get_current_pose().pose.orientation.w

MoveToPosition(pose_1)
print("Finished moving to the first pose")

MoveToPosition(pose_2)
print("Finished moving to the second pose")

# After your loop, save the position data to a CSV file
with open('position_physical.csv', 'a', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    
    # Write a header row with column names
    csv_writer.writerow(['X', 'Y', 'Z'])
    
    # Write the position data
    csv_writer.writerows(position_data)
