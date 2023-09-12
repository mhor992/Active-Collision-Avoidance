#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
from math import pi
from moveit_commander import conversions
from std_srvs.srv import Empty



rospy.init_node('ur5e_simulation', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

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

rospy.sleep(1)

# Define the first target pose
target_pose = geometry_msgs.msg.Pose()
target_pose.position.x = -0.5
target_pose.position.y = -0.5
target_pose.position.z = 0.5
target_pose.orientation.x = 0.0
target_pose.orientation.y = 0.0
target_pose.orientation.z = 0.0
target_pose.orientation.w = 1.0

# Set first target pose
move_group.set_pose_target(target_pose)
plan = move_group.plan()
plan = move_group.go(wait=True)

# Stop the robot and clear pose targets
move_group.stop()
move_group.clear_pose_targets()

print("Finished moving to the first pose")

# Define the second target pose
target_pose = geometry_msgs.msg.Pose()
target_pose.position.x = -0.5
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

# Stop the robot and clear pose targets
move_group.stop()
move_group.clear_pose_targets()

print("Finished moving to the second pose")
