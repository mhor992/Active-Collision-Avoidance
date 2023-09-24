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
from std_msgs.msg import Float64MultiArray


rospy.init_node('ur5e_physical_movement', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.set_planner_id("RRTConnectkConfigDefault")

# set tolerance
move_group.set_goal_position_tolerance(0.05)
#move_group.set_planning_time(25.0)

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
back_wall.pose.position.x = 0.5
back_wall.pose.position.y = 0.1
back_wall.pose.position.z = 0.5
#scene.add_box("back_wall", back_wall, (0.3, 2, 1))

left_wall = geometry_msgs.msg.PoseStamped()
left_wall.header.frame_id = robot.get_planning_frame()
left_wall.pose.position.x = 0
left_wall.pose.position.y = 0.75
left_wall.pose.position.z = 0.5
#scene.add_box("left_wall", left_wall, (1.7, 0.1, 1))

right_wall = geometry_msgs.msg.PoseStamped()
right_wall.header.frame_id = robot.get_planning_frame()
right_wall.pose.position.x = 0
right_wall.pose.position.y = -0.75
right_wall.pose.position.z = 0.5
#scene.add_box("right_wall", right_wall, (1.7, 0.1, 1))

bottom_wall = geometry_msgs.msg.PoseStamped()
bottom_wall.header.frame_id = robot.get_planning_frame()
bottom_wall.pose.position.x = 0
bottom_wall.pose.position.y = 0
bottom_wall.pose.position.z = -0.3
#scene.add_box("bottom_wall", bottom_wall, (2, 2, 0.1))

top_wall = geometry_msgs.msg.PoseStamped()
top_wall.header.frame_id = robot.get_planning_frame()
top_wall.pose.position.x = 0
top_wall.pose.position.y = 0
top_wall.pose.position.z = 1.1
#scene.add_box("top_wall", top_wall, (2, 2, 0.1))

rospy.sleep(1)

####################################SAFETY FUNCTIONS###############################################################

#Checks to see if the robot is moving within the required boundary zones
def WithinBoundary():
    #Manually set boundary constraints
    minimum_x_boundary = -0.7
    maximum_x_boundary = -0.11
    minimum_y_boundary = -0.5
    maximum_y_boundary = 0.490
    minimum_z_boundary = 0.14
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
    threshold = 0.05

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
    move_group.set_max_velocity_scaling_factor(0.04)
    # Turn check to True
    checkBoundary = True
    checkFinished = False
    checkEmergencyStop = False

    #tester = move_group.get_current_joint_values()

    if (plan):
        plan = move_group.go(wait=False)
        while(checkBoundary is True and checkFinished is False and checkEmergencyStop is False):
            # Check position
            checkBoundary = WithinBoundary()
            checkFinished = WithinTarget(target_pose)
            checkEmergencyStop = CheckEmergencyStop()
            AppendData()
            rospy.sleep(0.25)
        # Stop the robot and clear pose targets
        move_group.stop()
        move_group.clear_pose_targets()

def PoseMaker(x, y, z):
    #Generates and returns a target pose with x y z cartesian coordinates as an input
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    target_pose.orientation.x = move_group.get_current_pose().pose.orientation.x
    target_pose.orientation.y = move_group.get_current_pose().pose.orientation.y
    target_pose.orientation.z = move_group.get_current_pose().pose.orientation.z
    target_pose.orientation.w = move_group.get_current_pose().pose.orientation.w

    return target_pose

def MoveToCartesian(x, y ,z):
    #Moves to a target position using x y z cartesian coordinates as an input.
    target_pose = PoseMaker(x, y, z)
    MoveToPosition(target_pose)

def CheckEmergencyStop():
    threshold = 0
    #Checks to see if any robot joints are close to collision with human joints
    #Currently only for robot end effector - Ideas => Manual calculation of inverse kinematics for calculating other joint positions
    current_pose_x = move_group.get_current_pose().pose.position.x
    current_pose_y = move_group.get_current_pose().pose.position.y
    current_pose_z = move_group.get_current_pose().pose.position.z

    joint_pos = GetJointPositions()
    for joints in joint_pos:
        distance = math.sqrt(((current_pose_x - joints[0])**2) + ((current_pose_y - joints[1])**2) + ((current_pose_z - joints[2])**2))
        if distance < threshold:
            rospy.loginfo("Current Position X:{} Y:{} Z:{}".format(current_pose_x, current_pose_y,current_pose_z))
            rospy.loginfo("EMERGENCY STOP")
            return True
        
    return False

def HandFollowing():

    minimum_x_boundary = -0.7
    maximum_x_boundary = -0.11
    minimum_y_boundary = -0.5
    maximum_y_boundary = 0.490
    minimum_z_boundary = 0.14
    maximum_z_boundary = 0.7
    while 1:
        try:
            lhx = joint_coords[24]
            lhy = joint_coords[25]
            lhz = joint_coords[26]
            print("left hand is at: ", lhx,lhy,lhz)
            goal.position.x = min(maximum_x_boundary, max(minimum_x_boundary, lhx+0.3))
            goal.position.y = min(maximum_y_boundary, max(minimum_y_boundary, lhy))
            goal.position.z = min(maximum_z_boundary, max(minimum_z_boundary, lhz))
            goal.orientation.x = move_group.get_current_pose().pose.orientation.x
            goal.orientation.y = move_group.get_current_pose().pose.orientation.y
            goal.orientation.z = move_group.get_current_pose().pose.orientation.z
            goal.orientation.w = move_group.get_current_pose().pose.orientation.w
            MoveToPosition(goal)
            rospy.sleep(0.5)
        except:
            pass

def separation(p1,p2):
    dX = p2[0]-p1[0]
    dY = p2[1]-p1[1]
    dZ = p2[2]-p1[2]
    sep = ((dX)**2+(dY)**2+(dZ)**2)**0.5
    return sep #returns distance between points and angle in radians from point 1

def cp_cost(point, joints, weight):
    # calculates the cost associated with a point due to surrounding joints
    cost = 0
    for joint in joints:
        cost = weight/separation(point, joint)**2 + cost
    return cost

def next_move(r, joints, target): #r is range for each movement, joint is array of joints [[x1, y1, z1], [x2,y2,z2]...], target is coord also
    x = move_group.get_current_pose().pose.position.x
    y = move_group.get_current_pose().pose.position.y
    z = move_group.get_current_pose().pose.position.z
    poses = [[x,y,z],[x+r,y,z],[x-r,y,z],[x+r,y+r,z],[x+r,y-r,z],[x-r,y+r,z],[x-r,y-r,z],[x+r,y,z+r],
    [x+r,y,z-r],[x-r,y,z+r],[x-r,y,z-r],[x+r,y+r,z+r],[x+r,y+r,z-r],[x+r,y-r,z+r],[x+r,y-r,z-r],
    [x-r,y+r,z+r],[x-r,y+r,z-r], [x-r,y-r,z+r],[x-r,y-r,z-r], [x,y+r,z],[x,y-r,z],[x,y+r,z+r],
    [x,y+r,z-r],[x,y-r,z+r],[x,y-r,z-r],[x,y,z+r],[x,y,z-r]]

    if separation(poses[0], target)<= r:
        target_weight = 0
    else:
        target_weight = 1
    #    return target 

    min_cost = 1000000
    target_weight = 2
    joint_weight = 1
    for pose in poses:
        cost = cp_cost(pose, joints, joint_weight) - (target_weight/separation(pose, target)) #change value to weight target more heavily / less heavily, should this be squared?
        if cost < min_cost:
            min_cost = cost
            min_pose = pose
    
    return min_pose # the most cost effective pose to move to

############################################### CALLBACK FUNCTIONS (FOR SUBCRIBER) ######################################################
def joint_coordinates_callback(data):
    # This function will be called whenever a message is received on the /joint_coordinates topic
    # The received joint coordinates are stored in 'data'
    # joint coord global
    global joint_coords

    # Extract and print the joint coordinates
    joint_coords = data.data
    #print("Received joint coordinates:", joint_coords)

    # doing some global stuff

################################################ SENSOR PROCESSING#########################################################################
def GetJointPositions():

    try:
        joint_pos = [None] * 32

        for joint in range(0, 32):
            index_length = 3 * joint
            joint_pos[joint] = [1, 2, 3]
            joint_pos[joint] = [joint_coords[0 + index_length], joint_coords[1 + index_length], joint_coords[2 + index_length]]

        return joint_pos    
    except:
        joint_pos = [[999, 999, 999]]
        return joint_pos

###########################################################################################################################################

# Create a subscriber for the /joint_coordinates topic
rospy.Subscriber('/joint_coordinates', Float64MultiArray, joint_coordinates_callback)

#Create a position saving variable
position_data = []

print("check")

#Is a global until FSM is set up
checkEmergencyStop = False

goal = geometry_msgs.msg.Pose()

'''while 1:  
    l_hand = [joint_coords[24], joint_coords[25], joint_coords[26]]
    hand_pos = geometry_msgs.msg.PoseStamped()
    hand_pos.header.frame_id = robot.get_planning_frame()
    hand_pos.pose.position.x = l_hand[0]
    hand_pos.pose.position.y = l_hand[1]
    hand_pos.pose.position.z = l_hand[2]
    scene.add_box("left hand", hand_pos, (0.1, 0.1, 0.1))
    sep = separation(l_hand, [-0.3,0.2,0.2])
    print("Left hand is at:", l_hand[0], l_hand[1], l_hand[2])
    print("Distance from target:", sep)
    rospy.sleep(0.5)

    next_pos = next_move(0.1, [l_hand], [-0.56,-0.2,0.21])
    print("Moving to:", next_pos[0], next_pos[1], next_pos[2])
    pose_1 = geometry_msgs.msg.Pose()
    pose_1.position.x = next_pos[0]
    pose_1.position.y = next_pos[1]
    pose_1.position.z = next_pos[2]
    pose_1.orientation.x = move_group.get_current_pose().pose.orientation.x
    pose_1.orientation.y = move_group.get_current_pose().pose.orientation.y
    pose_1.orientation.z = move_group.get_current_pose().pose.orientation.z
    pose_1.orientation.w = move_group.get_current_pose().pose.orientation.w
    MoveToPosition(pose_1)
    print("Finished moving to pose")
    rospy.sleep(0.5)'''

target = [-0.56, -0.2, 0.2]




while 1:
    try:
        
        l_hand = [joint_coords[24], joint_coords[25], joint_coords[26]]
        print("trying")
        hand_pos = geometry_msgs.msg.PoseStamped()
        hand_pos.header.frame_id = robot.get_planning_frame()
        hand_pos.pose.position.x = l_hand[0]
        hand_pos.pose.position.y = l_hand[1]
        hand_pos.pose.position.z = l_hand[2]
        scene.add_box("left hand", hand_pos, (0.1, 0.1, 0.1))
        
        print("Left hand is at:", l_hand[0], l_hand[1], l_hand[2])

        next_pos = next_move(0.1, [l_hand], target)
        sep = separation(l_hand, target)
        print("Distance from target:", sep)
        print("Moving to:", next_pos[0], next_pos[1], next_pos[2])
        
        pose_1 = geometry_msgs.msg.Pose()
        pose_1.position.x = next_pos[0]
        pose_1.position.y = next_pos[1]
        pose_1.position.z = next_pos[2]
        pose_1.orientation.x = move_group.get_current_pose().pose.orientation.x
        pose_1.orientation.y = move_group.get_current_pose().pose.orientation.y
        pose_1.orientation.z = move_group.get_current_pose().pose.orientation.z
        pose_1.orientation.w = move_group.get_current_pose().pose.orientation.w
        MoveToPosition(pose_1)
        print("Finished moving to pose")
        rospy.sleep(0.5)
    except:
        pass
    


#HandFollowing()'''

#Notes
#z 0.45 limit
'''

goal.position.x = -0.56
goal.position.y = 0.2
goal.position.z = 0.21
goal.orientation.x = move_group.get_current_pose().pose.orientation.x
goal.orientation.y = move_group.get_current_pose().pose.orientation.y
goal.orientation.z = move_group.get_current_pose().pose.orientation.z
goal.orientation.w = move_group.get_current_pose().pose.orientation.w

# Define the first target pose
pose_1 = geometry_msgs.msg.Pose()
pose_1.position.x = -0.56
pose_1.position.y = 0.20
pose_1.position.z = 0.21
pose_1.orientation.x = move_group.get_current_pose().pose.orientation.x
pose_1.orientation.y = move_group.get_current_pose().pose.orientation.y
pose_1.orientation.z = move_group.get_current_pose().pose.orientation.z
pose_1.orientation.w = move_group.get_current_pose().pose.orientation.w

# Define the second target pose
pose_2 = geometry_msgs.msg.Pose()
pose_2.position.x = -0.56
pose_2.position.y = -0.20
pose_2.position.z = 0.21
pose_2.orientation.x = move_group.get_current_pose().pose.orientation.x
pose_2.orientation.y = move_group.get_current_pose().pose.orientation.y
pose_2.orientation.z = move_group.get_current_pose().pose.orientation.z
pose_2.orientation.w = move_group.get_current_pose().pose.orientation.w

# Define the third target pose
pose_3 = geometry_msgs.msg.Pose()
pose_3.position.x = -0.56
pose_3.position.y = -0.40
pose_3.position.z = 0.21
pose_3.orientation.x = move_group.get_current_pose().pose.orientation.x
pose_3.orientation.y = move_group.get_current_pose().pose.orientation.y
pose_3.orientation.z = move_group.get_current_pose().pose.orientation.z
pose_3.orientation.w = move_group.get_current_pose().pose.orientation.w

# Define the fourth target pose
pose_4 = geometry_msgs.msg.Pose()
pose_4.position.x = -0.56
pose_4.position.y = -0.40
pose_4.position.z = 0.40
pose_4.orientation.x = move_group.get_current_pose().pose.orientation.x
pose_4.orientation.y = move_group.get_current_pose().pose.orientation.y
pose_4.orientation.z = move_group.get_current_pose().pose.orientation.z
pose_4.orientation.w = move_group.get_current_pose().pose.orientation.w

MoveToPosition(pose_1)
print("Finished moving to the first pose")
rospy.sleep(0.5)

MoveToPosition(pose_2)
print("Finished moving to the second pose")
rospy.sleep(0.5)

MoveToPosition(pose_3)
print("Finished moving to the third pose")
rospy.sleep(0.5)

MoveToPosition(pose_4)
print("Finished moving to the fourth pose")

# After your loop, save the position data to a CSV file
with open('position_physical.csv', 'a', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    
    # Write a header row with column names
    csv_writer.writerow(['X', 'Y', 'Z'])
    
    # Write the position data
    csv_writer.writerows(position_data)'''
