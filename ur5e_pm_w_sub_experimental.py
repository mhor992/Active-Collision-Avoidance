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
import numpy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
import math
import csv
from visual_kinematics.RobotSerial import *
from moveit_commander import conversions
from std_msgs.msg import Float64MultiArray


rospy.init_node('ur5e_physical_movement', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.set_planner_id("RRTConnectkConfigDefault")

# set tolerance & velocity factor
move_group.set_goal_position_tolerance(0.01)
move_group.set_max_velocity_scaling_factor(0.08)
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
scene.add_box("back_wall", back_wall, (0.3, 2, 1))

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
bottom_wall.pose.position.z = -0.1
scene.add_box("bottom_wall", bottom_wall, (2, 2, 0.1))

top_wall = geometry_msgs.msg.PoseStamped()
top_wall.header.frame_id = robot.get_planning_frame()
top_wall.pose.position.x = 0
top_wall.pose.position.y = 0
top_wall.pose.position.z = 1.1
scene.add_box("top_wall", top_wall, (2, 2, 0.1))

global minimum_x_boundary, maximum_x_boundary, minimum_y_boundary, maximum_y_boundary, minimum_z_boundary, maximum_z_boundary
minimum_x_boundary = -0.8
maximum_x_boundary = -0.11
minimum_y_boundary = -0.5
maximum_y_boundary = 0.490
minimum_z_boundary = 0.14
maximum_z_boundary = 0.7


rospy.sleep(1)

####################################SAFETY FUNCTIONS###############################################################

#Checks to see if the robot is moving within the required boundary zones
def WithinBoundary():
    #Manually set boundary constraints

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
    current_pose = current_position
    position_data.append(current_pose)
############################################### MOVEMENT FUNCTIONS ######################################################
def boundary_exception():
    print("boundary exception triggered")
    cp = current_position()
    bound_move = cp

    if cp[0] <= minimum_x_boundary:
        bound_move = PoseMaker(minimum_x_boundary + 0.05, cp[1], cp[2])
    elif cp[0] >= maximum_x_boundary:
        bound_move = PoseMaker(maximum_x_boundary - 0.05, cp[1], cp[2])
    elif cp[1] <= minimum_y_boundary:
        bound_move = PoseMaker(cp[0], minimum_y_boundary + 0.05, cp[2])
    elif cp[1] >= maximum_y_boundary:
        bound_move = PoseMaker(cp[0], maximum_y_boundary - 0.05, cp[2])
    elif cp[2] <= minimum_z_boundary:
        bound_move = PoseMaker(cp[0], cp[1], minimum_z_boundary + 0.05)
    elif cp[2] >= maximum_z_boundary:
        bound_move = PoseMaker(cp[0], cp[1], maximum_z_boundary - 0.05)

    move_group.set_pose_target(bound_move)    
    
    while plan is False:
        print("planning boundary move")        
        joints_list = update_joints(joint_coords)    
        plan = move_group.plan()
    print("moving to:", bound_move)
    move_group.go(wait=True)   
    
def saturate_bounds(x, y, z):
    if x <= minimum_x_boundary:
        x = minimum_x_boundary + 0.05
    elif x >= maximum_x_boundary:
        x = maximum_x_boundary - 0.05
    
    if y <= minimum_y_boundary:
        y = minimum_y_boundary + 0.05
    elif y >= maximum_y_boundary:
        y = maximum_y_boundary - 0.05
    
    if z <= minimum_z_boundary:
        z = minimum_z_boundary + 0.05
    elif z >= maximum_z_boundary:
        z = maximum_z_boundary - 0.05

    return [x, y, z]

def escape():
    current_pose = current_position()
    joints_list = update_joints(joint_coords)
    print("in escape")
    #rospy.sleep(1.0)
    min_sep = 1000
    plan = False
            
    for joint in joints_list:
        sep = separation(current_pose, joint)
        if sep <= min_sep:
            min_joint = joint
            min_sep = sep

    if  min_sep <= 0.4: # if joints is within 0.2 of tcp
        print ("Separation is:", min_sep)
        print("Current pose is:", current_pose, "joint is", min_joint)
        f = 1.5 # wil go x (0.3) from closest joint
        v = numpy.subtract(current_pose, min_joint) # numpy.subtract
        v_scaled = [f * v[0], f * v[1], f * v[2]]
        sat_p = saturate_bounds(min_joint[0] + v_scaled[0], min_joint[1] + v_scaled[1], min_joint[2] + v_scaled[2])
        retreat = PoseMaker(sat_p[0], sat_p[1], sat_p[2])
        print("Retreating to", retreat)
        move_group.set_pose_target(retreat)
        while plan is False:        
            joints_list = update_joints(joint_coords)    
            plan = move_group.plan()
        move_group.go(wait=True)
        #rospy.sleep(1.0)

def current_position():
    current_pose_x = move_group.get_current_pose().pose.position.x
    current_pose_y = move_group.get_current_pose().pose.position.y
    current_pose_z = move_group.get_current_pose().pose.position.z
    current_pose = [current_pose_x, current_pose_y, current_pose_z]

    return current_pose

# def MoveToPosition(target_pose):
       
#     if CheckEmergencyStop():
#         escape()

    # # Turn check to True
    # in_bound = True
    # in_target = False
    # in_emergency = False   

#     while in_target is False:
  
#         joints_list = update_joints(joint_coords)
#         plan = False
#         target_xyz = [target_pose.position.x, target_pose.position.y, target_pose.position.z]
#         next_position = next_move(joints_list, target_xyz, ws_array)
#         next_pose = PoseMaker(next_position[0], next_position[1], next_position[2])
                
#         move_group.set_pose_target(next_pose)
#         in_bound = WithinBoundary()
#         in_emergency = CheckEmergencyStop()
#         in_emergency_non_tcp = CheckNonTcp()
        
#         if (in_bound is True and in_target is False and in_emergency is False):
#             while plan is False:
#                 print("planning movement to:", next_pose)
#                 joints_list = update_joints(joint_coords)
#                 plan = move_group.plan()
#             move_group.go(wait=False)

#         while(in_bound is True and in_target is False and in_emergency is False):
#             # Check position
#             # rospy.sleep(0.1)
            # in_bound = WithinBoundary()
            # in_target = WithinTarget(next_pose)
            # in_emergency = CheckEmergencyStop()
            # in_emergency_non_tcp = CheckNonTcp()
#             #AppendData()
#             # Stop the robot and clear pose targets
        
#         move_group.stop() # something has happened, stop the robot
#         move_group.clear_pose_targets()

#         in_target = WithinTarget(target_pose)

#         while in_emergency_non_tcp is True:
#             in_emergency_non_tcp = CheckNonTcp()
#             # rospy.sleep(0.1)

#         if in_emergency is True:
#             escape()
#             print("escaping")
#             in_emergency = False

#         elif in_bound is False:
#             print("out of bounds") # update this later
#             boundary_exception()
#             in_bound = True     
# 
def MoveToPosition(finalPose):
    # Turn checks to True
    in_bound = True
    in_target = False
    in_final_target = False
    in_emergency = False 

    while in_final_target is False:
        #Run while not in final pose
        in_final_target = WithinTarget(finalPose)
        target_poses = planPath(finalPose)

        #Run while all conditions are met
        while in_bound is True and in_target is False and in_emergency is False:

            current_target_pose = PoseMaker(target_poses[0][0], target_poses[0][1], target_poses[0][2])
            move_group.set_pose_target(current_target_pose)

            plan = move_group.plan()
            if plan:
                move_group.go(wait=False)

                #Check boundaries and target pose
                in_bound = WithinBoundary()
                in_target = WithinTarget(current_target_pose)

                #Check emergency stop
                in_emergency = CheckEmergencyStop()

                #Could add in Check collision for path
                if CheckPathCollision(target_poses) is True:
                    #Replan the path
                    target_poses = planPath(finalPose)
                #Or else continue through the loop
        
        #Occurs when conditions are not met, stop the robot
        move_group.stop() # something has happened, stop the robot
        move_group.clear_pose_targets()

        #Perform checks are relevant actions
        if in_emergency is True:
            escape()
            print("escaping")
            in_emergency = False

        elif in_bound is False:
            print("out of bounds") # update this later
            boundary_exception()
            in_bound = True    


def planPath(finalPose):

    posePath = []
    path = []
    current_position = current_position()
    
    dX = finalPose[0]-current_position[0]
    dY = finalPose[1]-current_position[1]
    dZ = finalPose[2]-current_position[2]
    sep = ((dX)**2+(dY)**2+(dZ)**2)**0.5

    num_segments = sep/0.2

    #Assume a straight line and cut it into points

    for positions in range(num_segments):
        posePath[positions] = [current_position[0] + dX * (positions + 1/num_segments), current_position[1] + dY * (positions + 1/num_segments), current_position[2] + dZ * (positions + 1/num_segments)]

    #Use next_move function to calculate the best move given human joints
    for positions in enumerate(posePath):
        path[positions] = next_move(joints_list, posePath[positions], ws_array)

    return path
              
def CheckPathCollision(target_poses):
    
    path_collision_threshold = 0.25

    joints_list = update_joints(joint_coords)

    for poses in target_poses:
        #Check for collisions between each robot joint and each human joint
        for h_joint in joints_list:
            sep = separation(poses, h_joint)
            if sep <= path_collision_threshold:
                print("Human is too close to robot path")
                print("Separation is", sep, "from joint at: ", poses)
                return True
    return False 
    

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

def CheckEmergencyStop():

    collision_threshold = 0.1

    current_robot_joints = GetRobotJointPositions()
    joints_list = update_joints(joint_coords)

    for r_joint in current_robot_joints:
        #Check for collisions between each robot joint and each human joint
        for h_joint in joints_list:
            sep = separation(r_joint, h_joint)
            if sep <= collision_threshold:
                print("Robot is too close to joints")
                print("Separation is", sep, "from joint at: ", r_joint)
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

def cp_cost(point, joints):
    # calculates the cost associated with a point due to surrounding joints
    cost = 0
    for joint in joints:
        sep = separation(point, joint)
        if sep == 0:
            return 1000000
        elif sep < 0.5:
            cost = 1/sep**2 + cost
    return cost

def next_move (joints, target, ws_array):
    min_cost = 1000000 # declare as any large number
    ws_array += [target] # ensure that the precise target position cost is always evaluated
    weight = 0.001 # reducing this will allow the robot to get close to joints, and vice versa
    for pos in ws_array: # evaluate each position in array
        cost = cp_cost(pos, joints)*weight + separation(pos, target) 
        if cost < min_cost:
            min_cost = cost
            min_pose = pos
    return min_pose # the most cost effective position to move to
    
def init_ws_array(resolution): # will return array with resolution**3 coordinates
    x_size = maximum_x_boundary - minimum_x_boundary
    y_size = maximum_y_boundary - minimum_y_boundary
    z_size = maximum_z_boundary - minimum_z_boundary

    i = []
    x_vals = []
    y_vals = []
    z_vals = []

    for pos in range(resolution):
        x_vals += [minimum_x_boundary + pos*(x_size/(resolution-1))]

    for pos in range(resolution):
        y_vals += [minimum_y_boundary + pos*(y_size/(resolution-1))]
  
    for pos in range(resolution):
        z_vals += [minimum_z_boundary + pos*(z_size/(resolution-1))]

    for j in x_vals:
        for k in y_vals:
            for l in z_vals:
                i += [[j, k, l]]

    return(i)
    
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
    
def GetRobotJointPositions():

    np.set_printoptions(precision=3, suppress=True)

    d1 = 0.163
    a2 = -0.42500
    a3 = -0.39225
    d4 = 0.134      
    d5 = 0.100
    d6 = 0.100

    current_joint_pos = move_group.get_current_joint_values()

    dh_params = np.array([[d1, 0, pi/2, 0]])
    theta = np.array([current_joint_pos[0]])
    base_joint = getJointPose(dh_params, theta)

    dh_params = np.array([[d1, 0, pi/2, 0],
                          [0, a2, 0, 0]])
    theta = np.array([current_joint_pos[0], current_joint_pos[1]])
    shoulder_joint = getJointPose(dh_params, theta)

    dh_params = np.array([[d1, 0, pi/2, 0],
                          [0, a2, 0, 0],
                          [0, a3, 0, 0]])
    theta = np.array([current_joint_pos[0], current_joint_pos[1], current_joint_pos[2]])
    elbow_joint = getJointPose(dh_params, theta)

    dh_params = np.array([[d1, 0, pi/2, 0],
                          [0, a2, 0, 0],
                          [0, a3, 0, 0],
                          [d4 , 0, pi/2, 0]])
    theta = np.array([current_joint_pos[0], current_joint_pos[1], current_joint_pos[2], current_joint_pos[3]])
    wrist_1_joint = getJointPose(dh_params, theta)

    dh_params = np.array([[d1, 0, pi/2, 0],
                          [0, a2, 0, 0],
                          [0, a3, 0, 0],
                          [d4 , 0, pi/2, 0],
                          [d5, 0, -pi/2, 0]])
    theta = np.array([current_joint_pos[0], current_joint_pos[1], current_joint_pos[2], current_joint_pos[3], current_joint_pos[4]])
    wrist_2_joint = getJointPose(dh_params, theta)

    dh_params = np.array([[d1, 0, pi/2, 0],
                          [0, a2, 0, 0],
                          [0, a3, 0, 0],
                          [d4, 0, pi/2, 0],
                          [d5, 0, -pi/2, 0],
                          [d6, 0, 0, 0]])
    theta = np.array([current_joint_pos[0], current_joint_pos[1], current_joint_pos[2], current_joint_pos[3], current_joint_pos[4], current_joint_pos[5]])
    wrist_3_joint = getJointPose(dh_params, theta)

    base_joint = [-base_joint[0], -base_joint[1], base_joint[2]]
    shoulder_joint = [-shoulder_joint[0], -shoulder_joint[1], shoulder_joint[2]]
    elbow_joint = [-elbow_joint[0], -elbow_joint[1], elbow_joint[2]]
    wrist_1_joint = [-wrist_1_joint[0], -wrist_1_joint[1], wrist_1_joint[2]]
    wrist_2_joint = [-wrist_2_joint[0], -wrist_2_joint[1], wrist_2_joint[2]]
    wrist_3_joint = [-wrist_3_joint[0], -wrist_3_joint[1], wrist_3_joint[2]]

    joint_positions = [base_joint, shoulder_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
    #print(joint_positions)

    return joint_positions

def getJointPose(dh_params, theta):
    np.set_printoptions(precision=3, suppress=True)
    robot = RobotSerial(dh_params)
    f = robot.forward(theta)
    return f.t_3_1.reshape([3, ])


def update_joints(joint_coords):
    SHOULDER_LEFT = [joint_coords[15], joint_coords[16], joint_coords[17]]
    ELBOW_LEFT = [joint_coords[18], joint_coords[19], joint_coords[20]]
    HAND_LEFT = [joint_coords[24], joint_coords[25], joint_coords[26]]
    SHOULDER_RIGHT = [joint_coords[36], joint_coords[37], joint_coords[38]]
    ELBOW_RIGHT = [joint_coords[39], joint_coords[40], joint_coords[41]]
    HAND_RIGHT = [joint_coords[45], joint_coords[46], joint_coords[47]]
    HEAD = [joint_coords[78], joint_coords[79], joint_coords[80]]

    joints = [SHOULDER_LEFT, ELBOW_LEFT, HAND_LEFT, SHOULDER_RIGHT, ELBOW_RIGHT,HAND_RIGHT,HEAD]

    SHOULDER_LEFT_p = geometry_msgs.msg.PoseStamped()
    SHOULDER_LEFT_p.header.frame_id = robot.get_planning_frame()
    SHOULDER_LEFT_p.pose.position.x = joints[0][0]
    SHOULDER_LEFT_p.pose.position.y = joints[0][1]
    SHOULDER_LEFT_p.pose.position.z = joints[0][2]
    scene.add_box("SHOULDER_LEFT", SHOULDER_LEFT_p, (0.2, 0.2, 0.2))
    
    ELBOW_LEFT_p = geometry_msgs.msg.PoseStamped()
    ELBOW_LEFT_p.header.frame_id = robot.get_planning_frame()
    ELBOW_LEFT_p.pose.position.x = joints[1][0]
    ELBOW_LEFT_p.pose.position.y = joints[1][1]
    ELBOW_LEFT_p.pose.position.z = joints[1][2]
    scene.add_box("ELBOW_LEFT", ELBOW_LEFT_p, (0.2, 0.2, 0.2))
    
    HAND_LEFT_p = geometry_msgs.msg.PoseStamped()
    HAND_LEFT_p.header.frame_id = robot.get_planning_frame()
    HAND_LEFT_p.pose.position.x = joints[2][0]
    HAND_LEFT_p.pose.position.y = joints[2][1]
    HAND_LEFT_p.pose.position.z = joints[2][2]
    scene.add_box("HAND_LEFT", HAND_LEFT_p, (0.2, 0.2, 0.2))
    
    SHOULDER_RIGHT_p = geometry_msgs.msg.PoseStamped()
    SHOULDER_RIGHT_p.header.frame_id = robot.get_planning_frame()
    SHOULDER_RIGHT_p.pose.position.x = joints[3][0]
    SHOULDER_RIGHT_p.pose.position.y = joints[3][1]
    SHOULDER_RIGHT_p.pose.position.z = joints[3][2]
    scene.add_box("SHOULDER_RIGHT", SHOULDER_RIGHT_p, (0.2, 0.2, 0.2))
    
    ELBOW_RIGHT_p = geometry_msgs.msg.PoseStamped()
    ELBOW_RIGHT_p.header.frame_id = robot.get_planning_frame()
    ELBOW_RIGHT_p.pose.position.x = joints[4][0]
    ELBOW_RIGHT_p.pose.position.y = joints[4][1]
    ELBOW_RIGHT_p.pose.position.z = joints[4][2]
    scene.add_box("ELBOW_RIGHT", ELBOW_RIGHT_p, (0.2, 0.2, 0.2))
    
    HAND_RIGHT_p = geometry_msgs.msg.PoseStamped()
    HAND_RIGHT_p.header.frame_id = robot.get_planning_frame()
    HAND_RIGHT_p.pose.position.x = joints[5][0]
    HAND_RIGHT_p.pose.position.y = joints[5][1]
    HAND_RIGHT_p.pose.position.z = joints[5][2]
    scene.add_box("HAND_RIGHT", HAND_RIGHT_p, (0.2, 0.2, 0.2))
    
    HEAD_p = geometry_msgs.msg.PoseStamped()
    HEAD_p.header.frame_id = robot.get_planning_frame()
    HEAD_p.pose.position.x = joints[6][0]
    HEAD_p.pose.position.y = joints[6][1]
    HEAD_p.pose.position.z = joints[6][2]
    scene.add_box("HEAD", HEAD_p, (0.2, 0.2, 0.2))
        
    return joints

###########################################################################################################################################

# Create a subscriber for the /joint_coordinates topic
rospy.Subscriber('/joint_coordinates', Float64MultiArray, joint_coordinates_callback)

#Create a position saving variable
position_data = []
global ws_array
ws_array = init_ws_array(20)
print("check")

#Is a global until FSM is set up
checkEmergencyStop = False
goal = geometry_msgs.msg.Pose()
target1 = [-0.56, 0.4, 0.3]
target2 = [-0.56, -0.4, 0.3]

''' FOR POSITION CHECKING / CALIBRATION
while 1:
    cp = current_position()
    print(cp[0], cp[1], cp[2])
    rospy.sleep(1.0)
'''

global joint_coords

joint_coords = numpy.zeros(96) # initialize joint coords

a = move_group.get_current_pose()
b = GetRobotJointPositions()

while 1:
    # stall while no body detected
    if joint_coords[0] != 0:
        # First Movement
        joints_list = update_joints(joint_coords) # JOINTS PUBLISHER MUST BE RUNNING
        print("Left hand is at:", joints_list[2]) # DEBUGGING / CHECKING
        next_pos = next_move(joints_list, target1, ws_array)
        print("Moving to:", next_pos)
        pose_1 = PoseMaker(next_pos[0], next_pos[1], next_pos[2])
        MoveToPosition(pose_1)
        print("Finished moving to pose")
        rospy.sleep(0.5)

        # Second Movement
        joints_list = update_joints(joint_coords) # JOINTS PUBLISHER MUST BE RUNNING
        print("Left hand is at:", joints_list[2]) # DEBUGGING / CHECKING
        next_pos = next_move(joints_list, target2, ws_array)
        print("Moving to:", next_pos)    
        pose_1 = PoseMaker(next_pos[0], next_pos[1], next_pos[2])
        MoveToPosition(pose_1)
        print("Finished moving to pose")
        rospy.sleep(0.5)

    elif 1:
        rospy.sleep(1.0)
        print("no body detected, stalling")

    
# Setting up joints
while 1:
    try:
        print("entered loop")
        rospy.sleep(1.0)
        joints_list = update_joints(joint_coords) # JOINTS PUBLISHER MUST BE RUNNING
        print("joints identified")
        print("Left hand is at:", joints_list[2]) # DEBUGGING / CHECKING
        next_pos = next_move(joints_list, target1, ws_array)
        print("Moving to:", next_pos)
        
        pose_1 = geometry_msgs.msg.Pose()
        pose_1.position.x = next_pos[0]
        pose_1.position.y = next_pos[1]
        pose_1.position.z = next_pos[2]
        pose_1.orientation.x = move_group.get_current_pose().pose.orientation.x
        pose_1.orientation.y = move_group.get_current_pose().pose.orientation.y
        pose_1.orientation.z = move_group.get_current_pose().pose.orientation.z
        pose_1.orientation.w = move_group.get_current_pose().pose.orientation.w
        #MoveToPosition(pose_1)
        print("Finished moving to pose")
        rospy.sleep(0.5)
    except:
        pass


    


#HandFollowing()'''


#Notes
#z 0.45 limit
goal.position.x = -0.56
goal.position.y = 0.2
goal.position.z = 0.21
goal.orientation.x = move_group.get_current_pose().pose.orientation.x
goal.orientation.y = move_group.get_current_pose().pose.orientation.y
goal.orientation.z = move_group.get_current_pose().pose.orientation.z
goal.orientation.w = move_group.get_current_pose().pose.orientation.w

# Define the first target pose
pose_1 = geometry_msgs.msg.Pose()
pose_1.position.x = -0.3
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
    csv_writer.writerows(position_data)

