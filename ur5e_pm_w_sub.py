#!/usr/bin/env python3
# This makes the ur5 move to a position 1, wait 2 seconds and then move to position 2, configured to allow for testing with static and dynamic obstacles
# To execute this code ensure MoveIt!, Universal Robots, Ros, and this directory is sourced.
# Ensure this code is executable by running chmod +x "'this file name'.py"
# start MoveIt:
# roslaunch ur_gazebo ur5_bringup.launch
# roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=true
# roslaunch ur5_moveit_config moveit_rviz.launch
# Authors: Jarrod Chan & Matthew Horning

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

# ROS initilization
rospy.init_node('ur5e_physical_movement', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Specify Static obstacle planner
move_group.set_planner_id("RRTConnectkConfigDefault")

# Set tolerance & Velocity Scaling Factor
move_group.set_goal_position_tolerance(0.01)
move_group.set_max_velocity_scaling_factor(0.1)
#move_group.set_planning_time(25.0)

# Logging
rospy.loginfo("Start")

# If generated objects exist from previous trial, remove them
scene.remove_world_object("back_wall")
scene.remove_world_object("left_wall")
scene.remove_world_object("right_wall")
scene.remove_world_object("bottom_wall")
scene.remove_world_object("top_wall")
rospy.sleep(1)

# Add collision boundarys for the workspace
back_wall = geometry_msgs.msg.PoseStamped()
back_wall.header.frame_id = robot.get_planning_frame()
back_wall.pose.position.x = 0.7
back_wall.pose.position.y = 0.1
back_wall.pose.position.z = 0.5
scene.add_box("back_wall", back_wall, (0.3, 2, 1))

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
top_wall.pose.position.z = 0.9
scene.add_box("top_wall", top_wall, (2, 2, 0.1))

# Specify numerical bounds for robot movement, requested movements outside these regions will be rejected
global minimum_x_boundary, maximum_x_boundary, minimum_y_boundary, maximum_y_boundary, minimum_z_boundary, maximum_z_boundary
minimum_x_boundary = -0.8
maximum_x_boundary = -0.11
minimum_y_boundary = -0.5
maximum_y_boundary = 0.490
minimum_z_boundary = 0.14
maximum_z_boundary = 0.9

# Will show zones in RVIZ for when planning is not running, useful for visualisaiton and debugging
global vis_separation
vis_separation = 0.3
rospy.sleep(1)

####################################SAFETY FUNCTIONS###############################################################
def WithinBoundary():
    # Check on whether or not the robot's TCP is outside the user defined boundary, returns TRUE if robot is outside bounds
    # Get current TCP position
    current_pose_x = move_group.get_current_pose().pose.position.x
    current_pose_y = move_group.get_current_pose().pose.position.y
    current_pose_z = move_group.get_current_pose().pose.position.z

    # Determine boundary which has been exceeded (IF ANY)
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

def WithinTarget(target_pose):
    # Checks to see if the robot has reached the goal position, returns true if robot's TCP is within 0.05m of target
    # Define target acceptance threshold
    threshold = 0.05
    
    # Get current TCP position
    current_pose_x = move_group.get_current_pose().pose.position.x
    current_pose_y = move_group.get_current_pose().pose.position.y
    current_pose_z = move_group.get_current_pose().pose.position.z
    
    # Calculate separation of TCP from target position
    distance = math.sqrt((current_pose_x - target_pose.position.x )**2 + (current_pose_y - target_pose.position.y)**2 + (current_pose_z - target_pose.position.z)**2)
    
    # Return true is separation of TCP from target is wihtin threshold
    if distance < threshold:
        rospy.loginfo("Current Position X:{} Y:{} Z:{}".format(current_pose_x, current_pose_y,current_pose_z))
        return True
    else:
        return False
############################################### DATA STORAGE FUNCTIONS ######################################################
def AppendData():
    # Data logging funciton to output TCP position over time, used to produce figures illustrating robot trajectory across different trials
    current_pose_x = move_group.get_current_pose().pose.position.x
    current_pose_y = move_group.get_current_pose().pose.position.y
    current_pose_z = move_group.get_current_pose().pose.position.z
    position_data.append([current_pose_x, current_pose_y, current_pose_z])
    
############################################### MOVEMENT FUNCTIONS ######################################################
def boundary_exception():
    # In cases where the function WithinBoundary() returns true, this function will be called to retreat the robot back within bounds
    print("boundary exception triggered")
    # Initializing variables
    cp = current_position()
    bound_move = cp
    in_target = False
    last_pos = [0,0,0]

    # Determine which boundary has been exceeded, and consequently where to set the target pose to move back wihtin bounds
    # In cases where more than one boundary is exceeded, exception will be triggered again
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

    # Plan movement
    move_group.set_pose_target(bound_move)
    plan = move_group.plan()

    # Loop to allow for replanning
    while plan is False:
        print("planning boundary move")        
        joints_list = update_joints(joint_coords, vis_separation)    
        plan = move_group.plan()
    print("moving to:", bound_move)
    move_group.go(wait=False)

    # Continue moving and updating joint positions until target position for boundary exception is reached
    while in_target is False: 
        AppendData()
        in_target = WithinTarget(bound_move)
        move_group.go(wait=False)
        current_pos = current_position()
        
        while in_target is False:
            joints_list = update_joints(joint_coords, vis_separation)
            AppendData()
            in_target = WithinTarget(bound_move)
            current_pos = current_position()
            
            if abs(current_pos[0] - last_pos[0]) < 0.0001:
                move_group.go(wait=False)
            last_pos = current_pos
            rospy.sleep(0.1)

    print("Exiting boundary exception")   

def saturate_bounds(x, y, z):
    # Function to saturate a position value, used for commanding the robot, within workspace bounds
    if x <= minimum_x_boundary:
        x = minimum_x_boundary + 0.1
    elif x >= maximum_x_boundary:
        x = maximum_x_boundary - 0.1
    
    if y <= minimum_y_boundary:
        y = minimum_y_boundary + 0.1
    elif y >= maximum_y_boundary:
        y = maximum_y_boundary - 0.1
    
    if z <= minimum_z_boundary:
        z = minimum_z_boundary + 0.1
    elif z >= maximum_z_boundary:
        z = maximum_z_boundary - 0.1

    return [x, y, z]

def escape():
    # Function to control the direction the robot will attempt to move after it moves too close to a joint
    # Initialization
    current_pose = current_position()
    joints_list = update_joints(joint_coords, 0.1)
    print("in escape")
    min_sep = 1000
    plan = False
    in_target = False
    last_pos = [0,0,0]

    # Iterate through all nominated joints and determine which is closest
    for joint in joints_list:
        sep = separation(current_pose, joint)
        if sep <= min_sep:
            min_joint = joint
            min_sep = sep

    # If closest joint is within threshold, retreat object away from joint
    if  min_sep <= 0.4: # if joints is within 0.2 of tcp
        print ("Separation is:", min_sep)
        print("Current pose is:", current_pose, "joint is", min_joint)
        f = 1.5 # wil go x (0.3) from closest joint
        v = numpy.subtract(current_pose, min_joint) # numpy.subtract
        v_scaled = [f * v[0], f * v[1], f * v[2]]
        sat_p = saturate_bounds(min_joint[0] + v_scaled[0] + 0.2, min_joint[1] + v_scaled[1], min_joint[2] + v_scaled[2])
        retreat = PoseMaker(sat_p[0], sat_p[1], sat_p[2])

        # Correcting orientation so tcp remains standard if movement is interrupted
        retreat.orientation.x = -0.7055840819676955
        retreat.orientation.y = -0.708634452751245
        retreat.orientation.z = 0.00014032697939014228
        retreat.orientation.w = 0.0005000702047506833
        print("Retreating to", retreat)
        move_group.set_pose_target(retreat)
        while plan is False:        
            joints_list = update_joints(joint_coords, 0.1)    
            plan = move_group.plan()
        move_group.go(wait=False)

        # Continue movement until new target pose is reached
        while in_target is False:
            AppendData()
            in_target = WithinTarget(retreat)
            current_pos = current_position()
            joints_list = update_joints(joint_coords, 0.1)
            
            if abs(current_pos[0] - last_pos[0]) < 0.0001:
                move_group.go(wait=False)
            last_pos = current_pos
            rospy.sleep(0.1)

    print("Exiting escape")


def current_position():
    # Function to convert data held in robot 'pose' to an array of floats
    current_pose_x = move_group.get_current_pose().pose.position.x
    current_pose_y = move_group.get_current_pose().pose.position.y
    current_pose_z = move_group.get_current_pose().pose.position.z
    current_pose = [current_pose_x, current_pose_y, current_pose_z]
    return current_pose

def MoveToPosition(target_pose):
    # Main function that manages dynamic collision avoidance for the robot, most other functions in this file are used here       
    if CheckEmergencyStop():
        escape()

    # Initialize flags
    in_bound = True
    in_target = False
    in_emergency = False   

    # Continue movement until target is reached
    while in_target is False:
        joints_list = update_joints(joint_coords, 0.3)
        plan = False
        last_pos = [0,0,0]
        target_xyz = [target_pose.position.x, target_pose.position.y, target_pose.position.z]
        next_position = next_move(joints_list, target_xyz, ws_array)
        next_pose = PoseMaker(next_position[0], next_position[1], next_position[2])
        move_group.set_pose_target(next_pose)
        in_bound = WithinBoundary()
        in_emergency = CheckEmergencyStop()
        in_emergency_non_tcp = CheckNonTcp()
        joints_list = update_joints(joint_coords, 0.35)

        # Initital safety check of all flags prior to starting movement
        if (in_bound is True and in_target is False and in_emergency is False):
            while plan is False:
                print("planning movement to:", next_pose)
                joints_list = update_joints(joint_coords, 0.35)
                plan = move_group.plan()
            move_group.go(wait=False)

        # Main movement loop, will temporarily exit if boundary or emergency flag is triggered, permanently if target flag is triggered
        while(in_bound is True and in_target is False and in_emergency is False):
            # Check position
            in_bound = WithinBoundary()
            in_target = WithinTarget(next_pose)
            in_emergency = CheckEmergencyStop() # For joint proximity to TCP
            in_emergency_non_tcp = CheckNonTcp() # For joint proximity to other robot joints
            AppendData()

            current_pos = current_position()
            
            if abs(current_pos[0] - last_pos[0]) < 0.0001:
                move_group.go(wait=False)

            last_pos = current_position()
            rospy.sleep(0.1)
        
        move_group.stop() # Loop has exited, something has happened, stop the robot
        move_group.clear_pose_targets()

        in_target = WithinTarget(target_pose)

        # Determine which flag was triggered, causing the loop exit
        while in_emergency_non_tcp is True:
            # If a joint is too close to a non-tcp joint, implies it will be unsafe for the robot to move and in all cases, difficult for a safe trajectory to be generated
            in_emergency_non_tcp = CheckNonTcp()
            rospy.sleep(0.1)

        if in_emergency is True:
            print("escaping")
            escape()            
            in_emergency = False

        elif in_bound is False:
            print("out of bounds") # update this later
            boundary_exception()
            in_bound = True                
              
def CheckNonTcp():
    # Function which iterates through all of the non-tcp joints and calcualtes their separation from the nominated joints, returns true if any non-tcp robot joint is too close to a nominated joint
    threshold = 0.1
    current_joints = GetRobotJointPositions()

    joints_list = update_joints(joint_coords, vis_separation)
    for r_joint in current_joints:
        for h_joint in joints_list:
            sep = separation(r_joint, h_joint)
            if sep <= threshold:
                print("Robot is too close to non-tcp joint")
                print("Separation is", sep, "from joint at: ", r_joint)
                return True
            
    return False 

def PoseMaker(x, y, z):
    # Converts a cartesian point defined by inputs X, Y & Z to a MoveIt pose
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
    # Moves to a target position using x y z cartesian coordinates as an input instead of a robot pose
    target_pose = PoseMaker(x, y, z)
    MoveToPosition(target_pose)

def CheckEmergencyStop():
    # Function which checks the proximity of the robot's TCP to each of the nominated joints, if any is within threshold escape function will be triggered
    threshold = 0.4
    current_pose = current_position()    
    joints_list = update_joints(joint_coords, vis_separation)
    for joint in joints_list:
        sep = separation(current_pose, joint)
        if sep <= threshold:
            print("Emergency True")
            print("TCP is", sep, "from closest a joint at", joint)
            rospy.loginfo("Current Position X:{} Y:{} Z:{}".format(current_pose[0], current_pose[1], current_pose[2]))
            rospy.loginfo("EMERGENCY STOP")
            return True      
    return False
        
def HandFollowing():
    # An early function used to test depth information performance and planning times, robot is instructed to follow the hand of the operator indefinitely
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
    # Helper funciton which calculates the distance between two points in 3-D space
    dX = p2[0]-p1[0]
    dY = p2[1]-p1[1]
    dZ = p2[2]-p1[2]
    sep = ((dX)**2+(dY)**2+(dZ)**2)**0.5
    return sep

def cp_cost(point, joints):
    # calculates the cost associated with a point due to surrounding joints, backbone of potential field system
    cost = 0
    for joint in joints:
        sep = separation(point, joint)
        if sep == 0:
            return 1000000
        elif sep < 0.5:
            cost = 1/sep**2 + cost
    return cost

def next_move (joints, target, ws_array):
    # Determines the next position to move to, will usually return the input target but will return closest free position within bounds if target is occupied by a human joint
    min_cost = 1000000 # declare as any large number
    ws_array += [target] # ensure that the precise target position cost is always evaluated
    weight = 0.001 # reducing this will allow the robot to get close to joints, and vice versa
    for pos in ws_array: # evaluate each position in array
        cost = cp_cost(pos, joints)*weight + separation(pos, target) 
        if cost < min_cost:
            min_cost = cost
            min_pose = pos
    return min_pose # the most cost effective position to move to
    
def init_ws_array(resolution): 
    # will return array with resolution**3 coordinates, a discrete representation of the workspace used to calculate the costs associated with different positions    
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
    
############################################### CALLBACK FUNCTIONS (FOR JOINT COORDINATES SUBCRIBER) ######################################
def joint_coordinates_callback(data):
    # This function will be called whenever a message is received on the /joint_coordinates topic
    # The received joint coordinates are stored in 'data'
    # joint coord global
    global joint_coords

    # Extract the joint coordinates
    joint_coords = data.data

################################################ SENSOR PROCESSING#########################################################################
def GetJointPositions():
    # Function which gets current joint positions from array
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
    
def GetRobotJointPositions():
    # Function which returns the current joints position of the robot as an array using inverse kinematics
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


def update_joints(joint_coords, radius):
    # Function which updates the nominated joints based off of the subscriber data - update in real-time
    # Is called prior to any motion plans and during movement, to ensure dynamic collisions can be prevented
    # Adds collision objects in MoveIt which can be visualised in RVIZ, used for static obstacle planning
    SHOULDER_LEFT = [joint_coords[15], joint_coords[16], joint_coords[17]]
    ELBOW_LEFT = [joint_coords[18], joint_coords[19], joint_coords[20]]
    HAND_LEFT = [joint_coords[24], joint_coords[25], joint_coords[26]]
    SHOULDER_RIGHT = [joint_coords[36], joint_coords[37], joint_coords[38]]
    ELBOW_RIGHT = [joint_coords[39], joint_coords[40], joint_coords[41]]
    HAND_RIGHT = [joint_coords[45], joint_coords[46], joint_coords[47]]
    HEAD = [joint_coords[78], joint_coords[79], joint_coords[80]]

    joints = [SHOULDER_LEFT, ELBOW_LEFT, HAND_LEFT, SHOULDER_RIGHT, ELBOW_RIGHT,HAND_RIGHT,HEAD]

    # Add the joints as spherical collision objects
    SHOULDER_LEFT_p = geometry_msgs.msg.PoseStamped()
    SHOULDER_LEFT_p.header.frame_id = robot.get_planning_frame()
    SHOULDER_LEFT_p.pose.position.x = joints[0][0]
    SHOULDER_LEFT_p.pose.position.y = joints[0][1]
    SHOULDER_LEFT_p.pose.position.z = joints[0][2]
    scene.add_sphere("SHOULDER_LEFT", SHOULDER_LEFT_p, radius -0.05)
    
    ELBOW_LEFT_p = geometry_msgs.msg.PoseStamped()
    ELBOW_LEFT_p.header.frame_id = robot.get_planning_frame()
    ELBOW_LEFT_p.pose.position.x = joints[1][0]
    ELBOW_LEFT_p.pose.position.y = joints[1][1]
    ELBOW_LEFT_p.pose.position.z = joints[1][2]
    scene.add_sphere("ELBOW_LEFT", ELBOW_LEFT_p, radius)
    
    HAND_LEFT_p = geometry_msgs.msg.PoseStamped()
    HAND_LEFT_p.header.frame_id = robot.get_planning_frame()
    HAND_LEFT_p.pose.position.x = joints[2][0]
    HAND_LEFT_p.pose.position.y = joints[2][1]
    HAND_LEFT_p.pose.position.z = joints[2][2]
    scene.add_sphere("HAND_LEFT", HAND_LEFT_p, radius)
    
    SHOULDER_RIGHT_p = geometry_msgs.msg.PoseStamped()
    SHOULDER_RIGHT_p.header.frame_id = robot.get_planning_frame()
    SHOULDER_RIGHT_p.pose.position.x = joints[3][0]
    SHOULDER_RIGHT_p.pose.position.y = joints[3][1]
    SHOULDER_RIGHT_p.pose.position.z = joints[3][2]
    scene.add_sphere("SHOULDER_RIGHT", SHOULDER_RIGHT_p, radius -0.05)
    
    ELBOW_RIGHT_p = geometry_msgs.msg.PoseStamped()
    ELBOW_RIGHT_p.header.frame_id = robot.get_planning_frame()
    ELBOW_RIGHT_p.pose.position.x = joints[4][0]
    ELBOW_RIGHT_p.pose.position.y = joints[4][1]
    ELBOW_RIGHT_p.pose.position.z = joints[4][2]
    scene.add_sphere("ELBOW_RIGHT", ELBOW_RIGHT_p, radius)
    
    HAND_RIGHT_p = geometry_msgs.msg.PoseStamped()
    HAND_RIGHT_p.header.frame_id = robot.get_planning_frame()
    HAND_RIGHT_p.pose.position.x = joints[5][0]
    HAND_RIGHT_p.pose.position.y = joints[5][1]
    HAND_RIGHT_p.pose.position.z = joints[5][2]
    scene.add_sphere("HAND_RIGHT", HAND_RIGHT_p, radius)
    
    HEAD_p = geometry_msgs.msg.PoseStamped()
    HEAD_p.header.frame_id = robot.get_planning_frame()
    HEAD_p.pose.position.x = joints[6][0]
    HEAD_p.pose.position.y = joints[6][1]
    HEAD_p.pose.position.z = joints[6][2]
    scene.add_sphere("HEAD", HEAD_p, radius+0.05)
        
    return joints

def align_tcp():
    # Ensure tcp has standard orientation
    cp = current_position()
    cp = PoseMaker(cp[0], cp[1], cp[2])
    cp.orientation.x = -0.7055840819676955
    cp.orientation.y = -0.708634452751245
    cp.orientation.z = 0.00014032697939014228
    cp.orientation.w = 0.0005000702047506833
    move_group.set_pose_target(cp)
    plan = move_group.plan()
    move_group.go(wait=True)

###########################################################################################################################################

# Create a subscriber for the /joint_coordinates topic
rospy.Subscriber('/joint_coordinates', Float64MultiArray, joint_coordinates_callback)

#Create a position saving variable
position_data = []
global ws_array
global joint_coords
ws_array = init_ws_array(20)
joint_coords = numpy.zeros(96) # initialize joint coords
print("check")
checkEmergencyStop = False
goal = geometry_msgs.msg.Pose()
target1 = [-0.56, 0.4, 0.3]
target2 = [-0.56, -0.4, 0.3]
completed = False

# Ensure TCP is in standard position
align_tcp()

# Stalls until body of human operator is detected detected
while completed == False:
    if joint_coords[0] != 0:
        # First Movement
        joints_list = update_joints(joint_coords, vis_separation) # JOINTS PUBLISHER MUST BE RUNNING
        print("Left hand is at:", joints_list[2]) # DEBUGGING / CHECKING
        next_pos = next_move(joints_list, target1, ws_array)
        print("Moving to:", next_pos)
        pose_1 = PoseMaker(next_pos[0], next_pos[1], next_pos[2])
        MoveToPosition(pose_1)
        print("Finished moving to pose")
        rospy.sleep(0.5)

        # Second Movement
        joints_list = update_joints(joint_coords, vis_separation) # JOINTS PUBLISHER MUST BE RUNNING
        print("Left hand is at:", joints_list[2]) # DEBUGGING / CHECKING
        next_pos = next_move(joints_list, target2, ws_array)
        print("Moving to:", next_pos)    
        pose_1 = PoseMaker(next_pos[0], next_pos[1], next_pos[2])
        MoveToPosition(pose_1)
        print("Finished moving to pose")
        rospy.sleep(0.5)

        completed = True

        # After your loop, save the position data to a CSV file
    print("no body detected, stalling")
    rospy.sleep(1.0)
        

with open('tcp_position.csv', 'a', newline='') as csvfile:
        # Exports data collected using AppendData function to a csv, 3-D plots were later produced in MATLAB
        csv_writer = csv.writer(csvfile)
        
        # Write a header row with column names
        csv_writer.writerow(['X', 'Y', 'Z'])
        
        # Write the position data
        csv_writer.writerows(position_data)
