#!/usr/bin/env python
#################################################################
##\file
#
# \note
#   Copyright (c) 2011-2012 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_calibration
# \note
#   ROS package name: cob_calibration_executive
#
# \author
#   Author: Topias Liikanen, email:liikanen.topias@gmail.com
# \author
#   Supervised by: Nadia Hammoudeh Garcia, email:Nadia.HammoudehGarcia@ipa.fhg.de
# \author
#   Author: Shehab El Din, email:###
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: July 2015
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################
from visualization_msgs.msg._MarkerArray import MarkerArray
PKG = 'cob_calibration_executive'
NODE = 'generate_robot_calibration_data_node'
""" The strings here regarding group names should be accounted for later.
The number of groups should be dynamic"""
import roslib
roslib.load_manifest(PKG)
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
import tf
import numpy as np
import yaml
import os
import math
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from simple_script_server import simple_script_server
from random import shuffle  #shuffle links to look at
# from cob_calibration_executive.torso_ik import TorsoIK

import pdb # debugger

#board       = Checkerboard(self.pattern_size, self.square_size)
#checkerboard_detector=CheckerboardDetector()
#latest_image=Image()

def visualize_markers(x,y,z,frame,id,color,scale):
    marker = Marker()
    marker.header.frame_id = frame
    marker.id = id
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    return marker


def get_cb_pose_center(listener, base_frame):
    return listener.lookupTransform(base_frame, '/chessboard_center', rospy.Time(0))


def get_cb_pose(listener, base_frame):
    return listener.lookupTransform(base_frame, '/chessboard_position_link', rospy.Time(0))

def get_position(listener, tip):
    return listener.lookupTransform('/torso_3_link', tip, rospy.Time(0))
def generate_calibration_trajectory(moveGroup,limits,discritization,orientation_mean,reference_frame,gazebo_simulation, cb_publisher):
    '''
    @param moveGroup: Is the move MoveGroupCommander of the group
    @type moveGroup: moveit_commander.MoveGroupCommander
    @param limits: Are the limits of the cube wrt to the @reference_frame
    @type limits: a list of lists [[x1,y1,z1],[x2,y2,z2]]
    @param discritization: the number of times for which the range is discretized [x,y,z]
    @type discritization: A list []
    @param orientation_mean: The mean Orientation before noise addition [x,y,z,w]
    @type orientation_mean: A list []
    @param reference_frame: the frame of refrence in which the planning will take place using the pose given by @limits and @orientation_mean
    @type reference_frame: String (name of reference frame)
    @param gazebo_simulation: Flag to signal if we want gazebo to simulate the path while planning or not
    @param gazebo_simulation: Boolean
    @param cb_publisher: Chess board publisher
    '''
    listener = tf.TransformListener()
    m_topic_limits = 'marker_array_limits'
    m_topic_positions = 'marker_array_cb_positions'
    marker_publisher_limits = rospy.Publisher(m_topic_limits, MarkerArray)
    marker_publisher_positions = rospy.Publisher(m_topic_positions, MarkerArray)
    marker_array_limits = MarkerArray()
    marker_array_positions = MarkerArray()
    id = 0
    for i in range(2):
        for j in range(2):
            for k in range(2):
                id += 1
                marker = visualize_markers(limits[i][0], limits[j][1], limits[k][2], reference_frame, id, [1.0, 1.0, 0.0], 0.03)
                marker_array_limits.markers.append(marker)
    #marker1 = visualize_limits(limits[0][0], limits[0][1], limits[0][2], reference_frame, 1)
    #marker2 = visualize_limits(limits[1][0], limits[1][1], limits[1][2], reference_frame, 2)
    #marker_array.markers.append(marker1)
    #marker_array.markers.append(marker2)
    
    next_pose = geometry_msgs.msg.PoseStamped()  #init of the geometry_msgs pose
    current_joint_angles = moveGroup.get_current_joint_values()
    # making a state message and filling it
    group_state = moveit_msgs.msg.RobotState()
    group_state.joint_state.name = moveGroup.get_joints()
    group_state.joint_state.position = current_joint_angles
    # define cuboid for positions
    # limits from base_link frame
    limits = {'x': (limits[0][0], limits[1][0]),
              'y': (limits[0][1], limits[1][1]),
              'z': (limits[0][2], limits[1][2])}

    sample_density = {'x': discritization[0],
                      'y': discritization[1],
                      'z': discritization[2]}

    sample_positions = {'x': [],# initialization 
                        'y': [],
                        'z': []}
    for key in limits.keys():
        limits[key] = sorted(limits[key])
        sample_positions[key].append(limits[key][0])
        diff = limits[key][1] - limits[key][0]
        step = 1.0 * diff / (sample_density[key] - 1)

        while sample_positions[key][-1] + step <= (limits[key][1] + 0.01):
            sample_positions[key].append(sample_positions[key][-1] + step)

    group_joint_states = []
    group_joint_trajectory = []
    '''
@todo: Check this cb_links and find out if it should be removed
    '''
    cb_links = ["/chessboard_center","/chessboard_lu_corner",
                "/chessboard_ru_corner", "/chessboard_ll_corner",
                "/chessboard_rl_corner"]
    generated_positions = []
    marker_id = 0
    for x in sample_positions['x']:
        for y in sample_positions['y']:
            for z in sample_positions['z']:
                #for q in quaternion:
                for cb_link in cb_links:
                    """
@todo: Check why we need this cb_link loop
                    """
                    print "\033[1;34mNew Position\033[1;m"
                    next_pose.header.frame_id = reference_frame
                    next_pose.pose.position.x = x
                    next_pose.pose.position.y = y
                    next_pose.pose.position.z = z
                    '''
@todo: Check this part again for the addition of random noise
                    '''
                    # (0,0,0,1) for cob3-6
                    next_pose.pose.orientation.x = orientation_mean[0]
                    next_pose.pose.orientation.y = orientation_mean[1]
                    next_pose.pose.orientation.z = orientation_mean[2]
                    next_pose.pose.orientation.w = orientation_mean[3]

                    cb_publisher.publish(next_pose)  ### is this publish still needed ??
                    rospy.sleep(0.2)
                    next_pose.pose.orientation.x = orientation_mean[0]
                    next_pose.pose.orientation.y = orientation_mean[1]
                    next_pose.pose.orientation.z = orientation_mean[2]
                    next_pose.pose.orientation.w = orientation_mean[3]
                    cb_publisher.publish(next_pose)
                    marker_publisher_limits.publish(marker_array_limits)
                    rospy.sleep(0.2)
                    #transformation_base_cb = get_position(listener, cb_link)
                #Set starting pose for trajectory to continue planning from the last achieved point 
                group_state.joint_state.position = current_joint_angles
                moveGroup.set_start_state(group_state)
                #Set end pose for trajectory
                moveGroup.set_pose_target(next_pose.pose)
                #Plant paths for groups
                group_path_planned = moveGroup.plan()
                if group_path_planned.joint_trajectory.header.frame_id: # check if planning succeeded
                    print "\033[1;32mA solution has been found\033[1;m"
                    #update next iterations starting pose
                    if(gazebo_simulation):
                        moveGroup.go()
                    group_joint_trajectory.append(group_path_planned)
                    current_joint_angles = group_path_planned.joint_trajectory.points[-1].positions #update the robots joint angles for next planning iteration
                    group_joint_states.append({'joint_position': group_path_planned.joint_trajectory.points[-1].positions}) # append the attainable pose\s joint angles to the list then YAML 
                    marker = visualize_markers(x, y, z, reference_frame, marker_id, [0.0, 0.0, 1.0], 0.05)
                    generated_positions.append([x, y, z])
                    marker_id += 1
                    marker_array_positions.markers.append(marker)
                    marker_publisher_positions.publish(marker_array_positions)
                else:
                    print "\033[1;31mNo solution was found\033[1;m"
    return [group_joint_trajectory, group_joint_states, generated_positions]
    
def main():
    rospy.init_node(NODE)
    print "==> %s started " % NODE
    rospy.wait_for_service("/move_group/plan_execution/set_parameters",timeout=30.0)
    chessboard_pose = rospy.Publisher('/cob_calibration/chessboard_pose', geometry_msgs.msg.PoseStamped, queue_size=1)
    print 'chessboard_pose publisher activated'
    moveit_commander.roscpp_initialize(sys.argv) # init of moveit
    robot = moveit_commander.RobotCommander()# init of moveit wrt robot
#     scene = moveit_commander.PlanningSceneInterface() # init if scene if collisiton is to be avoided and for the addition of the cb pattern later
    print "============ Robot Groups:"
    print robot.get_group_names()
    print "============"
    listener = tf.TransformListener()
    rospy.sleep(1.5)
#     torso_ik = rospy.ServiceProxy('/lookat_get_ik', GetPositionIK) # no Torso link
    # init
    print "--> initializing sss"
    sss = simple_script_server()
    sss.init("base")
    sss.init("torso")
    sss.init("head")
    sss.recover("base")
    sss.recover("torso")
    sss.recover("head")
    sss.move("arm_left","home")
    sss.move("arm_right","home")
    print "--> components initialized"
    #camera_link_left = "/torso_cam3d_left_link" #renamed
    #camera_link_right = "/torso_cam3d_right_link" #new addition

    #[xhead, yhead, zhead] = get_position(listener, camera_link_left)[0]
    #print "Positon wrt torso_cam3d_left"
    #print xhead, yhead, zhead
    
    #[xhead, yhead, zhead] = get_position(listener, camera_link_right)[0]
    #print "Positon wrt torso_cam3d_left"
    #print xhead, yhead, zhead

    print "--> setup care-o-bot for capture"
    
#     arm_left_group = moveit_commander.MoveGroupCommander("arm_left") # define groups
#     arm_right_group = moveit_commander.MoveGroupCommander("arm_right")
#     arm_left_group.set_pose_reference_frame('torso_3_link')#***
#     arm_right_group.set_pose_reference_frame('torso_3_link')#***
#     arm_list = [arm_left_group,arm_right_group]
    
#     arm_planning_reference_frames = ['torso_cam3d_left_link','torso_cam3d_right_link']
#     orientation_mean = [[-0.55194, -0.83389, 0, 0],
#                         [0.83389, 0.55194, 0, 0]]
#     limits = [
#               [[0.15074, 0.40, 0.52078-0.698747/2],  #-0.457/2.0
#                [0.6808, -0.40, 1.135-0.698747/2]],
#               [[0.15074, 0.40, 0.52078-0.698747/2],  #-0.457/2.0
#                [0.6808, -0.40, 1.135-0.698747/2]]
#               ] 
#     discritization = [[2,2,2],
#                       [2,2,2]]
    
    
    try:
        arms = rospy.get_param('/cob_calibration_executive/arms')
    except:
        raise NameError('Couldn\'t find the configuration parameters under \'~cob_calibration/arms\' namespace.')
    
    config_list = []
    for k in arms.keys():
            config_list.append(arms[k])
            
    arm_names = []
    limits = []
    discritization = []
    orientation_mean = []
    arm_planning_reference_frames = []
    for i, arm in enumerate(config_list):
        arm_names.append(arm['name'])
        
        l1 = arm['limits_corner_1']
        l2 = arm['limits_corner_2']
        l1 = [float(eval(str(i))) for i in l1]   # convert the limit values into float 
        l2 = [float(eval(str(i))) for i in l2]
        limits.append([l1, l2])
        
        d = map(int, arm['discritization'])
        discritization.append(d)
        
        h = arm['hand_orientation']
        h = [float(eval(str(i))) for i in h]   # convert the orientation values into float 
        orientation_mean.append(h)
        
        arm_planning_reference_frames.append(str(arm['arm_planning_reference_frame']))
    
    arm_list = []
    for i, arm in enumerate(arm_names):
        arm_list.append(moveit_commander.MoveGroupCommander(arm))
        f = arm_planning_reference_frames[i]
        arm_list[i].set_pose_reference_frame(f)
    
    trajectory_joint_angles = [] # the first element of each entry is the Trajectory list for all successful calibration points while the second is the list of finla joint angles corresponding to that list
    
    file_path = rospy.get_param('~output_path', None)
    directory = os.path.dirname(file_path)
    if file_path is not None:
        if not os.path.exists(directory):
            os.makedirs(directory)
    #pdb.set_trace()
    for i in xrange(len(arm_list)):
        trajectory_joint_angles.append(generate_calibration_trajectory(
                                                    arm_list[i], 
                                                    limits[i],
                                                    discritization[i],
                                                    orientation_mean[i],
                                                    arm_planning_reference_frames[i],
                                                    False,
                                                    chessboard_pose))
   
        file_path_group_angles = file_path+arm_list[i].get_name()+"calibration_positions.yaml"
        with open(file_path_group_angles, 'w') as f:
            f.write('# autogenerated: Do not edit #\n')
            f.write(yaml.dump(trajectory_joint_angles[i][1]))# this is the second entry of the element in the list corresponding to the joint angles for all successful calibration points
        file_path_group_trajectories = file_path+arm_list[i].get_name()+"calibration_trajectories.yaml"
        with open(file_path_group_trajectories, 'w') as f:
            f.write('# autogenerated: Do not edit #\n')
            f.write(yaml.dump(trajectory_joint_angles[i][0])) #this is the first entry of the element in the list corresponding to the joint trajectories for all successful calibration points
            display_string  = '%s ik solutions found for '% len(trajectory_joint_angles[i][0])
            display_string = display_string + arm_list[i].get_name()
            print (display_string)
        file_path_cb_positions = file_path+arm_list[i].get_name()+"_cb_positions.yaml"
        with open(file_path_cb_positions, 'w') as f:
            f.write('# autogenerated: Do not edit #\n')
            trajectory_joint_angles[i][2].append(arm_planning_reference_frames[i])
            f.write(yaml.dump(trajectory_joint_angles[i][2]))


def calculate_angles(t):
    '''
    computes pan and tilt angles for camera like translations
    z-axis: optical axis
    y-axis: vertical
    x-axis: horizontal
    '''
    angles = {}
    angles['p'] = np.arctan2(t[0], t[2])
    angles['t'] = np.arctan2(t[1], t[2])
    return angles

if __name__ == '__main__':
    main()
    print "==> done exiting"
