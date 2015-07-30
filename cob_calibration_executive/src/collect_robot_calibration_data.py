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
#   Author: Shehab El Din, email:###
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# \author
#   Author: Topias Liikanen, email:liikanen.topias@gmail.com
# \author
#   Supervised by: Nadia Hammoudeh Garcia, email:Nadia.HammoudehGarcia@ipa.fhg.de
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
PKG = 'cob_calibration_executive'
NODE = 'collect_robot_calibration_data_node'
import roslib
roslib.load_manifest(PKG)
import rospy

from simple_script_server import simple_script_server
from cob_calibration_msgs.srv import Visible, Capture
from cob_calibration_msgs.msg import Progress
import yaml
import tf
import moveit_commander
import moveit_msgs.msg
import sys
# from cob_srvs.srv import SetJointStiffnessRequest, SetJointStiffness

import pdb # debugger

def capture_loop(positions, sss, visible, capture_kinematics, capture_image):
    '''
    @deprecated: This function is being depreciated check out data_capture_loop()
    Moves arm to all positions using script server instance sss
    and calls capture() to capture samples
    '''
    progress_pub = rospy.Publisher(
        "/calibration/data_collection/progress", Progress)
    br = tf.TransformBroadcaster()
    counter_camera = 0
    counter_kinematics = 0
    msg = Progress()
    for index in range(len(positions)):
        msg.Percent_done = round(100.0 * index / len(positions), 2)
        msg.Samples_left = len(positions) - index
        progress_pub.publish(msg)

        print "--> moving arm to sample #%s" % index
        pos = positions[index]
        #joint_pos = [[((a + (np.pi)) % (2 * np.pi)) - (np.pi)
                      #for a in positions[index]['joint_position']]]
        joint_pos = [[a for a in positions[index]['joint_position']]]
        print pos
        nh = sss.move("arm", joint_pos)
        while nh.get_state() == 0:
            rospy.sleep(0.2)
        if nh.get_state() != 3:     #??? Take another look (doesn't seem to make sense)
            sss.move("torso", "home")
            nh = sss.move("arm", joint_pos)
            rospy.sleep(1)
            if nh.get_state() != 3:
                continue

        print nh.get_state()
        br.sendTransform((0, 0, 0.24),      #??? Take another look (doesn't seem to make sense)
                         (0, 0, 0, 1),
                         rospy.Time.now(),
                         "/chessboard_center",
                         "/sdh_palm_link")  # right upper corner

        sss.move("torso", [positions[index]['torso_position']])
        sss.sleep(1)
        
        # CONTINUE HERE
        visible_response = visible()
        #pdb.set_trace()
        if visible_response.every:
            print "All Checkerboards found"
            capture_kinematics()                       ### Bool return value not handled    # Captures the joint values and image points
            capture_image()                            # Saves the image
            print "--> captured 1 sample for camera calibration"
            print "--> captured 1 sample for kinematics calibration"
            counter_camera += 1
            counter_kinematics += 1
        elif visible_response.master:               ### Doesn't make any sense
            print "Master Checkerboard found"
            capture_kinematics()
            print "--> captured 1 sample for kinematics calibration"
            counter_kinematics += 1

        msg.Camera_samples = counter_camera
        msg.Kinematic_samples = counter_kinematics
            #capture()

def data_capture_loop(group,visible,capture_kinematics,capture_image):
    '''
    @summary: the function takes a group and loops through the corresponding trajectories 
    which are auto-generated from generate_positions.py and iterates through them, it checks if a camera
    detects a CB pattern and if so saves the camera ID and Kinematics of the robot in a bag file by
    publishing to the /collect_data/capture using the service defined in collect_data
    @param group: this is the group for which the function will load the .yaml file
    and move the group
    @type group: moveit_commander.MoveGroupCommander
    @param visible: this is the the service used to check if the CB has been detected
    @param capture_image: this is the the service used to save images
    @param capture_kinematics: this is the the service used to save kinematics and image 
    '''
    # get position from parameter server
    trajectory_path_root = rospy.get_param('~trajectory_path_root', None)
    trajectory_path_of_group = trajectory_path_root+group.get_name()+"calibration_trajectories.yaml" #this is how the name is saved in generate_positions.py
    if trajectory_path_root is None:
        print "[ERROR]: no trajectory for %s set" %group.get_name()
        return
    with open(trajectory_path_of_group, 'r') as f:
        trajectory_of_group = yaml.load(f)
    print "YAML trajectory file loaded for "+group.get_name()
    progress_pub = rospy.Publisher(
        "/calibration/data_collection/progress", Progress)
#     br = tf.TransformBroadcaster()
    counter_calibration_data = 0
    msg = Progress()
    for i in xrange(len(trajectory_of_group)):
        group.execute(trajectory_of_group[i])
        msg.Percent_done = round(100.0 * i / len(trajectory_of_group), 2)
        msg.Samples_left = len(trajectory_of_group) - i
        progress_pub.publish(msg)
        print "Progress of current group is %s out of %s" %(i, len(trajectory_of_group))
        visible_response = visible()
        print "visibility check has finished"
        if visible_response.every:
            print "All Checkerboards found"
            capture_kinematics()                       ### Bool return value not handled    # Captures the joint values and image points
            capture_image()                            # Saves the image
            print "--> captured a sample"
            counter_calibration_data += 1
        elif True in visible_response.visibleImages:               ### Doesn't make any sense
            print "%s camera(s) have detected the CB" %visible_response.visibleImages.count(True)
            print (visible_response.visibleImages)
#             pdb.set_trace()
            capture_kinematics()
            capture_image()  
            counter_calibration_data += 1
            print "--> captured a sample"
        else:
            print "No camera has detected the CB"

        msg.Camera_samples = counter_calibration_data #redundant but taken from an old implementation
        msg.Kinematic_samples = counter_calibration_data
        
    print "%s has finished obtaining data from it's trajectories" %group.get_name()

def main():
    rospy.init_node(NODE)
    print "==> %s started " % NODE



    rospy.sleep(4)
    # service client
    checkerboard_checker_name = "/image_capture/visibility_check"
    visible = rospy.ServiceProxy(checkerboard_checker_name, Visible)
    rospy.wait_for_service(checkerboard_checker_name, 30)
    print "--> service client for for checking for chessboards initialized"

    kinematics_capture_service_name = "/collect_data/capture" # note the use of the collect_data and NOT image_capture this is used to fill the bag file
    capture_kinematics = rospy.ServiceProxy(
        kinematics_capture_service_name, Capture)
    rospy.wait_for_service(kinematics_capture_service_name, 6)
    print "--> service client for capture robot_states initialized"

    image_capture_service_name = "/image_capture/capture"
    capture_image = rospy.ServiceProxy(image_capture_service_name, Capture)
    rospy.wait_for_service(image_capture_service_name, 6)
    print "--> service client for capture images initialized"

    # init
    print "--> initializing sss"
    sss = simple_script_server()
    sss.init("base")
    sss.init("torso")
    sss.init("head")
    sss.recover("base")
    sss.recover("torso")
    sss.recover("head")
    sss.move("arm_left","home") #move to home position
    sss.move("arm_right","home")#move to home position
    sss.move("head", "back")    #move to calibration position
    rospy.wait_for_service("/move_group/plan_execution/set_parameters",timeout=30.0)
    moveit_commander.roscpp_initialize(sys.argv) # init of moveit
    robot = moveit_commander.RobotCommander()# init of moveit wrt robot
    arm_left_group = moveit_commander.MoveGroupCommander("arm_left") # define groups
    arm_right_group = moveit_commander.MoveGroupCommander("arm_right")
    arm_left_group.set_pose_reference_frame('torso_3_link')#*** Set referece
    arm_right_group.set_pose_reference_frame('torso_3_link')#*** Set referece
    arm_list = [arm_left_group,arm_right_group] # list of groups for which we want to test the trajectories used for calibration
    #scene = moveit_commander.PlanningSceneInterface() # init if scene if collisiton is to be avoided and for the addition of the cb pattern later
    print "--> setup care-o-bot for data capture"
    start = rospy.Time.now()
    for i in xrange(len(arm_list)):
        data_capture_loop(arm_list[i],visible,capture_kinematics,capture_image)
        sss.move(arm_list[i].get_name(),"home")
    print "finished after %s seconds" % (rospy.Time.now() - start).to_sec()
    sss.move("arm_left","home") #move to home position
    sss.move("arm_right","home")#move to home position
    
#     
#     # get position from parameter server
#     position_path = rospy.get_param('~position_path', None)
#     if position_path is None:
#         print "[ERROR]: no path for positions set"
#         return
#     with open(position_path, 'r') as f:
#         positions = yaml.load(f)
#     print "==> capturing samples"
#     start = rospy.Time.now()
#     capture_loop(positions, sss, visible, capture_kinematics, capture_image)
#     sss.move("arm", "calibration")
#     print "finished after %s seconds" % (rospy.Time.now() - start).to_sec()


if __name__ == '__main__':
    main()
    print "==> done exiting"
