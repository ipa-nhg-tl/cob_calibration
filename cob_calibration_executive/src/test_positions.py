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
#   Author: Jannik Abbenseth, email:jannik.abbenseth@gmail.com
# \author
#   Author: Sebastian Haug, email:sebhaug@gmail.com
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: January 2012
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
NODE = 'test_robot_calibration_data_node'
import roslib
roslib.load_manifest(PKG)
import rospy
import sys
from simple_script_server import simple_script_server
import yaml
import tf
import moveit_commander
import moveit_msgs.msg

import pdb # debugger

def capture_loop(trajectories, sss):
    '''
    Moves arm to all trajectories using moveit commander
    and calls capture() to capture samples
    @deprecated: This function is currently no longer being used in test_positions.py
    '''
    br = tf.TransformBroadcaster()
    for index in range(len(trajectories)):
        if trajectories[index]['torso_position'] == [0, 0, 0]:
            continue
        pre_signs=[a*b<0 for a,b in zip(trajectories[index]['joint_position'], trajectories[index-1]['joint_position'])]
        if any(pre_signs[0:3]):
            sss.move("arm","home")
        print "--> moving arm to sample #%s" % index
        pos = trajectories[index]
        joint_pos = [[a for a in trajectories[index]['joint_position']]]
        print pos
        nh = sss.move("arm", joint_pos)
        while nh.get_state() == 0:
            rospy.sleep(0.2)
        if nh.get_state() != 3:
            sss.move("torso", "home")
            nh = sss.move("arm", joint_pos)
            rospy.sleep(1)
            if nh.get_state() != 3:
                continue

        br.sendTransform((0, 0, 0.24),
                         (0, 0, 0, 1),
                         rospy.Time.now(),
                         "/chessboard_center",
                         "/sdh_palm_link")  # right upper corner

        sss.move("torso", [trajectories[index]['torso_position']])
def trajectory_test_loop (group):
    '''
    @summary: the function takes a group and finds the corresponding trajectroies 
    which are auto-generated from generate_positions.py and iterates through them
    @param group: this is the group for which the function will load the .yaml file
    and move the group
    @type group: moveit_commander.MoveGroupCommander
    '''
    # get position from parameter server
    trajectory_path_root = rospy.get_param('trajectory_path_root', None)
    trajectory_path_of_group = trajectory_path_root+group.get_name()+"calibration_trajectories.yaml" #this is how the name is saved in generate_positions.py
    if trajectory_path_root is None:
        print "[ERROR]: no trajectory for %s set" %group.get_name()
        return
    with open(trajectory_path_of_group, 'r') as f:
        trajectory_of_group = yaml.load(f)
    print "YAML trajectory file loaded for "+group.get_name()
    for i in xrange(len(trajectory_of_group)):
        group.execute(trajectory_of_group[i])
        print "Progress of current group is %s out of %s" %(i, len(trajectory_of_group))
    print "%s has finished testing it's trajectories" %group.get_name()
    

def main():
    rospy.init_node(NODE)
    print "==> %s started " % NODE

    ## service client
    #checkerboard_checker_name = "/image_capture/visibility_check"
    #visible = rospy.ServiceProxy(checkerboard_checker_name, Visible)
    #rospy.wait_for_service(checkerboard_checker_name, 2)
    #print "--> service client for for checking for chessboards initialized"
    #kinematics_capture_service_name = "/collect_data/capture"
    #capture_kinematics = rospy.ServiceProxy(
        #kinematics_capture_service_name, Capture)
    #rospy.wait_for_service(kinematics_capture_service_name, 2)
    #print "--> service client for capture robot_states initialized"
    #image_capture_service_name = "/image_capture/capture"
    #capture_image = rospy.ServiceProxy(image_capture_service_name, Capture)
    #rospy.wait_for_service(image_capture_service_name, 2)
    #print "--> service client for capture images initialize
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
    print "--> setup care-o-bot for capture"
    start = rospy.Time.now()
    for i in xrange(len(arm_list)):
        trajectory_test_loop(arm_list[i])
        sss.move(arm_list[i].get_name(),"home")
    print "finished after %s seconds" % (rospy.Time.now() - start).to_sec()
    sss.move("arm_left","home") #move to home position
    sss.move("arm_right","home")#move to home position

if __name__ == '__main__':
    main()
    print "==> done exiting"
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

