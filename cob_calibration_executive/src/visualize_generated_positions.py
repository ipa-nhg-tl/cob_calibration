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

def main():
    rospy.init_node(NODE)
    
    arm_list = ['arm_left','arm_right']
    dir = rospy.get_param('~trajectory_path_root', None)
    m_topic_positions = 'marker_array_cb_positions'
    marker_publisher_positions = rospy.Publisher(m_topic_positions, MarkerArray)
    marker_array_positions = MarkerArray()
    marker_id = 0
    for j, arm_name in enumerate(arm_list):
        file_path_positions = dir+arm_name+"_cb_positions.yaml"
        with open(file_path_positions, 'r') as f:
            cb_positions = yaml.load(f)
    
        reference_frame = cb_positions[-1]
        for i in range(0, len(cb_positions)-1):
            marker = visualize_markers(cb_positions[i][0], cb_positions[i][1], cb_positions[i][2], reference_frame, marker_id, [j, 0.5, 1.0], 0.05)
            marker_id += 1
            marker_array_positions.markers.append(marker)
    print 'Launch rviz now if it\'s not already open. In rviz add MarkerArray display and as a marker topic choose: %s' %(m_topic_positions)
    while not rospy.is_shutdown():
        marker_publisher_positions.publish(marker_array_positions)
        rospy.sleep(0.1)
    
if __name__ == '__main__':
    main()
    print "==> done exiting"
