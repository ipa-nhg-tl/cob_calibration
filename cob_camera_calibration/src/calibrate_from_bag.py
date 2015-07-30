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
#   ROS package name: cob_camera_calibration
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
import rospy

import os
import sensor_msgs.msg
import sensor_msgs.srv
from cob_calibration_msgs.msg._RobotMeasurement import RobotMeasurement
import functools
import rosbag
import numpy
from cv_bridge import CvBridge, CvBridgeError

import cv
import cv2

from calibrator import MonoCalibrator, StereoCalibrator, ChessboardInfo, Patterns
from sensor_msgs import *

NODE = 'monocalibrator'



def calibrate_mono(bag_file_dir, boards, cam_info_topics, pattern=Patterns.Chessboard, directory='/tmp', check_goodenough=True, min_samples=0):
    rospy.init_node(NODE)
    calib_flags = 0
    
    bag = rosbag.Bag(bag_file_dir)
    print bag_file_dir
    cameras = {}
    for topic, msg, t in bag.read_messages(topics=['/robot_measurement', 'robot_measurement']):
        if topic == "robot_measurement" or topic == "/robot_measurement":
            cam = msg.M_cam[0].camera_id
            if not cameras.has_key(cam):
                cameras[cam] = {}
                cameras[cam]['raw_images'] = []
                cameras[cam]['cv_images'] = []
                cameras[cam]['corners'] = []
            cameras[cam]['raw_images'].append(msg.M_cam[0].image)
    bag.close()
    
    for key in cameras.keys():
        if key in cam_info_topics:
            cameras[key]['cam_info_topic'] = cam_info_topics[key]
        else:
            raise NameError('No camera_info topic found for %s. Make sure that it is defined in cameras.yaml') %(key)
        cameras[key]['cam_info_service'] = rospy.ServiceProxy(cameras[key]['cam_info_topic'], sensor_msgs.srv.SetCameraInfo)
    
    for key, value in cameras.iteritems():
        cameras[key]['calibrator'] = MonoCalibrator(boards,           # Init the calibrator object for each camera
                                                    calib_flags, 
                                                    pattern, 
                                                    name=key)
    
    for key, value in cameras.iteritems():
        print key
        for img in cameras[key]['raw_images']:
            cameras[key]['calibrator'].handle_msg(img)      # Detects the calibration target, if found and provides enough new information, adds it to the sample database
    
    all_good = True
    some_good = False
    for key, value in cameras.iteritems():
        if cameras[key]['calibrator'].goodenough:
            cameras[key]['goodenough'] = True
            some_good = True
        else:
            cameras[key]['goodenough'] = False
            all_good = False
    
    if not all_good and some_good:
        print '\n'
        print '\033[1;31mNot enough calibration data for some of the camera(s):\033[1;m '
        for key in cameras.keys():
            if cameras[key]['goodenough'] is False:
                print key + ': ' + str(len(cameras[key]['calibrator'].good_corners)) + '   \033[1;31m[Not enough samples]\033[1;m'
            if cameras[key]['goodenough'] is True:
                print key + ': ' + str(len(cameras[key]['calibrator'].good_corners)) + '   \033[1;32m[Enough samples]\033[1;m'
        print '\n'
        if check_goodenough:
            raw_input('Press enter to continue the calibration for the cameras with enough samples...')
        else:
            raw_input('Note that \'check_goodenough\' variable is set to False, which means that the calibration parameters for the cameras with insufficient number of samples will also be calculated. Press enter to continue...')
            
        
    if not some_good:
        print '\n'
        print '\033[1;31mNot enough calibration data. More sample images are needed in order to calculate good calibration parameters.\033[1;m'
        print 'Current sample image numbers:'
        for key in cameras.keys():
            if cameras[key]['goodenough'] is False:
                print key + ': ' + str(len(cameras[key]['calibrator'].good_corners)) + '   \033[1;31m[Not enough samples]\033[1;m'
            if cameras[key]['goodenough'] is True:
                print key + ': ' + str(len(cameras[key]['calibrator'].good_corners)) + '   \033[1;32m[Enough samples]\033[1;m'
        print '\n'
        if check_goodenough:
            raise NameError('\033[1;31mNot enough calibration samples. Try capturing more sample images. Note that not only the number of images is important, but also the varying between the individual images. However if you want to continue the calibration anyway with this insufficient set of images, try to set the \'check_goodenough\' variable to False.\033[1;m')
  
    for key in cameras.keys():  
        if check_goodenough and cameras[key]['goodenough']:                ### Should we check goodenough?
            if len(cameras[key]['calibrator'].good_corners) > 0:
                cameras[key]['calibrator'].do_calibration()
                cameras[key]['calibrator'].do_save(directory, key)
        if not check_goodenough:
            if len(cameras[key]['calibrator'].good_corners) > 0:
                cameras[key]['calibrator'].do_calibration()
                cameras[key]['calibrator'].do_save(directory, key)
    
    print '\033[1;32mCalibration parameters saved in %s\033[1;m' %(directory)
    
    raw_input('Press enter to upload the new calibration parameters to the camera driver...')
    for key in cameras.keys():
        if not check_goodenough:
            cameras[key]['calibrator'].report()
            info = cameras[key]['calibrator'].as_message()
            response = cameras[key]['cam_info_service'](info)
            if response.success:
                print '\033[1;32mThe new parameters were successfully sent to camera_info for %s\033[1;m' %(key)
            else:
                print '\033[1;31mAttempt to send the new parameters to camera_info failed for %s\033[1;m' %(key)
            

def main():
    ptrn_size = []
    sqr_size = []
    bag_file_dir = '/tmp/cal/cal_measurements.bag'
    ptrn_size.append(rospy.get_param('/cob_camera_calibration/pattern_size'))
    sqr_size.append(rospy.get_param('/cob_camera_calibration/square_size'))
    check_goodenough = rospy.get_param('/cob_camera_calibration/check_goodenough', True)
    
    result_directory = '/tmp/cal/'
    
    cam_info_topics = {}
    cam_list = rospy.get_param(("/cob_camera_calibration/cameras"))
    print cam_list["reference"]["name"]
    cam_info_topics[cam_list["reference"]["name"]] = cam_list['reference']['cam_info_topic']
    for camera in cam_list["further"]:
        cam_info_topics[camera["name"]] = camera["cam_info_topic"]
    

    if len(ptrn_size) != len(sqr_size):
        parser.error("Number of size and square inputs must be the same!")

    boards = []
    for (sz, sq) in zip(ptrn_size, sqr_size):
        size = tuple([int(c) for c in sz.split('x')])
        boards.append(ChessboardInfo(size[0], size[1], float(sq)))

    pattern = Patterns.Chessboard

    calibrate_mono(bag_file_dir, boards, cam_info_topics, pattern, result_directory, check_goodenough)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        traceback.print_exc()
