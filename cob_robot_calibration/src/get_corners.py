#!/usr/bin/env python
#
# \file
#
# \note
#   Copyright (c) 2011-2012 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_calibration
# \note
#   ROS package name: cob_robot_calibration
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
#
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
#
PKG = 'cob_robot_calibration'
NODE = 'get_corners_node'
import roslib
roslib.load_manifest(PKG)
import rospy

import yaml
from sensor_msgs.msg import Image
from cob_calibration_msgs.msg import RobotMeasurement, ChainMeasurement, CameraMeasurement, ImagePoint
from cv_bridge import CvBridge

from cob_calibration_msgs.srv import Capture, CaptureResponse
from cob_camera_calibration import Checkerboard, CheckerboardDetector, cv2util

CHECKERBOARD_NAME = "cb_9x6"            ### Should be read from an yaml file
CHECKERBOARD_CHAIN = "arm_chain"        ### Should be read from an yaml file


class GetCbCorners():

    '''
    @summary:
    '''

    def __init__(self):
        '''
        Set up subscribers and local storage
        '''
        rospy.init_node(NODE)
        print "==> %s started " % NODE

        checkerboard = rospy.get_param("~calibration_pattern")
        self.checkerboard_square_size = checkerboard["square_size"]
        self.checkerboard_pattern_size = (
            int(checkerboard["pattern_size"].split("x")[0]),
            int(checkerboard["pattern_size"].split("x")[1]))
        with open(rospy.get_param("~sensors_yaml"), 'r') as a:
            sensors_yaml = yaml.load(a.read())
        # self._get_transformation_links(sensors_yaml)
        self._create_transformation_callbacks(sensors_yaml) # have a look at this
        #self.listener = tf.TransformListener()

       # CvBridge
        self.bridge = CvBridge()

        # initialize private storage
        self._images = {}
        self._images_received = {}

        #  init publisher / subscriber
        self._robot_measurement_pub = rospy.Publisher(
            "/robot_measurement", RobotMeasurement)
 

        # left camera
        rospy.Subscriber(
                         rospy.get_param("~cameras")["reference"]["topic"],
                         Image,
                         self._callback_image,
                         rospy.get_param("~cameras")["reference"]["name"]
                        )
        self._images[rospy.get_param("~cameras")["reference"]["name"]] = {}
        self._images_received[rospy.get_param("~cameras")["reference"]["name"]] = False
        for camera in rospy.get_param("~cameras")["further"]:
            rospy.Subscriber(
                             camera["topic"],
                             Image,
                             self._callback_image,
                             camera["name"]
                            )
            self._images[camera["name"]] = {}
            self._images_received[camera["name"]] = False

        print "==> done with initialization"

    def _callback_image(self, image_raw, id):
        '''
        Callback function for left camera message filter
        '''
        # print "DEBUG: callback left"
        self._images[id]["image"] = image_raw
        # if self._left_received == False:
            # print "--> left sample received (this only prints once!)"
        self._images_received[id] = True


    def get_corners(self):

        # --------------------
        # receive images
        # -----------------
        for v in self._images_received:
            self._images_received[v] = False

        start_time = rospy.Time.now()
        while (not all(self._images_received.values()) or not all(self.transformations.values())):
            rospy.sleep(0.005)
            # print warning every 2 seconds if one of the messages is still
            # missing...
            if start_time + rospy.Duration(2.0) < rospy.Time.now():
                for name, v in self._images_received.iteritems():
                    if not v:
                        print "--> still waiting for sample from %s"%name
                for name, v in self._transformations_received.iteritems():
                    if not v:
                        print "--> still waiting for sample from %s"%name
                start_time = rospy.Time.now()

        # set up checkerboard and checkerboard detector
        checkerboard = Checkerboard(pattern_size, square_size)
        checkerboard_detector = CheckerboardDetector(checkerboard)
        
        # detect cb
        for camera, image in self._images.iteritems():
            image = image["image"]
            cvImage = self.bridge.imgmsg_to_cv2(image, "mono8")
            #imagecv = cv2util.cvmat2np(cvImage)
            self._images[camera]['corners'] = None
            try:
                corners = checkerboard_detector.detect_image_points(cvImage, is_grayscale=True)
            except:
                rospy.logwarn("No calibration pattern found for: '%s'"%camera)
            else:
                print "cb found: %s"%camera
                #img_points = []
                #for (x, y) in corners.reshape(-1, 2):
                #    img_points.append(ImagePoint(x, y))
                self._images[camera]['corners'] = corners

        return self._images
