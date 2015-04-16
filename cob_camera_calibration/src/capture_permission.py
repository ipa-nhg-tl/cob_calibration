#!/usr/bin/env python

# Imports
import rospy
from cob_calibration.srv import PermissionTrig
from dialog_client import *

# This script is used to manually give a permission for the 
# cameracalibrator to capture an image. The permission is given by 
# pressing 'Enter' on the terminal window while running this script.


class CapturePermission():
	
	def handle_capture_permission(self, req):
		return self.permission

	def capture_permission_server(self):
		s = rospy.Service('capture_img_permission', PermissionTrig, self.handle_capture_permission)
		print "Capture permission server up"
		
	def run(self):
		rospy.init_node('capture_permission_server')
		self.permission = False
		self.capture_permission_server()
		while not rospy.is_shutdown():
			self.permission = False
			raw_input('Press Enter to take an image')
			self.permission = True
			print 'Capturing image...'
			
			captured = False
			while not captured and not rospy.is_shutdown():
				rospy.wait_for_service('image_captured')
				query = rospy.ServiceProxy('image_captured', PermissionTrig)
				resp = query()
				if resp.capture_img == True:
					captured = True
					self.permission = False
				
			print 'Move to a new position'


if __name__ == "__main__":
	try:
		node = CapturePermission()
		node.run()
	except KeyboardInterrupt, e:
		pass
	print "exiting"
