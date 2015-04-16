#!/usr/bin/env python
import rospy
import sensor_msgs.msg
import sensor_msgs.srv
from cob_camera_calibration.srv import PermissionTrig
import message_filters
from camera_calibration.approxsync import ApproximateSynchronizer

import os
try:
    from queue import Queue
except ImportError:
    from Queue import Queue
import threading
import functools

import cv
import cv2

from calibrator import cvmat_iterator, MonoCalibrator, StereoCalibrator, ChessboardInfo, Patterns
from std_msgs.msg import String
from std_srvs.srv import Empty


#############
# TODO:
# - Add support for calibrating multiple cameras at once
# - Add stereo calibration support
# - 
#############


class ConsumerThread(threading.Thread):
    def __init__(self, queue, function):
        threading.Thread.__init__(self)
        self.queue = queue
        self.function = function

    def run(self):
        while True and not rospy.is_shutdown():
            while True and not rospy.is_shutdown():
                m = self.queue.get()
                if self.queue.empty():
                    break
            self.function(m)


class CalibrationNode:
    def __init__(self, boards, camera_namespace, image_topic, synchronizer = message_filters.TimeSynchronizer, pattern=Patterns.Chessboard, directory='/tmp', min_samples=0):
        
        fullservicename = "%s/set_camera_info" % camera_namespace
        print "Waiting for service %s ..." %(fullservicename)
        try:
            rospy.wait_for_service(fullservicename, 5)
            print("OK")
        except rospy.ROSException:
            print("Service not found")
            rospy.signal_shutdown('Quit')
        
        
        #if service_check:
            ## assume any non-default service names have been set.  Wait for the service to become ready
            #for svcname in ["camera", "left_camera", "right_camera"]:
                #remapped = rospy.remap_name(svcname)
                #if remapped != svcname:
                    #fullservicename = "%s/set_camera_info" % remapped
                    #print "Waiting for service %s ..." %(fullservicename)
                    #try:
                        #rospy.wait_for_service(fullservicename, 5)
                        #print("OK")
                    #except rospy.ROSException:
                        #print("Service not found")
                        #rospy.signal_shutdown('Quit')
        
        # Minimum number of samples to capture
        if min_samples <= 0:
            self.min_samples_number = float('inf')
        else:
            self.min_samples_number = min_samples
        self._boards = boards
        self._calib_flags = 0
        self._pattern = pattern
        self._camera_name = ''      # Delete?
        self._directory = directory
        lsub = message_filters.Subscriber('left', sensor_msgs.msg.Image)
        rsub = message_filters.Subscriber('right', sensor_msgs.msg.Image)
        ts = synchronizer([lsub, rsub], 4)
        ts.registerCallback(self.queue_stereo)

        msub = message_filters.Subscriber(image_topic, sensor_msgs.msg.Image)
        msub.registerCallback(self.queue_monocular)
        
        self.set_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % camera_namespace,
                                                          sensor_msgs.srv.SetCameraInfo)
        self.set_left_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("left_camera"),
                                                               sensor_msgs.srv.SetCameraInfo)
        self.set_right_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("right_camera"),
                                                                sensor_msgs.srv.SetCameraInfo)

        self.q_mono = Queue()
        self.q_stereo = Queue()

        self.c = None
        self.captured_images_count = 0
        self.capture_img_permission = False
        self.image_captured = False
        self.image_captured_server()
        
        mth = ConsumerThread(self.q_mono, self.handle_monocular)
        mth.setDaemon(True)
        mth.start()

        sth = ConsumerThread(self.q_stereo, self.handle_stereo)
        sth.setDaemon(True)
        sth.start()
        
        self.enough_samples = False
        self.check_goodenough()
        
        rospy.signal_shutdown('Quit')
            
        
    def redraw_stereo(self, *args):
        pass
    def redraw_monocular(self, *args):
        pass

    def queue_monocular(self, msg):
        self.q_mono.put(msg)

    def queue_stereo(self, lmsg, rmsg):
        self.q_stereo.put((lmsg, rmsg))

    def handle_monocular(self, msg):
        if self.c == None:
            if self._camera_name:
                self.c = MonoCalibrator(self._boards, self._calib_flags, self._pattern, name=self._camera_name)
            else:
                self.c = MonoCalibrator(self._boards, self._calib_flags, self._pattern)
        if self.min_samples_number >= self.captured_images_count and self.c.goodenough:
            self.enough_samples = True
        if not self.capture_img_permission and not self.enough_samples:
            self.permission_query()
        if self.capture_img_permission and not self.image_captured:
            # This should just call the MonoCalibrator
            drawable = self.c.handle_msg(msg)
            self.displaywidth = drawable.scrib.cols
            #self.redraw_monocular(drawable)
            if len(self.c.db) > self.captured_images_count:
                print 'len(self.c.db): %s    pic_count: %s' %(len(self.c.db), self.captured_images_count)
                print 'Image captured'
                self.captured_images_count += 1
                self.capture_img_permission = False
                self.image_captured = True
    
    def check_goodenough(self):
        while not self.enough_samples:
            pass
        raw_input('Enough samples captured. Press Enter to calculate the calibration parameters...')
        self.c.do_calibration()
        self.c.do_save(self._directory)
        print 'Calibration finished.'
        raw_input('Press Enter to upload the new camera parameters to the camera driver...')
        self.do_upload()
    
    def permission_query(self):
        rospy.sleep(0.5)
        try:
            rospy.wait_for_service('capture_img_permission', 5)
        except:
            print("Service not found")
            rospy.signal_shutdown('Quit')
        query = rospy.ServiceProxy('capture_img_permission', PermissionTrig)
        print 'Waiting for a permission to capture sample image...'
        try:
            resp = query()
            if resp.capture_img == True:
                print 'Permission given'
                self.capture_img_permission = True
        except rospy.ServiceException as exc:
            print str(exc)
            rospy.signal_shutdown('Quit')
        self.image_captured = False

    def handle_stereo(self, msg):
        if self.c == None:
            if self._camera_name:
                self.c = StereoCalibrator(self._boards, self._calib_flags, self._pattern, name=self._camera_name)
            else:
                self.c = StereoCalibrator(self._boards, self._calib_flags, self._pattern)

        drawable = self.c.handle_msg(msg)
        self.displaywidth = drawable.lscrib.cols + drawable.rscrib.cols
        self.redraw_stereo(drawable)
            
 
    def check_set_camera_info(self, response):
        if response.success:
            return True

        for i in range(10):
            print("!" * 80)
        print()
        print("Attempt to set camera info failed: " + response.status_message)
        print()
        for i in range(10):
            print("!" * 80)
        print()
        rospy.logerr('Unable to set camera info for calibration. Failure message: %s' % response.status_message)
        return False

    def do_upload(self):
        self.c.report()
        print(self.c.ost())
        info = self.c.as_message()

        rv = True
        if self.c.is_mono:
            response = self.set_camera_info_service(info)
            rv = self.check_set_camera_info(response)
        else:
            response = self.set_left_camera_info_service(info[0])
            rv = rv and self.check_set_camera_info(response)
            response = self.set_right_camera_info_service(info[1])
            rv = rv and self.check_set_camera_info(response)
        return rv
        
    def handle_image_captured(self, req):
        return self.image_captured
        
    def image_captured_server(self):
        s = rospy.Service('image_captured', PermissionTrig, self.handle_image_captured)
        print "image_captured_server up"



def main():
    
    ptrn_size = []
    sqr_size = []
    ptrn_size.append(rospy.get_param('~/intrinsic_camera_calibration/camera_0/checkerboard_pattern_size'))
    sqr_size.append(rospy.get_param('~/intrinsic_camera_calibration/camera_0/checkerboard_square_size'))
    result_directory = rospy.get_param('~/intrinsic_camera_calibration/camera_0/output_dir')
    camera_namespace = rospy.get_param('~/intrinsic_camera_calibration/camera_0/topic_camera')
    image_topic = rospy.get_param('~/intrinsic_camera_calibration/camera_0/topic_image_raw')
    try:
        min_samples = rospy.get_param('~/intrinsic_camera_calibration/camera_0/minimum_number_of_samples')
    except:
        min_samples = 0

    if len(ptrn_size) != len(sqr_size):
        parser.error("Number of size and square inputs must be the same!")


    boards = []
    for (sz, sq) in zip(ptrn_size, sqr_size):
        size = tuple([int(c) for c in sz.split('x')])
        boards.append(ChessboardInfo(size[0], size[1], float(sq)))

    sync = message_filters.TimeSynchronizer


    pattern = Patterns.Chessboard


    rospy.init_node('cameracalibrator')
    node = CalibrationNode(boards, camera_namespace, image_topic, sync, pattern, result_directory, min_samples)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        traceback.print_exc()
