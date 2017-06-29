#!/usr/bin/env python

import numpy as np
import cv2
import sys
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

class Kinect2ImageRetriever:
	def __init__(self):
		self.enable_rgb = True
		self.enable_depth = True

		try:
		    from pylibfreenect2 import OpenCLPacketPipeline
		    self.pipeline = OpenCLPacketPipeline()
		except:
		    try:
		        from pylibfreenect2 import OpenGLPacketPipeline
		        self.pipeline = OpenGLPacketPipeline()
		    except:
		        from pylibfreenect2 import CpuPacketPipeline
		        self.pipeline = CpuPacketPipeline()
		print("Packet pipeline:", type(self.pipeline).__name__)

		rospy.init_node('RGBDImagePublisher', anonymous=True)
		self.color_image_pub = rospy.Publisher('ColorImage', Image, queue_size=10)
		self.depth_image_pub = rospy.Publisher('DepthImage', Image, queue_size=10)

	# Returns the registered frame for the
	# input stream of depth and color images
	def publishDepth(self, undistorted):
		depth_image_msg = CvBridge().cv2_to_imgmsg(undistorted.asarray(np.float32) / 4500.)
		self.depth_image_pub.publish(depth_image_msg, "mono8")
	
	def publishColor(self, registered):
		color_image_msg = CvBridge().cv2_to_imgmsg(registered.asarray(np.uint8))
		self.color_image_pub.publish(color_image_msg, "RGB8")


	def publishRGBD(self):
		fn = Freenect2()
		num_devices = fn.enumerateDevices()
		
		#Exit if device is not found
		if num_devices == 0:
		    print("No device connected!")
		    sys.exit(1)

		serial = fn.getDeviceSerialNumber(0)
		device = fn.openDevice(serial, pipeline=self.pipeline)

		types = 0
		if self.enable_rgb:
		    types |= FrameType.Color
		if self.enable_depth:
		    types |= (FrameType.Ir | FrameType.Depth)
		listener = SyncMultiFrameListener(types)

		# Register listeners
		device.setColorFrameListener(listener)
		device.setIrAndDepthFrameListener(listener)

		if self.enable_rgb and self.enable_depth:
		    device.start()
		else:
		    device.startStreams(rgb=self.enable_rgb, depth=self.enable_depth)

		# Should be transformed to manually calibrated values.
		if self.enable_depth:
		    registration = Registration(device.getIrCameraParams(),
		                                device.getColorCameraParams())

		undistorted = Frame(512, 424, 4)
		registered = Frame(512, 424, 4)

		while not rospy.is_shutdown():
		    frames = listener.waitForNewFrame()

		    if self.enable_rgb:
		        color = frames["color"]
		    if self.enable_depth:
		        ir = frames["ir"]
		        depth = frames["depth"]

		    if self.enable_rgb and self.enable_depth:
		        registration.apply(color, depth, undistorted, registered)
		    elif enable_depth:
		        registration.undistortDepth(depth, undistorted)

		    if self.enable_depth:
		        #cv2.imshow("undistorted", undistorted.asarray(np.float32) / 4500.)
		        self.publishDepth(undistorted)
		        
		    if self.enable_rgb and self.enable_depth:
		        #cv2.imshow("registered", registered.asarray(np.uint8))
		        self.publishColor(registered)

		    listener.release(frames)

		device.stop()
		device.close()

		sys.exit(0)
