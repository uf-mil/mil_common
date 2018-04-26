#!/usr/bin/env python
from __future__ import division

import rospy
from sensor_msgs.msg import Image, CameraInfo
import os

class DownCamRestart():
	def __init__(self):
		self.subscribe_down = rospy.Subscriber('/camera/down/left/image_rect_color', Image, self.img_callback, queue_size=1)
		self.last_time = rospy.Time.now()
		self.timer = rospy.Timer(rospy.Duration(1), self.check)
		self.restarting = (None, False)

	def check(self, t):
		if (self.restarting[1]):
			if(rospy.Time.now() - self.restarting[0] > 5):
				self.restarting[1] = False
		if (rospy.Time.now() - self.last_time > rospy.Duration(2) and not self.restarting[1]):
			self.restarting[1] = True
			self.restarting[0] = rospy.Time.now()
			retval = os.system('roslaunch sub8_launch down_cameras.launch')

	def img_callback(self, data):
		self.last_time = rospy.Time.now()
		# self.subscribe_down = rospy.Subscriber(root_topic + '/camera_info', CameraInfo, self.info_cb, queue_size=queue_size)

if __name__ == '__main__':
	rospy.init_node('down_cam_restart')
	DownCamRestart()
	rospy.spin()
