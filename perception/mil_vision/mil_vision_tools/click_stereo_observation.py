#!/usr/bin/env python
import cv2
import rospy
import numpy as np
import tf
from mil_ros_tools import Image_Publisher, Image_Subscriber, StereoImageSubscriber
from tf.transformations import quaternion_from_euler
from mil_ros_tools import numpy_pair_to_pose, numpy_to_vector3, numpy_to_point, numpy_to_quaternion, numpy_to_colorRGBA
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker
# Todo; r for reset, enter to confirm
class ClickStereoObservation(object):
	def __init__(self):
		self.tf_listener = tf.TransformListener()
		self.image_sub = StereoImageSubscriber('/camera/front/left/image_rect_color', '/camera/front/right/image_rect_color', slop=0.1)
		while(self.image_sub.last_image_left is None):
			rospy.sleep(0.5)
		self.image_info = self.image_sub.wait_for_camera_info()
		self.tf_listener.waitForTransform(
		                        '/map',
		                        '/front_left_cam',
		                        rospy.Time(),
								rospy.Duration(5))
		self.transform = self.tf_listener.lookupTransform('/map', '/front_left_cam', rospy.Time())
		self.img = np.concatenate((self.image_sub.last_image_left, self.image_sub.last_image_right), axis = 1)
		self.image_click_points = [[],[]]
		self.pub_marker = rospy.Publisher('vissss', Marker, queue_size=1)
		self.pub_point = rospy.Publisher('gate_point', PoseStamped, queue_size=1)

		self.run()

	def run(self):
		cv2.namedWindow('image')
		cv2.setMouseCallback('image', self.click_point_cb)
		while(1):
			cv2.imshow('image',self.img)
			k = cv2.waitKey(30)
			if k == 27:
			    break
		cv2.destroyWindow('image')
		print self.image_click_points
		self.get_3D_points()
	def click_point_cb(self, event, x, y, flags, param):
		if event == cv2.EVENT_LBUTTONDBLCLK:
			cv2.circle(self.img, (x,y), 5, (0,0,255), -1)
			if x > self.image_info[0].width:
				self.image_click_points[1].append((x % self.image_info[0].width ,y))
			else: self.image_click_points[0].append((x,y))

	def get_3D_points(self):
		left_pts = np.array(self.image_click_points[0], dtype=float).reshape(4, 2).T
		right_pts = np.array(self.image_click_points[1], dtype=float).reshape(4, 2).T
		print "Left: ", left_pts 
		print "Right: ", right_pts
		left_proj = np.array(self.image_sub.camera_info_left.P, dtype=float).reshape(3, 4)
		right_proj = np.array(self.image_sub.camera_info_right.P, dtype=float).reshape(3, 4)
		print 'left proj', left_proj
		print 'right_peoj', right_proj
		pts4D = cv2.triangulatePoints(left_proj.copy(), right_proj.copy(),left_pts.copy(), right_pts.copy())
		print "4d", pts4D
		pts3D = cv2.convertPointsFromHomogeneous(pts4D.T)
		pts3D = pts3D.reshape(4, 3)
		print pts3D
 		centroid = np.mean(pts3D, axis=0)
 		position_map = tf.transformations.quaternion_matrix(self.transform[1])[:3,:3].dot(centroid) + self.transform[0]

 		centroid_,quat_, _ = self.get_target_pose(pts3D)
 		print position_map
 		while (True):
 			pose = PoseStamped()
 			pose.header.frame_id = 'map'
 			pose.pose.position = numpy_to_point(position_map)
 			pose.pose.orientation = numpy_to_quaternion(quat_)

 			self.pub_point.publish(pose)
 			rospy.sleep(0.1)



	def get_target_pose(self, pts3D):
		'''
        Returns pose tutple (position, orientation) from
        approximate plane of  corner points pts3D
        @param pts3d 3D points
        '''
		pts3D_map = []
		for pt in pts3D:
			pts3D_map.append(tf.transformations.quaternion_matrix(self.transform[1])[:3,:3].dot(pt) + self.transform[0])
		pts3D_map = np.array(pts3D_map)
 		centroid = np.mean(pts3D_map, axis=0)
		pts3D = np.subtract(pts3D_map, centroid)
		A = np.c_[pts3D_map[:, 0], pts3D_map[:, 1], np.ones(pts3D_map.shape[0])]
		coeff, resid, _,_ = np.linalg.lstsq(A, pts3D_map[:, 2])
		norm = np.array([coeff[0], coeff[1], coeff[2]])

		#print 'RESID', resid
		yaw = np.arctan2(coeff[1], coeff[0])
		quat = quaternion_from_euler(0, 0, yaw)
		return centroid, quat, resid[0]

	def points_marker(self, pts, pose, id=0):
		marker = Marker()
		marker.header.frame_id = '/front_left_cam'
		marker.header.stamp = rospy.Time.now()
		marker.ns = 'torpedo_points'
		marker.type = Marker.POINTS
		marker.id = id
		marker.color = numpy_to_colorRGBA([1, 0, 0, 1])
		marker.scale.x = 0.05
		marker.scale.y = 0.05
		marker.frame_locked = True
		for pt in pts:
			marker.points.append(Point(pt[0], pt[1], pt[2]))
		self.pub_marker.publish(marker)
		marker.id = id + 100
		marker.type = Marker.ARROW
		marker.pose.position = numpy_to_point(pose[0])
		marker.pose.orientation = numpy_to_quaternion(pose[1])
		marker.points = []
		marker.scale.z = 0.05
		marker.scale.x = 0.5
		marker.scale.y = 0.05
		self.pub_marker.publish(marker)
		# pts3D = np.add(pts3D, self.tf_trans)
		# img = np.zeros((512,512,3), np.uint8)
		# cv2.namedWindow('image')
		# cv2.setMouseCallback('image',draw_circle)
		# last_image_l = self.image_sub_l.last_image
		# last_image_r = self.image_sub_r.last_image

		# self.tf_listener.waitForTransform(
		#                         '/map',
		#                         'front_stereo',
		#                         rospy.get_rostime(),
		#                         rospy.Duration(0.1))
		# (trans, rot) = self.tf_listener.lookupTransform('/map', 'front_stereo', rospy.get_rostime())

		# close the open windows

		# self.image_sub_r = Image_Subscriber('/camera/front/right/image_rect_color', self._img_callback_r)

	# def _img_callback_l(self, img):

	# # mouse callback function
	# def draw_circle(event,x,y,flags,param):
	#     if event == cv2.EVENT_LBUTTONDBLCLK:
	#         cv2.circle(img,(x,y),100,(255,0,0),-1)

	# img = np.zeros((512,512,3), np.uint8)
	# cv2.namedWindow('image')
	# cv2.setMouseCallback('image',draw_circle)
	# while(1):
	#     cv2.imshow('image',img)
	#     if cv2.waitKey(20) & 0xFF == 27:
	#         break
	# cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.init_node('click_stereo_observation')
	ClickStereoObservation()
	rospy.spin()
