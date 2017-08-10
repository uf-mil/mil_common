#!/usr/bin/env python
import cv2
import rospy
import tf
from tf.transformations import quaternion_from_euler
import numpy as np
from mil_ros_tools import Image_Subscriber, StereoImageSubscriber
import image_geometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from mil_ros_tools import numpy_to_point, numpy_to_quaternion, numpy_to_colorRGBA
import argparse

'''
TODO: 
		Numbering for clicked points in stereo. 
		Make stereo work outside of class
		Comment stereo code
		Find full pose in stereo
		Fix wait in stereo
		Use image time for finding tf
'''


def open_window(img, callback):
    cv2.namedWindow('click_image')
    cv2.setMouseCallback('click_image', callback)
    while(1):
        cv2.imshow('click_image', img)
        k = cv2.waitKey(30)
        if k == 27:
            break
    cv2.destroyWindow('click_image')


class SingleCameraClickedPoints(object):
    '''
    Class to power single camera clicked-point-callback and provide members to give projected rays
    and their visualizations.
    '''

    def __init__(self, camera_topic='/camera/front/left/image_rect_color', tf_frame='/front_left_cam'):
        '''
        Obtain a "snapshot" in time which includes: tf, image, and camera intrinsics
        Open cv window for clicking points
        '''
        self.tf_listener = tf.TransformListener()
        self.tf_frame = tf_frame
        self.image_sub = Image_Subscriber(camera_topic)
        self.pub_clicked_points = rospy.Publisher(
            'click_points_ray', MarkerArray, queue_size=1)
        self.markers = MarkerArray()

        self.setup_snapshot()

        self.clicked_points = []
        open_window(self.img, self.click_point_cb)
        if len(self.clicked_points) == 0:
            raise Exception('Need at least a single clicked point')

    def setup_snapshot(self, timeout=10):
        '''
        Obtain a "snapshot" in time which includes: tf, image, and camera intrinsics
        '''
        # Wait until recieved camera info
        self.image_info = self.image_sub.wait_for_camera_info()

        timeout = rospy.Duration(timeout)

        # Sleep until recieved an image
        while (self.image_sub.last_image is None) and (rospy.Time.now() - start_time < timeout) and (not rospy.is_shutdown()):
            rospy.sleep(1)
        if self.image_sub.last_image is None:
            raise Exception('Failed to obtain an image')

        # Obtaim camera instrincs from camera info
        cam_mod = image_geometry.PinholeCameraModel()
        cam_mod.fromCameraInfo(self.image_info)
        self.K = np.array(cam_mod.fullIntrinsicMatrix(), dtype=np.float32)
        self.K_inv = np.linalg.inv(self.K)

        # Wait and obtain transform
        self.tf_listener.waitForTransform(
            '/map',
            self.tf_frame,
            rospy.Time(),
            rospy.Duration(5))
        self.transform = self.tf_listener.lookupTransform(
            '/map', self.tf_frame, rospy.Time())

        self.img = self.image_sub.last_image

    def click_point_cb(self, event, x, y, flags, param):
        '''Callback for CV to use and remember the points clicked'''
        if event == cv2.EVENT_LBUTTONDBLCLK:
            self.clicked_points.append((x, y))
            cv2.circle(self.img, (x, y), 5, (0, 0, 255), -1)

    def get_rays(self):
        '''Find projection rays of click points using camera intrinsics'''
        rays = []
        for point in self.clicked_points:
            v = np.array(point)
            pt = np.reshape(v, (len(v), 1))
            vec = self.K_inv.dot((np.vstack([pt, 1.0])))
            rays.append(vec)  # TODO
        return np.array(rays)

    def fill_rays_marker(self, rays):
        '''
        Function to populate markerarray with arrow markers
        @param rays : array of rays in camera frame
        '''
        for idx, ray in enumerate(rays):
            marker = Marker()
            marker.header.frame_id = self.tf_frame
            marker.ns = 'ray_markers'
            marker.header.stamp = rospy.Time.now()
            marker.type = Marker.ARROW
            marker.id = idx
            marker.color.r = 1
            marker.color.g = 0
            marker.color.b = 0
            marker.color.a = 1
            marker.scale.x = 0.01
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            marker.action = Marker.ADD

            begin_pt = np.array([[ray[0][0]], [ray[1][0]], [0]])

            marker.points.append(numpy_to_point(begin_pt))
            marker.points.append(numpy_to_point(ray * 10 + begin_pt))
            self.markers.markers.append(marker)

    def publish_markers(self):
        '''Typedef-like function to publish the markers'''
        self.pub_clicked_points.publish(self.markers)


class StereoClickedPoints(object):
    def __init__(self, left_camera_topic='/camera/front/left/image_rect_color', right_camera_topic='/camera/front/right/image_rect_color', tf_frame='/front_left_cam'):
        self.tf_listener = tf.TransformListener()
        self.tf_frame = tf_frame
        self.image_sub = StereoImageSubscriber(
            left_camera_topic, right_camera_topic, slop=0.1)
        self.pub_clicked_points = rospy.Publisher(
            'click_points_ray', MarkerArray, queue_size=1)
        self.markers = MarkerArray()

        self.setup_snapshot()

        self.clicked_points = [[], []]
        open_window(self.img, self.click_point_cb)
        if len(self.clicked_points) == 0:
            raise Exception('Need at least a single clicked point')
        # x = self.calc_rays()
        # self.draw_ray_rviz()
        pts = self.get_3D_points()
        centoid, quat, _ = self.get_target_pose(pts)

        self.rviz_pts3D_map(pts)
        self.rviz_pose(centoid, quat)
        self.pub_clicked_points.publish(self.markers)

    def setup_snapshot(self):
        while(self.image_sub.last_image_left is None or self.image_sub.last_image_right is None):
            rospy.sleep(1)
        self.image_info = self.image_sub.wait_for_camera_info()
        self.tf_listener.waitForTransform(
            '/map',
            self.tf_frame,
            rospy.Time(),
            rospy.Duration(5))
        self.transform = self.tf_listener.lookupTransform(
            '/map', self.tf_frame, rospy.Time())
        self.img = np.concatenate(
            (self.image_sub.last_image_left, self.image_sub.last_image_right), axis=1)

    def click_point_cb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            cv2.circle(self.img, (x, y), 5, (0, 0, 255), -1)
            if x > self.image_info[0].width:
                self.clicked_points[1].append(
                    (x % self.image_info[0].width, y))
            else:
                self.clicked_points[0].append((x, y))

    def get_3D_points(self):
        if not (len(self.clicked_points[0]) == len(self.clicked_points[1])):
            raise Exception(
                'Must have same number of clicked points in both cameras')
        left_pts = np.array(self.clicked_points[0], dtype=float).reshape(
            len(self.clicked_points[0]), 2).T
        right_pts = np.array(self.clicked_points[1], dtype=float).reshape(
            len(self.clicked_points[1]), 2).T
        left_proj = np.array(
            self.image_sub.camera_info_left.P, dtype=float).reshape(3, 4)
        right_proj = np.array(
            self.image_sub.camera_info_right.P, dtype=float).reshape(3, 4)
        pts4D = cv2.triangulatePoints(
            left_proj.copy(), right_proj.copy(), left_pts.copy(), right_pts.copy())
        pts3D = cv2.convertPointsFromHomogeneous(pts4D.T)
        pts3D = pts3D.reshape(len(pts3D), 3)
        # print self.pts3D
        pts3D_map = []
        for pt in pts3D:
            pts3D_map.append(tf.transformations.quaternion_matrix(
                self.transform[1])[:3, :3].dot(pt) + self.transform[0])
        pts3D_map = np.array(pts3D_map)
        return pts3D_map

    def get_target_pose(self, pts3D_map):
        '''
Returns pose tutple (position, orientation) from
approximate plane of  corner points pts3D
@param pts3d 3D points
'''
        centroid = np.mean(pts3D_map, axis=0)
        pts3D = np.subtract(pts3D_map, centroid)
        A = np.c_[pts3D_map[:, 0], pts3D_map[:, 1],
                  np.ones(pts3D_map.shape[0])]
        coeff, resid, _, _ = np.linalg.lstsq(A, pts3D_map[:, 2])
        norm = np.array([coeff[0], coeff[1], coeff[2]])

        yaw = np.arctan2(coeff[1], coeff[0])
        quat = quaternion_from_euler(0, 0, yaw)
        return centroid, quat, resid[0]

    def rviz_pts3D_map(self, pts):
        marker = Marker()
        marker.header.frame_id = '/map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'clicked_points_3d'
        marker.type = Marker.POINTS
        marker.color = numpy_to_colorRGBA([1, 0, 0, 1])
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.frame_locked = True
        for pt in pts:
            marker.points.append(numpy_to_point(pt))
        self.markers.markers.append(marker)

    def rviz_pose(self, centroid, quat):
        marker = Marker()
        marker.header.frame_id = '/map'
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.ARROW
        marker.ns = 'clicked_points_pose'

        marker.pose.position = numpy_to_point(centroid)
        marker.pose.orientation = numpy_to_quaternion(quat)
        marker.color = numpy_to_colorRGBA([0, 1, 0, 1])

        marker.scale.z = 0.05
        marker.scale.x = 0.5
        marker.scale.y = 0.05
        self.markers.markers.append(marker)

    def publish_markers(self):
        '''Typedef-like function to publish the markers'''
        self.pub_clicked_points.publish(self.markers)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-s', '--use_stereo', help='Use stereo for clicked points0', action='store_true')
    args = parser.parse_args()

    rospy.init_node('click_stereo_observation')
    clicked_points = None
    if (args.use_stereo):
        clicked_points = StereoClickedPoints()
    else:
        clicked_points = SingleCameraClickedPoints()
        rays = clicked_points.get_rays()
        clicked_points.fill_rays_marker(rays)
    clicked_points.publish_markers()
    rospy.spin()
