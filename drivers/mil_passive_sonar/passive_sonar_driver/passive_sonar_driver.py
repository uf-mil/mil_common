#!/usr/bin/env python
from __future__ import division

import numpy as np
from mil_passive_sonar.msg import Ping
import rospy
from multilateration import Multilaterator, ls_line_intersection3d, get_time_delta
from mil_ros_tools.image_helpers import Image_Publisher
from mil_ros_tools.msg_helpers import numpy_to_point, numpy_to_vector3
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3Stamped

class PassiveSonar(object):
    '''
    Passive Sonar Driver: listens for a pinger and uses multilateration to locate it in a
        specified TF frame.

    Args:
    * input_mode - one of the following strings: ['serial', 'log', 'signal_cb']
        'serial'    - get signal input with a data acquistion board via a serial port
        'log'       - get signal input from a .npz file saved to disk
        'signal_cb' - get signal input from user provided function
        For more information on how to use each of these modes, read the wiki page.
    * signal_callback - Optional function used as a source of input if in 'signal_cb' mode

    This driver can work with an arbitrary receiver arrangement and with as many receivers as your
    heart desires (with n>=4).

    Multiple solvers are available for solving the multilateration problem, each with their pro's
    and cons.

    Services:
      * get_pulse_heading:
          Requests signals from the input source and calculates the relative heading to the pulse
      * estimate_pinger_location
          Estimates the position of a stationary pinger as the least_squares intersection of a set
          of 3D lines in a set TF frame.
      * set_frequency
          Set's the frequncy of the pinger that is being listened to.
      * reset_frequency_estimate
          Flushes all of the saved heading observations, and set's the postion estimate to NaN
      * start_logging
          Starts recording all of the received signals to an internal buffer.
      * save_log
          Dumps the buffer of recorded signals to a file.

    An visualization marker will be published to RVIZ for both the heading and the estimated pinger
    location.

    For more information, read the Passive Sonar wiki page of the mil_common repository.
    '''
    def __init__(self):
        self.load_params()
        self.ping_sub = rospy.Subscriber('ping', Ping, self.ping_cb, queue_size=10)
        self.heading_pub = rospy.Publisher('heading', Vector3Stamped, queue_size=5)
        self.multilaterator = Multilaterator(self.receiver_locations, self.speed_of_sound, "LS")
        self.plot_pub = Image_Publisher('~plot')
        self.rviz_pub = rospy.Publisher("~visualization", Marker, queue_size=10)

    def load_params(self):
        self.receiver_locations = np.array(rospy.get_param('~receiver_locations_mm'))
        self.receiver_count = self.receiver_locations.shape[0] + 1
        self.speed_of_sound = rospy.get_param('~speed_of_sound', 1.484)
        self.upsampling_factor = rospy.get_param('~upsampling_factor', 20)
        self.start = rospy.Time.now()

    def ping_cb(self, msg):
        '''
        TODO
        '''
        rospy.loginfo('Ping started at {}'.format((rospy.Time.now() - self.start).to_sec()))
        if msg.channels != self.receiver_count:
            rospy.logwarn('Received signal with {} channels when node was only configured for {} channels. Ignoring.'.format(msg.channels))
            return

        # Restructure signal data into a NumChannels x NumSamples array
        signals = np.array(msg.data).reshape((msg.samples, msg.channels)).T

        # Calculate the dtoa for the non-origin channels
        dtoa = self.get_dtoa(signals, msg.sample_rate)

        # Calculate the
        position = self.multilaterator.get_pulse_location(dtoa)
        heading = position / np.linalg.norm(position)

        heading_msg = Vector3Stamped()
        heading_msg.header = msg.header
        heading_msg.vector = numpy_to_vector3(heading)

        self.heading_pub.publish(heading_msg)
        self.visualize_heading(msg.header, heading)
        rospy.loginfo('Ping ended at {}'.format((rospy.Time.now() - self.start).to_sec()))


    def get_dtoa(self, signals, sampling_frequency):
        '''
        Returns a list of difference in time of arrival measurements for a signal between
        each of the non_reference hydrophones and the single reference hydrophone.

        signals - (self.receiver_count x self.signal_size) numpy array. It is assumed that the
            first row of this array is the reference signal.

        returns: list of <self.receiver_count> dtoa measurements in units of microseconds.
        '''
        sampling_T = 1.0 / sampling_frequency
        upsamp_T = sampling_T / self.upsampling_factor
        t_max = sampling_T * signals.shape[1]

        t = np.arange(0, t_max, step=sampling_T)
        t_upsamp = np.arange(0, t_max, step=upsamp_T)

        signals_upsamp = np.array([np.interp(t_upsamp, t, x) for x in signals])

        dtoa, cross_corr, t_corr = \
            map(np.array, zip(*[get_time_delta(t_upsamp, non_ref, signals_upsamp[0]) for non_ref \
                                in signals_upsamp[1 : self.receiver_count]]))

        t_corr = t_corr[0]  # should all be the same
        #self.visualize_dsp(t_upsamp, signals_upsamp, t_corr, cross_corr, dtoa)
        return dtoa

    def visualize_heading(self, header, vector, length=1.0):
        '''
        Publishes an arrow marker to RVIZ representing the heading towards the last heard ping.

        tail - 3x1 numpy array
        head - 3x1 numpy array
        lenth - scalar (float, int) desired length of the arrow marker. If None, length will
            be unchanged.
        '''
        tail = vector * length
        marker = Marker()
        marker.ns = "passive_sonar"
        marker.header = header
        # TODO: remove, debug line
        marker.header.frame_id = 'hydrophones'
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.points.append(numpy_to_point([0, 0, 0]))
        marker.points.append(numpy_to_point(tail))
        marker.color.r = 1.0
        marker.color.a = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.2
        self.rviz_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node("passive_sonar_driver")
    ping_ping_motherfucker = PassiveSonar()
    rospy.spin()

