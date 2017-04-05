#!/usr/bin/env python
from __future__ import division

import numpy as np
from sklearn.preprocessing import normalize

import rospy
import tf
from mil_ros_tools import thread_lock, make_header
from multilateration import Multilaterator, ls_line_intersection3d, get_time_delta

import threading
import serial

from visualization_msgs.msg import Marker
from mil_passive_sonar.srv import *
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Bool
from std_srvs.srv import Empty

import matplotlib.pyplot as plt

__author__ = 'David Soto'

# GLOBALS
lock = threading.Lock()  # prevent multiple threads simultaneously using a serial port

def request_signals(serial_port, data_request_code, tx_start_code):
    '''
    Request a set of digital signals from a serial port

    serial_port - open instance of serial.Serial
    data_request_code - char or string to sent to receiver board to signal a data request
    tx_start_code - char or string that the receiver board will send  us to signal the
        start of a transmission
    '''
    serial_port.flushInput()
    try:
        readin = None

        # Request raw signal tx until start bit is received
        while readin == None or ord(readin) != ord(tx_start_code):
            serial_port.write(data_request_code)
            readin = serial_port.read(1)
            if size(readin) < size(data_request_code):  # serial read timed out
                raise IOError('Timed out waiting for serial response.')

    except serial.SerialException as e:
        raise IOError(str(e))

def receive_signals(serial_port, num_signals, signal_size, scalar_size, signal_bias=0):
    '''
    Receives a set of 1D signals from a serial port and packs them into a numpy array

    serial_port - port of type serial.Serial from which to read
    signal_size - number of sacalar elements in each 1D signal
    scalar_size - size of each scalar in bytes
    signal_bias - value to be subtracted from each read scalar before packing into array

    returns: 2D numpy array with <num_signals> rows and <signal_size> columns
    '''
    signals = np.full((num_signals, signal_size), signal_bias, dtype=float)

    for channel in range(num_signals):
        for i in range(signal_size):
            while serial_port.inWaiting() < scalar_size:
                pass
            signals[channel, i] = float(serial_port.read(scalar_size)) - signal_bias

    return signals

class PassiveSonar(object):
    '''
    Passive Sonar Driver: listens for a pinger and uses multilateration to locate it in a
        specified TF frame.

    Args:
    * input_mode - one of the following strings: ['serial', 'bag', 'signal_cb']
        'serial'    - get signal input with a data acquistion board via a serial port
        'bag'       - get signal input from a .npz file saved to disk
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
      * start_bagging
          Starts recording all of the received signals to an internal buffer.
      * save_bag
          Dumps the buffer of recorded signals to a file.

    An visualization marker will be published to RVIZ for both the heading and the estimated pinger
    location.

    For more information, read the Passive Sonar wiki page of the mil_common repository.
    '''
    def __init__(self, input_mode='serial', signal_callback=None):

        self.input_mode = input_mode
        self.load_params()
        # TODO: update to use ros_alarms

        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=self.read_timeout)
            self.ser.flushInput()        
        except Exception, e:
            print "\x1b[31mSonar serial connection error:\n\t", e, "\x1b[0m"
            return None

        self.reset_position_estimate(None)

        self.tf_listener = tf.TransformListener()
        self.multilaterator = Multilaterator(self.receiver_locations, self.c, self.method)

        self.rviz_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        self.declare_services()
        print "\x1b[32mPassive sonar driver initialized\x1b[0m"
        rospy.spin()

    # Passive Sonar Helpers

    def load_params(self):
        '''
        Loads all the parameters needed for receiving and processing signals from the passive
        sonar board and calculating headings towards an active pinger.

        These parameters are descrived in detail in the Passive Sonar page of the mil_common wiki.
        TODO: copy url here
        '''
        # ROS params expected to be loaded under namespace passive_sonar
        required = ['receiver_locations', 'method', 'c', 'sampling_freq', 'upsampling_factor',
                    'locating_frame', 'receiver_array_frame', 'min_variance', 'observation_buffer_size']
        serial = ['port', 'baud', 'tx_request_code', 'tx_start_code', 'read_timeout', 'scalar_size', 
                  'signal_bias']
        bag = ['bag_filename']

        load = lambda prop: setattr(self, prop, rospy.get_param('passive_sonar/' + prop))
        try:
            [load(x) for x in required]
            if self.input_mode == 'serial':
                [load(x) for x in serial]
            if self.input_mode == 'bag':
                [load(x) for x in bag]
        except KeyError as e:
            raise IOError('A required rosparam was not declared: ' + str(e))

        self.receiver_count = len(self.receiver_locations) + 1
        self.receiver_locations = np.array(  # dictionary to numpy array
            [np.array([x['x'], x['y'], x['z']]) for x in self.receiver_locations])

    def declare_services(self):
       '''
       Conveniently declares all the services offered by the driver
       '''
       services = {
                      'get_pulse_heading'        : GetPulseHeading,
                      'estimate_pinger_position' : EstimatePingerPosition,
                      'reset_position_estimate'  : ResetPositionEstimate,
                      'start_bagging'            : StartBagging,
                      'save_bag'                 : SaveBag,
                      'set_frequency'            : SetFrequency
                  }
       [rospy.Service('passive_sonar/' + s[0], s[1], getattr(self, s[0])) for s in services.items()]

    def make_response(header, v, success, err=''):
        if v is None:
           return make_response(header, np.full(3, np.NaN), success)
        return {'header' : header, 'x' : v[0], 'y' : v[1], 'z' : v[2], 'success' : success}

    def getHydrophonePose(self, time):
        '''
        Gets the pose of the receiver array frame w.r.t. the <self.locating_frame>
        (usually /map or /world).
        
        Returns a 3x1 translation and a 3x3 rotation matrix (numpy arrays)
        '''
        try:
            self.tf_listener.waitForTransform(
                self.locating_frame, self.receiver_array_frame, time, timeout=rospy.Duration(0.25))
            trans, rot = self.tf_listener.lookupTransform(
                self.locating_frame, self.receiver_array_frame, time)
            rot = tf.transformations.quaternion_matrix(rot)
            return trans, rot[:3,:3]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print e
            raise RuntimeError("Unable to acquire TF for receiver array")

    def get_dtoa(self, signals):
        '''
        Returns a list of difference in time of arrival measurements for a signal between
        each of the non_reference hydrophones and the single reference hydrophone.

        signals - (self.receiver_count x self.signal_sz) numpy array. It is assumed that the
            first row of this array is the reference signal.
        
        returns: list of <self.receiver_count> dtoa measurements in units of microseconds.
        '''
        plt.plot(signals.T)
        plt.show()
        sampling_T = 1.0 / self.samp_freq
        upsamp_T = sampling_T / self.upsamp
        t_max = sampling_T * self.signal_sz

        t = np.arange(0, t_max, step=sampling_T)
        t_upsamp = np.arange(0, t_max, step=upsamp_T)

        signals_upsamp = [np.interp(t_upsamp, t, x) for x in signals]

        dtoa = [get_time_delta(t_upsamp, non_ref, signals[0]) \
                for non_ref in signals_upsamp[1 : self.receiver_count]]

        print "dtoa: {}".format(np.array(dtoa)*1E6)
        return dtoa

    #Passive Sonar Services

    @thread_lock(lock)
    def get_pulse_heading(self, srv):
        '''
        Returns the heading towards an active pinger emmiting at <self.target_freq>.
        Heading will be a unit vector in hydrophone_array frame
        '''

        self.ser.flushInput()
        header = self.make_header(frame=self.locating_frame)

        time = rospy.Time.now()
        signals = receive_signals(self.ser, self.receiver_count, self.signal_sz, 
                                  self.scalar_sz, self.signal_bias)
        heading = self.multilaterator.getPulseLocation(self.get_dtoa(signals))

        # Add heaing observation to buffers if the signals are above the variance threshold
        variance = np.var(signals)
        if variance > self.min_variance:
            p0, R = self.getHydrophonePose(time)
            map_offset = R.dot(heading)
            p1 = p0 + map_offset

            self.visualize_heading(p0, p1, bgra=[1.0, 0, 0, 0.50], length=4.0)

            self.heading_start = np.append(self.heading_start, np.array([p0]), axis=0)
            self.heading_end = np.append(self.heading_end, np.array([p1]), axis=0)
            self.observation_variances = np.append(self.observation_variances, variance)

            # delete softest samples if we have over max_observations
            if len(self.heading_start) >= self.observation_buffer_size:
                softest_idx = np.argmin(self.observation_variances)
                self.heading_start = np.delete(self.line_array, softest_idx, axis=0)
                self.heading_end = np.delete(self.line_array, softest_idx, axis=0)
                self.observation_variances = np.delete(self.observation_variances, softest_idx, axis=0)

        H = Vector3(*heading)
        print "heading: {}".format(heading)
        return self.make_response(header, heading, False)

    def estimate_pinger_position(self, req):
        '''
        Uses a buffer of prior observations to estimate the position of the pinger as the intersection
        of a set of 3d lines in the least-squares sense.
        '''
        assert len(self.heading_start) > 1
        p = ls_line_intersection3d(self.heading_start, self.heading_end)
        p = self.pinger_postion
        self.visualize_pinger_pos_estimate()
        return {'header' : make_header(frame=self.locating_frame),
                'num_headings' : len(self.heading_start),
                'x' : p[0], 'y' : p[1], 'z' : p[2]}

    def reset_position_estimate(self, req):
        '''
        Clears all the heading and amplitude buffers and makes the position estimate NaN
        '''
	self.heading_start = np.empty((0, 3), float)
	self.heading_end = np.empty((0, 3), float)
        self.observation_variances = np.empty((0, 0), float)
        self.pinger_position = np.array([np.NaN, np.NaN, np.NaN])
        return {}

    def start_bagging(self, req):
        pass

    def save_bag(self, req):
        pass

    def set_frequency(self, req):
        '''
        Sets the assumed frequency (in absence of noise) of the signals to received by the driver
        '''
        self.target_freq = req.frequency
        self.heading_start = np.empty((0, 3), float)
        self.heading_end = np.empty((0, 3), float)
        self.sample_variances = np.empty((0, 1), float)
        self.pinger_position = np.array([np.NaN, np.NaN, np.NaN])
        return {}

    # Passive Sonar RVIZ visualization
    def visualize_pinger_pos_estimate(self, bgra):
        '''
        Publishes a marker to RVIZ representing the last calculated estimate of the position of
        the pinger.

        rgba - list of 3 or 4 floats in the interval [0.0, 1.0] representing the desired color and
            transparency of the marker
        '''
        print "Visualizing Position"
        marker = Marker()
        marker.ns = "passive_sonar-{}".format(self.target_freq)
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = self.locating_frame
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        np.clip(bgra, 0.0, 1.0)
        marker.color.b = bgra[0]
        marker.color.g = bgra[1]
        marker.color.r = bgra[2]
        marker.color.a = 1.0 if len(bgra) < 4 else bgra[3]
        marker.pose.position = numpy_to_point(self.pinger_est_position)
        self.rviz_pub.publish(marker)
        print "position: ({p.x[0]:.2f}, {p.y[0]:.2f})".format(p=self.pinger_position)

    def visualize_heading(self, tail, head, bgra, length=None):
        '''
        Publishes an arrow marker to RVIZ representing the heading towards the last heard ping.

        tail - 3x1 numpy array
        head - 3x1 numpy array
        lenth - scalar (float, int) desired length of the arrow marker. If None, length will
            be unchanged.
        '''
        if length is not None:
          head = tail + (head - tail) / np.linalg.norm(head-tail) * length
        head = Point(head[0], head[1], head[2])
        tail = Point(tail[0], tail[1], tail[2])
        marker = Marker()
        marker.ns = "passive_sonar-{}/heading".format(self.target_freq)
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = self.locating_frame
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.points.append(tail)
        marker.points.append(head)
        marker.color.b = bgra[0]
        marker.color.g = bgra[1]
        marker.color.r = bgra[2]
        marker.color.a = 1.0 if len(bgra) < 4 else bgra[3]
        marker.scale.x = 0.1
        marker.scale.y = 0.2
        self.rviz_pub.publish(marker)

if __name__ == "__main__":
    rospy.init_node("passive_sonar_driver")
    ping_ping_motherfucker = PassiveSonar()

