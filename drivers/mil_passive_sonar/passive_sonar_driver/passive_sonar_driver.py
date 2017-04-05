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
from mil_msgs.srv import EstimatePingerPosition, GetPulseHeading, SetPassiveSonarFrequency
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Bool
from std_srvs.srv import Empty

import matplotlib.pyplot as plt

__author__ = 'David Soto'

# GLOBALS
lock = threading.Lock()  # prevent multiple threads simultaneously using a serial port

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

    np.savetxt("/home/sub8/passive_sonar_signals.txt", signals, delimiter=',')
    return signals

class PassiveSonar():
    '''
    Smart Passive Sonar driver that communicates with an external data acquisiton board through
    a serial port.

    This driver can work with an arbitrary receiver arrangement and with as many receivers as your
    heart desires (with n>=4).

    Multiple solvers are available for solving the multilateration problem, each with their pro's
    and cons.

    Services:
      * get_heading_to_pinger
      * estimate_pinger_location

    An visualization marker will be published to RVIZ for both the heading and the estimated pinger
    location.

    For more information, read the Passive Sonar wiki page of the mil_common repository.
    '''
    def __init__(self):
        rospy.init_node("passive_sonar")

        self.load_params()
        # TODO: update to use ros_alarms

        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=self.request_timeout)
            self.ser.flushInput()        
        except Exception, e:
            print "\x1b[31mSonar serial connection error:\n\t", e, "\x1b[0m"
            return None

        self.reset_position_estimate(None)

        self.tf_listener = tf.TransformListener()
        self.multilaterator = Multilaterator(self.receiver_locations, self.c, self.method)

        self.rviz_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        rospy.Service('passive_sonar/estimate_pinger_position', EstimatePingerPosition, self.estimate_pinger_position)
        rospy.Service('passive_sonar/get_pulse_heading', GetPulseHeading, self.get_pulse_heading)
        rospy.Service('passive_sonar/set_frequency', SetPassiveSonarFrequency, self.set_freq)
        rospy.Service('passive_sonar/reset_postion_estimate', Empty, self.reset_position_estimate)
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
        def load(prop):
            param_name = 'passive_sonar_driver/' + prop
            return rospy.get_param(param_name)

        locs = load('receiver_locations')
        self.receiver_count = len(locs) + 1
        self.receiver_locations = np.zeros((self.receiver_count - 1, 3), dtype=float)
        for i in range(self.receiver_count - 1):
            self.receiver_locations[i] = np.array([locs[i]['x'], locs[i]['y'], locs[i]['z']])

        self.method = load('method')
        self.c = load('c')
        self.port = load('port')
        self.baud = load('baud')
	self.TX_REQUEST = load('tx_request_code')
        self.TX_START = load('tx_start_code')
        self.request_timeout = load('request_timeout')
        self.target_freq = load('target_freq')
        self.scalar_sz = load('scalar_size')
        self.signal_sz = load('signal_size')
        self.signal_bias = load('signal_bias')
        self.samp_freq = load('sampling_freq')
        self.upsamp = load('upsample_factor')
        self.locating_frame = load('locating_frame')
        self.receiver_array_frame = load('receiver_array_frame')
        self.min_variance = load('min_variance')
        self.observation_buffer_size = load('observation_buffer_size')

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
        def make_response(header, v, success, err=''):
            if v is None:
               return make_response(header, np.full(3, np.NaN), success)
            return {'header' : header, 'x' : v[0], 'y' : v[1], 'z' : v[2], 'success' : success}

        self.ser.flushInput()
        header = make_header(frame=self.locating_frame)
        try:
            # Request raw signal tx until start bit is received
            readin = None
            timeout_ref = rospy.Time.now()
            while readin == None or ord(readin) != ord(self.TX_START):
                self.ser.write(self.TX_REQUEST)
                readin = self.ser.read(1)
                print "a"
                if readin == '':  # serial read timed out
                    return make_response(header, None, False, 'Timed out waiting for serial response.')
        except serial.SerialTimeoutException as e:
            print e
            print "b"
            return make_response(header, None, False, e.what())
        except serial.SerialException as e:
            print  e
            print "c"
            return make_response(header, None, False, e.what())

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
        return make_response(header, heading, False)

    def estimate_pinger_position(self, req):
        '''
        Uses a buffer of prior observations to estimate the position of the pinger as the intersection
        of a set of 3d lines in the least-squares sense.
        '''
        assert len(self.heading_start) > 1
        self.pinger_postion = ls_line_intersection3d(self.heading_start, self.heading_end)
        self.visualize_pinger_pos_estimate()
        pinger_position = numpy_to_point(self.pinger_position)
        return {'pinger_position' : pinger_position, 'num_samples' : len(self.heading_start)}

    def set_freq(self, req):
        '''
        Sets the assumed frequency (in absence of noise) of the signals to received by the driver
        '''
        self.target_freq = req.frequency
        self.heading_start = np.empty((0, 3), float)
        self.heading_end = np.empty((0, 3), float)
        self.sample_variances = np.empty((0, 1), float)
        self.pinger_position = np.array([np.NaN, np.NaN, np.NaN])
        return {}

    def reset_position_estimate(self, req):
        '''
        Clears all the heading and amplitude buffers and makes the position estimate NaN
        '''
	self.heading_start = np.empty((0, 3), float)
	self.heading_end = np.empty((0, 3), float)
        self.observation_variances = np.empty((0, 0), float)
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
    ping_ping_motherfucker = PassiveSonar()

