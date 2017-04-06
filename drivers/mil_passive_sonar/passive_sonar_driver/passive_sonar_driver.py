#!/usr/bin/env python
from __future__ import division

import numpy as np

import rospy
import tf2_ros
from tf import transformations
from multilateration import Multilaterator, ls_line_intersection3d, get_time_delta

import threading
import serial
import os

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from mil_passive_sonar.srv import *

import matplotlib.pyplot as plt

__author__ = 'David Soto'

# GLOBALS
lock = threading.Lock()  # prevent multiple threads simultaneously using a serial port
tf2_buf = tf2_ros.Buffer()

def thread_lock(lock):  # decorator
    '''
    Use an existing thread lock prevent a function from being executed by multiple threads at once
    '''
    def lock_thread(function_to_lock):
        def locked_function(*args, **kwargs):
            with lock:
                result = function_to_lock(*args, **kwargs)
        return locked_function
    return lock_thread

def getReceiverPose(time, receiver_array_frame, locating_frame):
    '''
    Gets the pose of the receiver array frame w.r.t. the locating_frame
    (usually /map or /world).
    
    Returns a 3x1 translation and a 3x3 rotation matrix (numpy arrays)
    '''
    try:
        tfl = tf2_ros.TransformListener(tf2_buf)
        T = tf2_buf.lookup_transform(receiver_array_frame, locating_frame, time,
                                              timeout=rospy.Duration(0.20))
        q = T.transform.rotation
        t = T.transform.translation
        rot = transformations.quaternion_matrix([q.w, q.x, q.y, q.z])
        trans = np.array([t.x, t.y, t.z])
        return trans, rot[:3,:3]
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.err(str(e))
        return None

class PassiveSonarInputSource(object):
    ready = False
    def input_request(self, callback):
        '''
        The driver will call this function to request signal and tf data.
        Inheriting classes SHOULD OVERRIDE this method and do the following:
        1) assign signals to self.signals
        2) assign tf translation to self.translation
        3) assign tf rotation to self.rotation
        4) assign True to self.ready
        '''
        pass

    def get_input(self):
        '''
        Returns a tuple with 3 things: signals, translation, rotation

        signals - a numpy array with a row for each receiver, and a column for each element of
            the recorded signal. The first row is reserved for the reference signal. The remaining
            rows should be in the same order as define in rosparam /passive_sonar/receiver_locations.
        translation - a 3 element 1D numpy array representing the position of the receiver array
            in <ref_frame>. The second element should be a
        rotation - 3x3 rotation matrix (numpy array) representing the orientation of the receiver
            array in <ref_frame>.

        Derived classes SHOULD NOT OVERRIDE this method.
        '''
        return self.signals, self.translation, self.rotation

    def reset(self):
        '''
        Driver will reset state when signals are received, could be used as feedback that signals
        were received.
        '''
        self.ready = False


class _SerialInput(PassiveSonarInputSource):
    def __init__(self, param_names, num_receivers):
        self.num_receivers = num_receivers
        load = lambda prop: setattr(self, prop, rospy.get_param('passive_sonar/' + prop))
        [load(x) for x in param_names]

        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=self.read_timeout)
            self.ser.flushInput()
        except serial.SerialException, e:
            rospy.err("Sonar serial connection error: " + str(e))
            raise e

    def input_request(self):
        try:
            self._request_signals()
            self._receive_signals()
            T = getReceiverPose(rospy.Time.now(), self.receiver_array_frame, self.locating_frame)
            self.translation, self.rotation = T
            self.ready = True
        except Exception as e:
            rospy.logerr(str(e))
            raise e
 
    def _request_signals(self):
        '''
        Request a set of digital signals from a serial port

        serial_port - open instance of serial.Serial
        data_request_code - char or string to sent to receiver board to signal a data request
        tx_start_code - char or string that the receiver board will send  us to signal the
            start of a transmission
        '''
        self.ser.flushInput()
        try:
            readin = None

            # Request raw signal tx until start bit is received
            while readin == None or ord(readin) != ord(self.tx_start_code):
                self.ser.write(self.tx_request_code)
                readin = self.ser.read(1)
                if len(readin) < len(self.tx_request_code):  # serial read timed out
                    raise IOError('Timed out waiting for serial response.')

        except serial.SerialException as e:
            rospy.logerr(str(e))
            return None

    def _receive_signals(self):
        '''
        Receives a set of 1D signals from a serial port and packs them into a numpy array

        serial_port - port of type serial.Serial from which to read
        signal_size - number of sacalar elements in each 1D signal
        scalar_size - size of each scalar in bytes
        signal_bias - value to be subtracted from each read scalar before packing into array

        returns: 2D numpy array with <num_signals> rows and <signal_size> columns
        '''
        self.signals = np.full((self.num_receivers, self.signal_size), self.signal_bias, dtype=float)

        for channel in range(self.num_receivers):
            for i in range(self.signal_size):
                while self.ser.inWaiting() < self.scalar_size:
                    pass
                self.signals[channel, i] = float(self.ser.read(self.scalar_size)) - self.signal_bias

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
    def __init__(self, input_mode='serial', custom_input_source=None):

        self.input_mode = input_mode
        self.load_params()
        self.bagging = False

        # TODO: update to use ros_alarms

        if self.input_mode == 'bag':
            self.input_source = _BagInput(self.input_src_params['bag'])

        elif self.input_mode == 'serial':
            self.input_source = _SerialInput(self.input_src_params['serial'], self.receiver_count)

        elif self.input_mode == 'custom_cb':
            if not issubclass(custom_input_source, PassiveSonarInputSource):
                raise RuntimeError('The custom input source provided ({}) is not derived from {}'
                    .format(type(custom_input_source), 'PassiveSonarInputSource'))
            self.input_source = custom_input_source

        else:
            raise RuntimeError(self.input_mode + 'is not a supported input mode')

        self.reset_position_estimate(None)

        self.multilaterator = Multilaterator(self.receiver_locations, self.c, self.method)

        self.rviz_pub = rospy.Publisher("/passive_sonar/rviz", Marker, queue_size=10)
        self.declare_services()
        rospy.loginfo('Passive sonar driver initialized')

    # Passive Sonar Helpers

    def load_params(self):
        '''
        Loads all the parameters needed for receiving and processing signals from the passive
        sonar board and calculating headings towards an active pinger.

        These parameters are descrived in detail in the Passive Sonar page of the mil_common wiki.
        TODO: copy url here
        '''
        # ROS params expected to be loaded under namespace passive_sonar
        self.required_params = ['receiver_locations', 'method', 'c', 'target_frequency',
                                'sampling_frequency', 'upsampling_factor', 'locating_frame',
                                'receiver_array_frame', 'min_variance', 'observation_buffer_size',
                                'input_timeout']
        self.input_src_params = \
        {
         'serial' : ['port', 'baud', 'tx_request_code', 'tx_start_code', 'read_timeout',
                     'scalar_size', 'signal_size', 'signal_bias', 'locating_frame',
                     'receiver_array_frame'],
         'bag'    : ['bag_filename' ,'locating_frame', 'receiver_array_frame']
        }

        load = lambda prop: setattr(self, prop, rospy.get_param('passive_sonar/' + prop))
        try:
            [load(x) for x in self.required_params]
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

    def bag_data(self, signals, trans, rot):
        if self.bagging:
            self.signals_bag.append(signals)
            self.trans_bag.append(trans)
            self.rot_bag.append(rot)

    def get_dtoa(self, signals):
        '''
        Returns a list of difference in time of arrival measurements for a signal between
        each of the non_reference hydrophones and the single reference hydrophone.

        signals - (self.receiver_count x self.signal_size) numpy array. It is assumed that the
            first row of this array is the reference signal.
        
        returns: list of <self.receiver_count> dtoa measurements in units of microseconds.
        '''
        plt.plot(signals.T)
        plt.show()
        sampling_T = 1.0 / self.sampling_frequency
        upsamp_T = sampling_T / self.upsampling_factor
        t_max = sampling_T * signals.shape[1]

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
        Returns the heading towards an active pinger emmiting at <self.target_frequency>.
        Heading will be a unit vector in hydrophone_array frame
        '''
        success = True
        err_str = ''
        try:
            time = rospy.Time.now()

            # Poll input source until it reports it is ready
            self.input_source.input_request()
            while not self.input_source.ready:
                if rospy.Time.now() - time > rospy.Duration(self.input_timeout):
                    raise IOError('Timed out waiting for input source')
                else:
                    rospy.sleep(0.05)
            self.input_source.reset()

            # Gather input from source
            signals, p0, R = self.input_source.get_input()

            # Carry out multilateration to get heading to pinger
            heading = self.multilaterator.getPulseLocation(self.get_dtoa(signals))
            heading = heading / np.linalg.norm(heading)

        except Exception as e:
            rospy.logerr(str(e))
            heading = np.full(3, np.NaN)
            success = False
            err_str = str(e)

        res = GetPulseHeadingResponse(header=Header(stamp=time, frame_id=self.locating_frame),
                                      x=heading[0], y=heading[1], z=heading[2],
                                      success=success, err_str=err_str)

        # Bag input if self.bagging == True
        self.bag_data(signals, p0, R)

        # Add heaing observation to buffers if the signals are above the variance threshold
        try:
            variance = np.var(signals)
            if variance > self.min_variance:
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
                    self.observation_variances = np.delete(self.observation_variances,
                                                           softest_idx, axis=0)
        except Exception as e:
            rospy.logwarn(str(e)) # Service should still return

        print "{} {}".format(type(res), res)
        return res
        print 'fuck'

    def estimate_pinger_position(self, req):
        '''
        Uses a buffer of prior observations to estimate the position of the pinger as the intersection
        of a set of 3d lines in the least-squares sense.
        '''
        assert len(self.heading_start) > 1
        p = ls_line_intersection3d(self.heading_start, self.heading_end)
        p = self.pinger_postion
        self.visualize_pinger_pos_estimate()
        return {'header' : {'stamp' : ros.Time.now(), 'frame_id' : self.locating_frame},
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
        self.bag_filename = req.filename if req.filename.endswith('.npz') else req.filename + '.npz'
        if not os.access(self.bag_filename, os.W_OK):
            raise IOError("Unable to write to file: " + self.bag_filename)
        self.bagging = True
        self.signal_bag = []
        self.rot_bag = []
        self.trans_bag = []

    def save_bag(self, req):
        try:
            pass
        except Exception as e:
            rospy.logerr(str(e))

    def set_frequency(self, req):
        '''
        Sets the assumed frequency (in absence of noise) of the signals to received by the driver
        '''
        self.target_frequency = req.frequency
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
        marker = Marker()
        marker.ns = "passive_sonar-{}".format(self.target_frequency)
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

    def visualize_heading(self, tail, head, bgra, length=1.0):
        '''
        Publishes an arrow marker to RVIZ representing the heading towards the last heard ping.

        tail - 3x1 numpy array
        head - 3x1 numpy array
        lenth - scalar (float, int) desired length of the arrow marker. If None, length will
            be unchanged.
        '''
        head = tail + (head - tail) * length
        head = Point(head[0], head[1], head[2])
        tail = Point(tail[0], tail[1], tail[2])
        marker = Marker()
        marker.ns = "passive_sonar-{}/heading".format(self.target_frequency)
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
    rospy.spin()

