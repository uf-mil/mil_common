#!/usr/bin/env python
from __future__ import division

import numpy as np

import rospy
import tf2_ros
from tf import transformations
import multilateration as mlat
import receiver_array_interfaces as rai

import os
import serial
import threading
import traceback

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from mil_passive_sonar.srv import *
from mil_ros_tools import numpy_to_point, make_header


__author__ = 'David Soto'

# GLOBALS
lock = threading.Lock()  # prevent multiple threads simultaneously using a serial port

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

class PassiveSonar(object):
    '''
    Passive Sonar Driver: listens for a pinger and uses multilateration to locate it in a
        specified TF frame.

    Args:
    * input_mode - one of the following strings: ['serial', 'log', 'signal_cb']
        'serial'     - get signal input with a data acquistion board via a serial port
        'paul_board' - get signal input with a paul board via a Ping message
        'log'        - get signal input from a .npz file saved to disk
        'signal_cb'  - get signal input from user provided function
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
    def __init__(self, input_mode='serial', custom_input_source=None):

        self.input_mode = input_mode
        self.load_params()
        self.logging_enabled = False

        # TODO: update to use ros_alarms

        if self.input_mode == 'log':
            self.input_source = rai._Logged(self.input_src_params['log'])

        elif self.input_mode == 'serial':
            self.input_source = rai._Serial(self.input_src_params['serial'], self.receiver_count)

        elif self.input_mode == 'paul_board':
            self.input_source = rai._PaulBoard(self.input_src_params['paul_board'])

        elif self.input_mode == 'sim':
            self.input_source = rai._Simulated(self.input_src_params['sim'])

        elif self.input_mode == 'custom_cb':
            if not issubclass(custom_input_source, ReceiverArrayInterface):
                raise RuntimeError('The custom input source provided ({}) is not derived from {}'
                    .format(type(custom_input_source), 'ReceiverArrayInterface'))
            self.input_source = custom_input_source

        else:
            raise RuntimeError('\'' + self.input_mode + '\' is not a supported input mode')

        self.reset_position_estimate(None)

        self.multilaterator = mlat.Multilaterator(self.receiver_locations, self.c, self.method)
        rospy.loginfo('Multilaterator initialized with wave speed (c) of {} m/s.'.format(self.multilaterator.c))

        self.plot_pub = rospy.Publisher('/passive_sonar/plot', Image, queue_size=1)
        self.rviz_pub = rospy.Publisher("/passive_sonar/rviz", Marker, queue_size=10)
        self.declare_services()
        rospy.loginfo('Passive sonar driver initialized with "{}" input mode.'.format(self.input_mode))

    # Passive Sonar Helpers

    def load_params(self):
        '''
        Loads all the parameters needed for receiving and processing signals from the passive
        sonar board and calculating headings towards an active pinger.

        These parameters are described in detail in the Passive Sonar page of the mil_common wiki.
        TODO: copy url here
        '''
        # ROS params expected to be loaded under namespace passive_sonar
        self.required_params = ['receiver_locations', 'method', 'c',
                                'pinger_freq', 'sampling_freq',
                                'upsampling_factor', 'locating_frame',
                                'receiver_array_frame', 'min_variance',
                                'observation_buffer_size', 'input_timeout']
        self.input_src_params = {
            'serial': ['port', 'baud', 'tx_request_code', 'tx_start_code',
                       'read_timeout', 'scalar_size', 'receiver_array_frame',
                       'signal_bias', 'locating_frame', 'signal_size'],
            'paul_board' : ['receiver_array_frame', 'locating_frame',
                            'sampling_freq'],
            'log': ['log_filename', 'locating_frame', 'receiver_array_frame'],
            'sim': ['locating_frame', 'receiver_array_frame', 'pinger_frame',
                    'receiver_locations', 'c', 'pinger_freq', 'sampling_freq',
                    'noise_sigma']
        }

        def load(prop):
            setattr(self, prop, rospy.get_param('passive_sonar/' + prop))
        try:
            [load(x) for x in self.required_params]
        except KeyError as e:
            raise IOError('A required rosparam was not declared: ' + str(e))

        self.receiver_count = len(self.receiver_locations) + 1  # Add one for the reference receiver
        self.receiver_locations = np.array(self.receiver_locations)

    def declare_services(self):
        '''
        Conveniently declares all the services offered by the driver
        '''
        service_types = {
            'get_pulse_heading': GetPulseHeading,
            'estimate_pinger_position_2d': EstimatePingerPosition2D,
            'estimate_pinger_position_3d': EstimatePingerPosition3D,
            'reset_position_estimate': ResetPositionEstimate,
            'start_logging': StartLogging,
            'save_log': SaveLog,
            'set_frequency': SetFrequency
        }
        self.services = dict(zip(
            service_types.keys(),
            [rospy.Service('passive_sonar/' + s[0], s[1], getattr(self, s[0])) for s in service_types.items()])
        )

    def log_data(self, signals, trans, rot):
        '''
        Logs the signal and tf data to a file that can then be playedback and used as input by
        running the driver with input_mode='log'
        '''
        if self.logging_enabled:
            # If nothing has been logged yet, initialize log shape
            if signals.size == None:
                self.signals_log = self.signals_log.reshape(0, signals.shape[0], signals.shape[1])

            self.signals_log = np.stack((self.signals_log, [signals]))
            self.trans_log = np.stack((self.trans_log, [trans]))
            self.rot_log = np.stack((self.rot_log, [rot]))

    def get_dtoa(self, signals):
        '''
        Returns a list of difference in time of arrival measurements for a signal between
        each of the non_reference hydrophones and the single reference hydrophone.

        signals - (self.receiver_count x self.signal_size) numpy array. It is assumed that the
            first row of this array is the reference signal.

        returns: list of <self.receiver_count> dtoa measurements in units of microseconds.
        '''
        signals_upsamp = np.array([x.upsample_linear(self.upsampling_factor) for x in signals])

        dtoa, cross_corr = \
            map(np.array, zip(*[mlat.get_time_delta(signals_upsamp[0], non_ref) for non_ref \
                                in signals_upsamp[1:]]))

        self.visualize_dsp(signals_upsamp, cross_corr, dtoa)

        return dtoa

    #Passive Sonar Services

    @thread_lock(lock)
    def get_pulse_heading(self, srv):
        '''
        Returns the heading towards an active pinger emmiting at <self.pinger_freq>.
        Heading will be a unit vector in hydrophone_array frame
        '''
        success, err_str = True, ''
        signals, p0, R = None, None, None
        try:
            time = rospy.Time.now()

            # Poll input source until it reports it is ready
            self.input_source.input_request()
            while not self.input_source.ready and not rospy.is_shutdown():
                if rospy.Time.now() - time > rospy.Duration(self.input_timeout):
                    raise IOError('Timed out waiting for input source')
                else:
                    rospy.sleep(0.05)
            self.input_source.reset()

            # Gather input from source
            signals, p0, R = self.input_source.get_input()

            # Carry out multilateration to get heading to pinger
            dtoa, cross_corrs = mlat.get_dtoas(ref_signal=signals[0], non_ref_signals=signals[1:])
            self.visualize_dsp(signals, cross_corrs, dtoa)
            heading = self.multilaterator.get_pulse_location(dtoa, method=srv.method)
            heading = heading / np.linalg.norm(heading)
            rospy.loginfo('Receiver position: {}\nHeading to pinger: {}\nDTOA: {}'.format(p0, heading, dtoa))

        except Exception as e:
            rospy.logerr(traceback.format_exc())
            heading = np.full(3, np.NaN)
            dtoa = []
            success = False
            err_str = traceback.format_exc()


        res = GetPulseHeadingResponse(
            header=make_header(stamp=time, frame=self.locating_frame),
            x=heading[0], y=heading[1], z=heading[2],
            dtoa=dtoa,
            success=success,
            err_str=err_str)

        # Log input if self.logging_enabled == True
        self.log_data(signals, p0, R)

        # Add heaing observation to buffers if the signals are above the variance threshold
        try:
            variance = np.var(np.array([s.samples for s in signals]).flatten())
            # if variance > self.min_variance:  # Volume threshold, would be good to also have freq check
            if np.all(np.isfinite(variance)):  # checks for not being NAN or inf/ -inf
                angles = transformations.euler_from_matrix(R)
                delta = R.dot(heading)
                p1 = p0 + delta

                self.visualize_heading(p0, p1, bgra=[1.0, 1.0, 1.0, 0.50], length=4.0)

                self.heading_start = np.append(self.heading_start, np.array([p0]), axis=0)
                self.heading_end = np.append(self.heading_end, np.array([p1]), axis=0)
                self.observation_variances = np.append(self.observation_variances, variance)

                # delete softest samples if we have over max_observations
                if len(self.heading_start) >= self.observation_buffer_size:
                    rospy.logwarn('Observation buffer full, deleting old headings')
                    softest_idx = np.argmin(self.observation_variances)
                    self.heading_start = np.delete(self.heading_start, softest_idx, axis=0)
                    self.heading_end = np.delete(self.heading_end, softest_idx, axis=0)
                    self.observation_variances = np.delete(self.observation_variances, softest_idx, axis=0)

                rospy.loginfo('Added heading {} to the observation buffer (size={})'
                              .format(heading, len(self.heading_start)))
        except Exception as e:
            rospy.logwarn(traceback.format_exc())
            rospy.logwarn(str(e))  # Service should still return

        # print "{}\n{}".format(type(res), res)
        return res

    def estimate_pinger_position_2d(self, req):
        if len(self.heading_start) < 1:
            raise RuntimeError(
                'Not enough heading observations to estimate the pinger position')
        p = mlat.ls_line_intersection2d(self.heading_start[:, :2], self.heading_end[:, :2])
        self.pinger_position_2d = np.array([p[0], p[1], 0])
        self.visualize_pinger_pos_estimate(dim=2, bgra=[255, 0.0, 255, 0.75])
        return EstimatePingerPosition2DResponse(
            header=make_header(stamp=rospy.Time.now(), frame=self.locating_frame),
            num_headings=len(self.heading_start),
            x=p[0], y=p[1])

    def estimate_pinger_position_3d(self, req):
        '''
        Uses a buffer of prior observations to estimate the position of the pinger as the intersection
        of a set of 3d lines in the least-squares sense.
        '''
        if len(self.heading_start) < 1:
            raise RuntimeError(
                'Not enough heading observations to estimate the pinger position')
        p = mlat.ls_line_intersection3d(self.heading_start, self.heading_end)
        self.pinger_position_3d = np.array([p[0], p[1], p[2]])
        self.visualize_pinger_pos_estimate(dim=3, bgra=[0.0, 255, 255, 0.75])
        return EstimatePingerPosition3DResponse(
            header=make_header(stamp=rospy.Time.now(), frame=self.locating_frame),
            num_headings=len(self.heading_start),
            x=p[0], y=p[1], z=p[2])

    def reset_position_estimate(self, req):
        '''
        Clears all the heading and amplitude buffers and makes the position estimate NaN
        '''
        self.heading_start = np.empty((0, 3), float)
        self.heading_end = np.empty((0, 3), float)
        self.observation_variances = np.empty((0, 0), float)
        self.pinger_position_2d = np.array([np.NaN, np.NaN, np.NaN])
        self.pinger_position_3d = np.array([np.NaN, np.NaN, np.NaN])
        return {}

    def start_logging(self, req):
        '''
        Enables logging and checks that we have write access to the desired save path.
        Resets the logged data buffers.
        '''
        self.log_filename = req.filename if req.filename.endswith('.npz') else req.filename + '.npz'

        if not os.access(self.log_filename, os.W_OK):
            raise IOError("Unable to write to file: " + self.log_filename)

        self.logging_enabled = True
        self.signals_log = None
        self.trans_log = np.array([]).reshape(0, 3)
        self.rot_log = np.array([]).reshape(0, 3, 3)

    def save_log(self, req):
        '''
        Saves the buffers holding signals, and receiver array poses to a compressed .npz
        file. These files can be used as source of input by running the passive sonar
        driver with input_mode='log' and providing filename in a rosparam.
        '''
        try:
            np.savez_compressed(file=self.log_filename, signal=self.signals_log,
                                trans=self.trans_log, rot=self.rot_log)

        except IOError as e: # Couln't save to specified path
            rospy.logerr(str(e))

        finally:
            self.logging_enabled = False

    def set_frequency(self, req):
        '''
        Sets the assumed frequency (in absence of noise) of the signals to received by the driver
        '''
        self.pinger_freq = req.frequency
        self.heading_start = np.empty((0, 3), float)
        self.heading_end = np.empty((0, 3), float)
        self.sample_variances = np.empty((0, 1), float)
        self.pinger_position_2d = np.array([np.NaN, np.NaN, np.NaN])
        self.pinger_position_3d = np.array([np.NaN, np.NaN, np.NaN])
        return {}

    # Visualization

    def visualize_dsp(self, signals, cross_corr, dtoa):
        '''
        Plots the received signals and cross correlations and publishes the image to /passive_sonar/plot
        '''
        import matplotlib
        matplotlib.use('agg')
        import matplotlib.pyplot as plt
        plt.plasma()

        fig, axes = plt.subplots(nrows=2, ncols=1, sharex=False, sharey=False)
        axes[0].set_title("Recorded Signals (Black is reference)")
        axes[0].set_xlabel('Time (seconds)')
        axes[1].set_title("Cross-Correlations")
        axes[1].set_xlabel('Lag (seconds)')

        fig.set_size_inches(9.9, 5.4) # Experimentally determined
        fig.set_dpi(400)
        fig.tight_layout(pad=2)

        signals[0].plot(axes[0].plot, c='k', lw=0.75)
        signals[1].plot(axes[0].plot, c='r', lw=0.75)
        signals[2].plot(axes[0].plot, c='g', lw=0.75)
        signals[3].plot(axes[0].plot, c='b', lw=0.75)
        mlat.plot_signals(signals[1:], axes[0].plot, lw=0.75)
        mlat.plot_signals(cross_corr, axes[1].plot, lw=0.75)

        fig.canvas.draw() # render plot
        plot_img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        plot_img = plot_img.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        try:
            self.plot_pub.publish(CvBridge().cv2_to_imgmsg(plot_img, 'bgr8'))
        except CvBridgeError as e:
            rospy.logerr(e)  # Intentionally absorb CvBridge Errors

    def visualize_pinger_pos_estimate(self, dim, bgra=[255, 255, 255, 1.0]):
        '''
        Publishes a marker to RVIZ representing the last calculated estimate of the position of
        the pinger.

        rgba - list of 3 or 4 floats in the interval [0.0, 1.0] representing the desired color and
            transparency of the marker
        '''
        marker = Marker()
        marker.ns = "passive_sonar-{}".format(self.pinger_freq)
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = self.locating_frame
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        bgra = np.clip(bgra, 0.0, 1.0)
        marker.color.b = bgra[0]
        marker.color.g = bgra[1]
        marker.color.r = bgra[2]
        marker.color.a = 1.0 if len(bgra) < 4 else bgra[3]
        if dim == 2:
            marker.pose.position = numpy_to_point(self.pinger_position_2d)
        elif dim == 3:
            marker.pose.position = numpy_to_point(self.pinger_position_3d)
        else:
            rospy.logwarn('Invalid dim parameter passed to visualize_pinger_pos_estimate')

        self.rviz_pub.publish(marker)

    def visualize_heading(self, tail, head, bgra, length=1.0):
        '''
        Publishes an arrow marker to RVIZ representing the heading towards the last heard ping.

        tail - 3x1 numpy array
        head - 3x1 numpy array
        lenth - scalar (float, int) desired length of the arrow marker. If None, length will
            be unchanged.
        '''
        head = tail + (head - tail) / np.linalg.norm(head - tail) * length
        head = Point(head[0], head[1], head[2])
        tail = Point(tail[0], tail[1], tail[2])
        marker = Marker()
        marker.ns = "passive_sonar-{}/heading".format(self.pinger_freq)
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

    mode = None
    try:
       mode = rospy.get_param('passive_sonar/input_mode')
    except KeyError as e:
       mode = 'serial'
       rospy.logerr('The param passive_sonar/input_mode was not set, defaulting to \
                    \'serial\'. (' + str(e) + ')')

    ping_ping_motherfucker = PassiveSonar(input_mode=mode)
    rospy.spin()
