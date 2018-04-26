#!/usr/bin/env python
'''
This file contains multiple receiver array interfaces for both simulated and real passive
sonar functionality
'''


from __future__ import division

import serial
import traceback
import numpy as np

import rospy
import tf2_ros
from tf import transformations
from gazebo_msgs.srv import GetModelState

import multilateration as mlat
from hydrophones.msg import Ping


__author__ = 'David Soto'


def _load_interface_params(interface, param_names):
    '''
    Load params relevant to this interface as specified in the PassiveSonar object's load_params
      method in passive_sonar_driver.py
    These parameters can be accessed as self.<param_names>

    @param interface Interface object derived from the ReceiverArrayInterface class
    @param param_names List of names of ros params to load
    '''
    def load(prop):
        setattr(interface, prop, rospy.get_param('passive_sonar/' + prop))
    try:
        [load(x) for x in param_names]
    except KeyError as e:
        raise IOError('A required rosparam was not declared: ' + str(e))


def _pose_from_tf(tf):
    ''' Extracts a translation and rotation matrix from a transformation
    @rtype (np.array, np.array)
    @return Translation vector (3) and rotation matrix (3x3)
    '''
    t = tf.transform.translation
    q = tf.transform.rotation
    return (np.array([t.x, t.y, t.z]),
            transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3])


class ReceiverArrayInterface(object):
    '''
    This class encapsulates the process of acquiring signals from the receiver array
    and acquiring the pose of the receiver array in a world fixed frame.
    '''
    ready = False  # Request would be immediately responded to this wasn't initialized to False

    def input_request(self):
        '''
        The driver will call this function to request signal and tf data.
        Inheriting classes MUST OVERRIDE this method and do the following:
        1) Store signals in self.signals
        2) Store receiver translation in self.translation (3x1 numpy array)
        3) Store receiver rotation in self.rotation (3x3 numpy array)
        4) Set self.ready to true once the above things are done
        '''
        msg = 'Derived class of {} (mil_passive_sonar pkg) failed to override the input_request method'
        rospy.logerr(msg.format(type(self)))

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


class _Serial(ReceiverArrayInterface):
    '''
    This is the default serial ReceiverArrayInterface for the passive sonar driver.
    It is used when the keyword arg input_mode='serial' is passed to the passive sonar
    driver constructor.
    '''
    def __init__(self, param_names, num_receivers):
        self.num_receivers = num_receivers

        _load_interface_params(self, param_names)

        self.tf2_buf = tf2_ros.Buffer()
        self.tf2_list = tf2_ros.TransformListener(self.tf2_buf)

        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=self.read_timeout)
            self.ser.flushInput()
        except serial.SerialException as e:
            rospy.err("Sonar serial connection error: " + str(e))
            raise e

    def input_request(self):
        try:
            self._request_signals()
            self._receive_signals()
            self.translation, self.rotation = _pose_from_tf(
                self.tf2_buf.lookup_transform(
                    target_frame=self.receiver_array_frame,
                    source_frame=self.locating_frame,
                    time=rospy.Time.now(),
                    timeout=rospy.Duration(0.20))
            )
            self.ready = True
        # Intentionally absorb errors (driver will just timeout)
        except Exception:
            rospy.logerr(traceback.format_exc())

    def _request_signals(self):
        '''
        Request a set of digital signals from a serial port

        serial_port - open instance of serial.Serial
        data_request_code - char or string to sent to receiver board to signal a data request
        tx_start_code - char or string that the receiver board will send  us to signal the
            start of a transmission
        '''
        self.ser.flushInput()
        readin = None

        # Request raw signal tx until start bit is received
        while readin is None or ord(readin) != ord(self.tx_start_code):
            self.ser.write(self.tx_request_code)
            readin = self.ser.read(1)
            if len(readin) < len(self.tx_request_code):  # serial read timed out
                raise IOError('Timed out waiting for serial response.')

    def _receive_signals(self):
        '''
        Receives a set of 1D signals from a serial port and packs them into a numpy array

        serial_port - port of type serial.Serial from which to read
        signal_size - number of sacalar elements in each 1D signal
        scalar_size - size of each scalar in bytes
        signal_bias - value to be subtracted from each read scalar before packing into array

        returns: 2D numpy array with <num_signals> rows and <signal_size> columns
        '''
        # this doesn't work well, need to fix
        def error_correction(num_str):
            return num_str  # temp
            output = ''
            for char in num_str:
                valid = ord(char) > ord('0') and ord(char) < ord('9')
                output += char if valid else '5'
            return output

        self.signals = np.full((self.num_receivers, self.signal_size), self.signal_bias, dtype=float)

        for channel in range(self.num_receivers):
            for i in range(self.signal_size):
                while self.ser.inWaiting() < self.scalar_size:
                    rospy.sleep(0.001)

                self.signals[channel, i] = float(
                    error_correction(self.ser.read(self.scalar_size))) - self.signal_bias


class _PaulBoard(ReceiverArrayInterface):
    '''
    This is the Paul Board serial ReceiverArrayInterface for the passive sonar
    driver. It is used when the keyword arg input_mode='paul_board' is passed
    to the passive sonar driver constructor.
    '''
    def __init__(self, param_names):
        _load_interface_params(self, param_names)

        self.tf2_buf = tf2_ros.Buffer()
        self.tf2_list = tf2_ros.TransformListener(self.tf2_buf)

        self.latest_ping = Ping()
        self.cached_ping = Ping()

        rospy.Subscriber("/hydrophones/ping", Ping, self._cache_ping)

    def _cache_ping(self, msg):
        '''
        Store the latest ping inside this reciever array interface object
        whenever a new one is received.
        '''
        self.latest_ping = msg

    def input_request(self):
        '''
        Extracts the required information from the latest cached Ping message
        and converts it into the required format.
        '''
        self.cached_ping = self.latest_ping
        self._parse_samples()
        self.translation, self.rotation = _pose_from_tf(
            self.tf2_buf.lookup_transform(
                    target_frame=self.receiver_array_frame,
                    source_frame=self.locating_frame,
                    time=self.cached_ping.header.stamp,
                    timeout=rospy.Duration(0.20))
            )
        self.ready = True

    def _parse_samples(self):
        '''
        Parse the samples in the Ping message into a set of four TimeSignal1D
        objects ordered based on the arrangement of the hydrophones.
        '''
        self.signals = []

        # Reverse the list of samples so that they can be popped off in order
        reversed_data = list(self.cached_ping.data)[::-1]

        num_channels = self.cached_ping.channels
        num_samples_per_channel = int(self.cached_ping.samples / num_channels)

        # Create a TimeSignal1D object for each channel
        for channel_index in xrange(3):
            samples = []

            # Fill the TimeSignal1D object with the amount of samples in a channel
            for sample_index in xrange(num_samples_per_channel):
                samples.append(reversed_data.pop())
            time_signal_1d = mlat.TimeSignal1D(samples=np.array(samples, dtype=float), sampling_freq=self.sampling_freq)
            print(time_signal_1d.samples[:10])
            self.signals.append(time_signal_1d)


class _Logged(ReceiverArrayInterface):
    '''
    This is the default logged data ReceiverArrayInterface for the passive sonar driver.
    It is used when the keyword arg input_mode='log' is passed to passive sonar driver
    constructor.
    '''

    def __init__(self, param_names):
        _load_interface_params(self, param_names)
        self.iter_num = 0
        try:
            self.np_log = np.load(self.log_filename)
            rospy.loginfo('Loaded log file: ' + self.log_filename)
        except IOError as e:
            rospy.logerr('PASSIVE SONAR: Unable to access log file.')
            raise e

    def input_request(self):
        try:
            self.signals = self.np_log['signal'][self.iter_num]
            self.translation = self.np_log['trans'][self.iter_num]
            self.rotation = self.np_log['rot'][self.iter_num]
            self.ready = True
            self.iter_num += 1
        except ImportError as e:
            rospy.logerr(traceback.format_exc())
            raise e
        except IndexError as e:
            self.ready = False
            rospy.logwarn('The end of the log was reached.')


class _Simulated(ReceiverArrayInterface):
    '''
    This class will create delayed noisy signals simulating the listening to a sinusoidal
    pulse.
    '''
    def __init__(self, param_names):
        _load_interface_params(self, param_names)
        self.tf2_buf = tf2_ros.Buffer()
        self.tf2_list = tf2_ros.TransformListener(self.tf2_buf)

    def input_request(self):
        ''' Respond to a request for input (TF + signals) from the ROS driver '''
        try:
            ref_signal = self.generate_ping(
                ping_freq=self.pinger_freq / 1E6,
                ping_duration=0.004 * 1E6,
                sampling_freq=self.sampling_freq / 1E6
            )

            pinger_in_receiver_frame = _pose_from_tf(
                self.tf2_buf.lookup_transform(
                    source_frame=self.pinger_frame,
                    target_frame=self.receiver_array_frame,
                    time=rospy.Time.now(),
                    timeout=rospy.Duration(0.20))
            )
            pinger_position_mm = pinger_in_receiver_frame[0] * 1E3  # convert to millimeters

            dtoa_us = mlat.generate_dtoa(c=self.c,
                                         source_position=pinger_position_mm,
                                         non_ref_positions=self.receiver_locations)
            noisy_dtoa_us = dtoa_us + np.random.normal(scale=self.noise_sigma, size=3)
            self.signals = mlat.make_delayed_signals_from_DTOA(
                reference_signal=ref_signal,
                dtoa=noisy_dtoa_us
            )

            self.translation, self.rotation = _pose_from_tf(
                self.tf2_buf.lookup_transform(
                    source_frame=self.receiver_array_frame,
                    target_frame=self.locating_frame,
                    time=rospy.Time.now(),
                    timeout=rospy.Duration(0.20))
            )
            self.ready = True
        # Intentionally absorb errors (driver will just timeout)
        except Exception:
            rospy.logerr(traceback.format_exc())

    def generate_ping(self, ping_freq, ping_duration, sampling_freq):
        def f(t):
            return (np.sin(ping_freq * t) * 1.0 /
                    (1.0 + np.exp(t * -ping_freq / 2.0)))
        return mlat.TimeSignal1D.from_function(funct=f,
                                               start_t=-ping_duration / 2.0,
                                               end_t=ping_duration / 2.0,
                                               sampling_freq=sampling_freq)
