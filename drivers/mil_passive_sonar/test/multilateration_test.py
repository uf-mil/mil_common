#!/usr/bin/env python
import rospy
import rostest
import unittest
import sys
import numpy as np
import os


import multilateration as mlat
import passive_sonar_driver as psd

__author__ = 'David Soto'

PKG = 'mil_passive_sonar'
NAME = 'multilateration_test'

class MlatTester(unittest.TestCase):
    '''
    Tests multilateration class methods
    '''
    def __init__(self, *args):
        rospy.init_node(NAME, anonymous=True)
        super(MlatTester, self).__init__(*args)
        # To print debug statement during tests
        self.log = open(os.environ['HOME'] + '/temp_test_log.txt', 'w')

    def test_time_delta_from_delayed_signals(self):
        '''
        This test should fail if we fail to estimate the time delay between two signals
        that are identical delayed versions of each other
        '''
        # Define length of signal observations
        sig_dur = 100E-6 # s

        # First create a sinusoidal pulse
        ping_freq = 35E3 # Hz
        ping_duration = 20E-6 # s
        sampling_rate = 500E3 # samples per second
        num_ping_samples = int(round(sampling_rate * ping_duration)) + 1
        t_ping = np.linspace(0, ping_duration, num_ping_samples, endpoint=True)
        sig = np.sin(ping_freq * t_ping)
        f =  35000 # freq in Hz
        ping = mlat.TimeSignal1D(samples=sig, sampling_freq=f)

        # Create reference signal (has no delay)
        delay_gt = 0
        ref = mlat.make_delayed_signal(ping, delay_gt, sig_dur)

        # Create signal with random delay
        #delay_gt = 20E-6 # ground truth delay (s)
        delay_gt = np.random.uniform(low=-20E-6, high=20E-6) # ground truth delay (s)
        non_ref = mlat.make_delayed_signal(ping, delay_gt, sig_dur)
        mlat.plot_signals([ref, non_ref])

        # Estimate delay between signals using cross-correlation
        delay_est = mlat.get_time_delta(ref, non_ref)[0]
        
        err_msg = 'Mismatch between time delta ground truth and estimate ({} vs {})'
        err_msg = err_msg.format(delay_gt, delay_est)
        #self.log.write(err_msg)
        self.assertTrue(np.isclose(delay_gt, delay_est), err_msg)

    def test_trivial(self):
        # TODO: test line intersection from noisy headings w/ ground truth
        self.assertTrue(True)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, MlatTester, sys.argv)

