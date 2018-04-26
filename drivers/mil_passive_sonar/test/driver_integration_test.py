#!/usr/bin/env python
import rospy
import rostest
import unittest
import random
import sys

import multilateration as mlat
import passive_sonar_driver as psd

__author__ = 'David Soto'

PKG = 'mil_passive_sonar'
NAME = 'driver_integration_test'

class IntegrationTester(unittest.TestCase):
    '''
    Tests passive_sonar_driver ros interface
    '''
    def __init__(self, *args):
        rospy.init_node(NAME, anonymous=True)
        super(IntegrationTester, self).__init__(*args)

    def test_passed(self):
        self.assertTrue(True)

    def test_failed(self):
        self.assertTrue(False)

if __name__ == '__main__':
  rostest.rosrun(PKG, NAME, IntegrationTester, sys.argv)

