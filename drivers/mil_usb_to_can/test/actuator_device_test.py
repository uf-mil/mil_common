#!/usr/bin/env python
import unittest
import rospy
from mil_usb_to_can.srv import SetActuator, GetActuator


class ActuatorDeviceTest(unittest.TestCase):
    '''
    Integration test for CAN2USB board driver. Talks
    to a simulated actuator device which should control pneumatic actuators
    '''
    def __init__(self, *args):
        self.set_srv = rospy.ServiceProxy('/usb_to_can_driver/set_actuator', SetActuator)
        self.get_srv = rospy.ServiceProxy('/usb_to_can_driver/get_actuator', GetActuator)
        super(ActuatorDeviceTest, self).__init__(*args)

    def test_1service_exists(self):
        try:
            self.set_srv.wait_for_service(5)
        except rospy.ServiceException as e:
            self.fail('Service error: {}'.format(e))

    def test_2set_service_works(self):
        address = 7
        res = self.set_srv(address, True)
        expectedBytes = ord('A') * pow(2, 8 * 3) + address * pow(2, 8 * 2) + 1 * pow(2, 8 * 1) + 1 * pow(2, 8 * 0)
        self.assertEquals(res.sent_bytes, expectedBytes)

    def test_3service_raises_exception(self):
        address = 13
        self.assertRaises(rospy.ServiceException, self.set_srv, address, True)

    def test_4get_service_works(self):
        address = 4
        self.set_srv(address, True)
        self.assertTrue(self.get_srv(address).actuator_on)
        self.set_srv(address, False)
        self.assertFalse(self.get_srv(address).actuator_on)

if __name__ == "__main__":
    rospy.init_node('actuator_device_test', anonymous=True)
    import rostest
    rostest.rosrun('mil_usb_to_can', 'actuator_device_test', ActuatorDeviceTest)
    unittest.main()
