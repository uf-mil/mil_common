#!/usr/bin/env python
import unittest
import rospy
from mil_usb_to_can.srv import SetActuator


class ActuatorDeviceTest(unittest.TestCase):
    '''
    Integration test for CAN2USB board driver. Talks
    to a simulated actuator device which should control pneumatic actuators
    '''
    def __init__(self, *args):
        self.srv = rospy.ServiceProxy('/usb_to_can_driver/set_actuator', SetActuator)  # TODO remove tilde?
        super(ActuatorDeviceTest, self).__init__(*args)

    def test_1service_exists(self):
        try:
            self.srv.wait_for_service(5)
        except rospy.ServiceException as e:
            self.fail('Service error: {}'.format(e))

    def test_2service_works(self):
        address = 7
        res = self.srv(address, True)
        self.assertEquals(res.sent_byte, address + 0x80 + 0x40)

    def test_3service_raises_exception(self):
        address = 65
        self.assertRaises(rospy.ServiceException, self.srv, address, True)

if __name__ == "__main__":
    rospy.init_node('actuator_device_test', anonymous=True)
    import rostest
    rostest.rosrun('mil_usb_to_can', 'actuator_device_test', ActuatorDeviceTest)
    unittest.main()
