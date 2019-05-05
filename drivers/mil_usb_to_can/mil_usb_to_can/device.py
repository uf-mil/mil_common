#!/usr/bin/python
import rospy
import struct
import random
import string
import threading
from application_packet import ApplicationPacket
from rospy_tutorials.srv import AddTwoInts
from mil_usb_to_can.srv import SetActuator, GetActuator


class CANDeviceHandle(object):
    '''
    Helper class to allow developers to write handles for communication with a particular CAN device.
    See ExampleCANDeviceHandle for an example of implementing one of these.
    '''
    def __init__(self, driver, device_id):
        '''
        Creates a CANDeviceHandle.
        @param driver: a USBtoCANBoard object that will be used for communication with the USB to CAN board
        @param device_id: the CAN id of the device this class will handle
        '''
        self._driver = driver
        self._device_id = device_id

    def on_data(self, data):
        '''
        Called when data is received from the device this handle is registered for
        '''
        pass

    def send_data(self, data, can_id=0):
        '''
        Sends data to the device
        @param data: the data payload to send to the device (string/bytes object)
        '''
        return self._driver.send_data(data, can_id=can_id)


class ActuatorAddressOutOfRangeException(Exception):
    '''
    Exception thrown when the set_actuator service is called with an invalid address
    '''
    def __init__(self, address):
        super(ActuatorAddressOutOfRangeException, self).__init__(
            "actuator_address must be any integer from 0 to 11. The given address was {}".format(address))


class ActuatorDeviceHandle(CANDeviceHandle):
    '''
    An implementation of a CANDeviceHandle which will handle
    a device that controls pneumatic actuators.
    '''

    IDENTIFIER_BYTE = 'A'
    NUM_ADDRESSES = 0x0B

    def __init__(self, *args, **kwargs):
        super(ActuatorDeviceHandle, self).__init__(*args, **kwargs)
        self._set_srv = rospy.Service('~set_actuator', SetActuator, self.on_set_req)
        self._get_srv = rospy.Service('~get_actuator', GetActuator, self.on_get_req)
        self._actuator_on = [False for i in range(self.NUM_ADDRESSES)]
        self._data_lock = [threading.Condition() for i in range(self.NUM_ADDRESSES)]

    def on_set_req(self, req):
        if (
            not isinstance(req.actuator_address, int) or
            req.actuator_address < 0 or
            req.actuator_address > self.NUM_ADDRESSES
        ):
            raise ActuatorAddressOutOfRangeException(req.actuator_address)
        message = self.IDENTIFIER_BYTE + chr(req.actuator_address) + chr(1)
        if req.actuator_on:
            message += chr(1)
        else:
            message += chr(0)
        self.send_data(message)
        return reduce((lambda a, b: a * pow(2, 8) + b), [ord(char) for char in message])

    def on_get_req(self, req):
        if not isinstance(req.actuator_address, int) or req.actuator_address < 0 or req.actuator_address > 0x0B:
            raise ActuatorAddressOutOfRangeException(req.actuator_address)
        self._data_lock[req.actuator_address].acquire()
        message = self.IDENTIFIER_BYTE + chr(req.actuator_address) + chr(0) + chr(0)
        self.send_data(message)
        self._data_lock[req.actuator_address].wait()
        response = self._actuator_on[req.actuator_address]
        self._data_lock[req.actuator_address].release()
        return response

    def on_data(self, data):
        data_bytes = [ord(char) for char in data]
        if data_bytes[0] != 0x41 or len(data_bytes) != 4:
            raise Exception("data recieved by actuator handler does not match protocol")
        address = data_bytes[1]
        if not isinstance(address, int) or address < 0 or address > 0x0B:
            raise ActuatorAddressOutOfRangeException(address)
        self._data_lock[address].acquire()
        if data_bytes[2] == 1:
            state = True
        else:
            state = False
        self._actuator_on[address] = state
        self._data_lock[address].notifyAll()
        self._data_lock[address].release()


class ExampleEchoDeviceHandle(CANDeviceHandle):
    '''
    An example implementation of a CANDeviceHandle which will handle
    a device that echos back any data sent to it.
    '''
    def __init__(self, *args, **kwargs):
        super(ExampleEchoDeviceHandle, self).__init__(*args, **kwargs)
        self.last_sent = None
        self.send_new_string()

    def on_data(self, data):
        if self.last_sent is None:
            print 'Received {} but have not yet sent anthing'.format(data)
        elif data != self.last_sent[0]:
            print 'ERROR! Reveived {} but last sent {}'.format(data, self.last_sent)
        else:
            print 'SUCCESSFULLY echoed {} in {}seconds'.format(
                self.last_sent[0],
                (rospy.Time.now() - self.last_sent[1]).to_sec())
            rospy.sleep(0.15)
            self.send_new_string()

    def send_new_string(self):
        # Example string to test with
        test = ''.join([random.choice(string.ascii_letters) for i in range(4)])
        self.last_sent = (test, rospy.Time.now())
        print 'SENDING {}'.format(test)
        self.send_data(test)


class ExampleAdderDeviceHandle(CANDeviceHandle):
    '''
    An example implementation of a CANDeviceHandle which will handle
    a device that echos back any data sent to it.
    '''
    def __init__(self, *args, **kwargs):
        super(ExampleAdderDeviceHandle, self).__init__(*args, **kwargs)
        self.correct_response = 37
        self.response_received = None
        self._srv = rospy.Service('add_two_ints', AddTwoInts, self.on_service_req)

    def on_service_req(self, req):
        payload = struct.pack('hh', req.a, req.b)
        self.correct_response = req.a + req.b
        self.response_received = None
        self.send_data(ApplicationPacket(37, payload).to_bytes())
        start = rospy.Time.now()
        while self.response_received is None:
            if rospy.Time.now() - start > rospy.Duration(1):
                return -1
        res = ApplicationPacket.from_bytes(self.response_received, expected_identifier=37)
        my_sum = struct.unpack('i', res.payload)
        return my_sum

    def on_data(self, data):
        self.response_received = data
