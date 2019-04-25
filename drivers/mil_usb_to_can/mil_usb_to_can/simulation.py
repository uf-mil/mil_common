#!/usr/bin/python
from mil_misc_tools.serial_tools import SimulatedSerial
from utils import ReceivePacket, CommandPacket
import struct
import rospy
from constants import ThrusterKillBoardConstants as tkb


class SimulatedCANDevice(object):
    '''
    Simulates a CAN device, with functions to be overrided
    to handle data requests and sends from motherboard
    '''
    def __init__(self, sim_board, can_id):
        '''
        Constructs the simulated device, storing the simulated CAN2USB board it is attached to
        and its CAN device id.
        Child classes must call this method when their __init__ is called (see ExampleSimulatedCANDevice below)
        '''
        self._sim_board = sim_board
        self._can_id = can_id

    def send_data(self, data):
        '''
        Send data onto the bus, delivering it to other simulated
        devices and to the driver node
        '''
        self._sim_board.send_to_bus(self._can_id, data)

    def on_data(self, data):
        '''
        Called when the motherboard or another simulated device
        sends data onto the bus.
        Intended to be overriden by child classes (see ExampleSimulatedCANDevice below)
        @param data: the data payload as a string/bytes object
        NOTE: as the CAN bus is shared, you should inspect the data
              to make sure it was intended for this device before processing it
        '''
        pass


class ExampleSimulatedEchoDevice(SimulatedCANDevice):
    '''
    Example implementation of a SimulatedCANDevice.
    On sends, stores the transmited data in a buffer.
    When data is requested, it echos this data back.
    '''
    def __init__(self, *args, **kwargs):
        # Call parent classes contructor
        super(ExampleSimulatedEchoDevice, self).__init__(*args, **kwargs)

    def on_data(self, data):
        # Echo data received back onto the bus
        self.send_data(data)


class ExampleSimulatedAdderDevice(SimulatedCANDevice):
    '''
    Example implementation of a SimulatedCANDevice.
    On sends, stores the transmited data in a buffer.
    When data is requested, it echos this data back.
    '''
    def __init__(self, *args, **kwargs):
        # Call parent classes contructor
        super(ExampleSimulatedAdderDevice, self).__init__(*args, **kwargs)

    def on_data(self, data):
        if ord(data[0]) != 37:
            return
        _, a, b = struct.unpack('Bhh', data)
        c = a + b
        res = struct.pack('Bi', 37, c)
        self.send_data(res)


class SimulatedThrusterKillBoard(SimulatedCANDevice):
    def __init__(self, *args, **kwargs):
        super(SimulatedThrusterKillBoard, self).__init__(*args, **kwargs)
        self.go = False
        self.isSoftKilled = False
        self.thrusterValues = [0] * 8
        # TODO: better way of sending these messages
        self.softTimer = rospy.Timer(rospy.Duration(0.1), self._advertise_soft_kill_status)
        self.goTimer = rospy.Timer(rospy.Duration(0.5), self._advertise_go_status)

    def on_data(self, data):
        if ord(data[0]) not in [tkb.KILL_MESSAGE, tkb.THRUSTER_MESSAGE]:
            return
        if ord(data[0]) == tkb.KILL_MESSAGE:  # TODO: check if proper length?
            message = struct.unpack('5B', data)
            if message == (tkb.KILL_MESSAGE, tkb.COMMAND, tkb.SOFT_KILL, tkb.ASSERTED, tkb.END):
                self.isSoftKilled = True
            elif message == (tkb.KILL_MESSAGE, tkb.COMMAND, tkb.SOFT_KILL, tkb.UNASSERTED, tkb.END):
                self.isSoftKilled = False
        elif ord(data[0]) == tkb.THRUSTER_MESSAGE:
            _, thruster, value = struct.unpack('2Bf', data)
            self.thrusterValues[thruster] = value
            print ('Thruster {} set to {}'.format(thruster, value))

    def _advertise_soft_kill_status(self, *args):
        if (self.isSoftKilled):
            killResponse = struct.pack("5B", tkb.KILL_MESSAGE, tkb.RESPONSE, tkb.SOFT_KILL, tkb.ASSERTED, tkb.END)
            print('Soft Kill')
            self.send_data(killResponse)
        else:
            return

    def _advertise_go_status(self, *args):
        assertedByte = tkb.ASSERTED if self.go else tkb.UNASSERTED
        message = struct.pack("3B", tkb.GO, assertedByte, tkb.END)
        self.send_data(message)


class SimulatedUSBtoCAN(SimulatedSerial):
    '''
    Simulates the USB to CAN board. Is supplied with a dictionary of simualted
    CAN devices to simulate the behavior of the whole CAN network.
    '''
    def __init__(self, devices={0: SimulatedCANDevice}, can_id=-1, *args, **kwargs):
        '''
        @param devices: dictionary {device_id: SimulatedCANDevice} mapping CAN IDs
                        to SimulatedCANDevice classes that will be used for that ID
        @param can_id: ID of the CAN2USB device
        '''
        self._my_id = can_id
        self._bus = {}
        self._devices = dict((can_id, device(self, can_id)) for can_id, device in devices.iteritems())
        super(SimulatedUSBtoCAN, self).__init__()

    def send_to_bus(self, can_id, data):
        '''
        Sends data onto the simulated bus from a simulated device
        @param can_id: ID of sender
        @param data: data paylod
        '''
        # If not from the motherboard, store this for future requests from motherboard
        if can_id != self._my_id:
            self._bus[can_id] = data
        # Send data to all simulated devices besides the sender
        for device_can_id, device in self._devices.iteritems():
            if device_can_id != can_id:
                device.on_data(data)

    def write(self, data):
        # Parse incomming data as a command packet from motherboard
        p = CommandPacket.from_bytes(data)
        # Call appropriate handle for simulated device
        if p.is_receive:
            # If data from this device is on the simulated bus, pass it to the motherboard
            if p.filter_id in self._bus:
                self.buffer += ReceivePacket.create_receive_packet(self._bus[p.filter_id]).to_bytes()
            # Otherwise, simulate a serial timeout
            else:
                # Todo: raise actual serial timeout
                raise Exception('serial timeout')
        else:
            # If a send, send this data to other devices on simulated CAN network
            self.send_to_bus(self._my_id, p.data)
        return len(data)
