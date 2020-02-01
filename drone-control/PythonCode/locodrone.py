################# LocoDrone Python Control ##################

# Written by: Joey Orduna
# Filename: locodrone.py

##############################################################
# Import LocoRobo Libraries For Communication with Robot
from errors import DroneBindException, \
                   SerialClosedException, \
                   DroneUnbindException, \
                   DroneReconnectException, \
                   SerialMessageException, DroneInitializationException

from control_codes import OutgoingMessageType, \
                          IncomingMessageType, InitializationFailureType, OperatingModeType, ControlSourceType
import math
import serial
import time

DEBUG = True

# Accelerometer tilt range, to be mapped to PWM range
MAX_TILT_ANGLE = 60

# Time in seconds for the fly up and land methods
FLY_UP_TIME = 12
LAND_TIME = 7

# Pre-set control values for programmable flight routines
FLY_UP_THROTTLE = 215
LAND_THROTTLE = 127
DRIVE_THROTTLE = 210

MOVE_FORWARD_VALUE = 150
MOVE_BACKWARD_VALUE = 100

TURN_RIGHT_VALUE = 150
TURN_LEFT_VALUE = 100


class LocoDrone:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.bound = False

        self.throttle = 0
        self.yaw = 127
        self.pitch = 127
        self.roll = 127

        self.yawTrim = 64
        self.pitchTrim = 64
        self.rollTrim = 64

        self.flyDrive = 0  # 0 = Fly, 15 = Drive

    def open_serial(self):
        self.serial = serial.Serial(self.port, self.baudrate, timeout=10)

        time.sleep(5)

        print("Serial Port Opened!")

        print('Checking Communication...')
        self.ping()
        print('Communication Ok')

    def bind(self):
        self.verify_serial()

        print("Binding to drone...")
        self.serial.write([OutgoingMessageType.Bind, 0])

        status = self.read_response_status(IncomingMessageType.BindSuccess,
                                           IncomingMessageType.BindFailure)

        if status:
            self.bound = True
            time.sleep(2)
            print("Bound!")
            self.set_throttle(0)
        else:
            raise DroneBindException('Drone failed to bind')

    def unbind(self):
        print("Unbinding from drone...")

        self.serial.write([OutgoingMessageType.Unbind, 0])

        status = self.read_response_status(IncomingMessageType.UnbindSuccess,
                                           IncomingMessageType.UnbindFailure)

        if status:
            self.bound = False
        else:
            raise DroneUnbindException('Drone failed to unbind')

    def reconnect(self):
        print("Re-Connecting with drone...")

        self.serial.write([OutgoingMessageType.Reconnect, 0])

        status = self.read_response_status(IncomingMessageType.ReconnectSuccess                             ,IncomingMessageType.ReconnectFailure)
        if status:
            self.bound = True
            print("Drone Reconnected!")
        else:
            raise DroneReconnectException('Drone failed to reconnect')

    def fly_mode(self):
        print("Selecting Fly Mode.")

        self.serial.write([OutgoingMessageType.OperatingMode, 1, OperatingModeType.Fly])
        self.flyDrive = 0

    def drive_mode(self):
        print("Selecting Drive Mode.")

        self.serial.write([OutgoingMessageType.OperatingMode, 1, OperatingModeType.Drive])
        self.flyDrive = 15

    def joystick_control(self):
        print("Selecting Joystick Control.")

        self.serial.write([OutgoingMessageType.ControlSource, 1, ControlSourceType.Joystick])

    def accelerometer_control(self):
        print("Selecting Accelerometer Control.")

        self.serial.write([OutgoingMessageType.ControlSource, 1, ControlSourceType.Accelerometer])

    def serial_control(self):
        print("Selecting Serial Control.")

        self.serial.write([OutgoingMessageType.ControlSource, 1, ControlSourceType.Serial])

    def get_throttle(self):
        return self.throttle

    def get_yaw(self):
        return self.yaw

    def get_roll(self):
        return self.roll

    def get_pitch(self):
        return self.pitch

    def get_yaw_trim(self):
        return self.yawTrim

    def get_pitch_trim(self):
        return self.pitchTrim

    def get_roll_trim(self):
        return self.rollTrim

    def set_throttle(self, value):
        self.throttle = max(0, min(255, value))

    def set_yaw(self, value):
        self.yaw = max(0, min(255, value))

    def set_pitch(self, value):
        self.pitch = max(0, min(255, value))

    def set_roll(self, value):
        self.roll = max(0, min(255, value))

    def set_yaw_trim(self, value):
        self.yawTrim = max(0, min(128, value))

    def set_pitch_trim(self, value):
        self.pitchTrim = max(0, min(128, value))

    def set_roll_trim(self, value):
        self.rollTrim = max(0, min(128, value))

    def reset_payload(self):
        self.set_throttle(0)
        self.set_yaw(127)
        self.set_pitch(127)
        self.set_roll(127)

    def update_payload(self):

        payload = [OutgoingMessageType.ControlPayload,
                   8,
                   self.throttle,
                   self.yaw,
                   self.yawTrim,
                   self.pitch,
                   self.roll,
                   self.pitchTrim,
                   self.rollTrim,
                   self.flyDrive]

        self.serial.write(payload)

        if DEBUG:
            print(payload)

    def ping(self):
        self.serial.write([OutgoingMessageType.Echo, 0])

        self.read_response_status(IncomingMessageType.Ping,
                                  None)

    def request(self, data_request_code):
        self.serial.write([data_request_code, 0])

    def read_response_status(self, success_code, failure_code):
        control = self.read_byte()
        length = self.read_byte()
        msg = None

        if length > 0:
            msg = self.serial.read(length)

        if control == success_code:
            return True

        if control == failure_code:
            return False

        if control == IncomingMessageType.InitializationFailure:
            self.handle_initialization_failure(msg)

        if control == IncomingMessageType.Debug:
            print('Incoming Debug Message', msg.decode('utf-8'))
            return self.read_response_status(success_code, failure_code)
        else:
            raise SerialMessageException('Unexpected serial message {}'.format(control))

    def handle_initialization_failure(self, message):
        if len(message) == 0:
            if message[0] == InitializationFailureType.AccelerometerFailure:
                raise DroneInitializationException('Accelerometer Failed to Initialize')
            if message[0] == InitializationFailureType.JoystickFailure:
                raise DroneInitializationException('Joysticks Failed to Initialize')

        raise DroneInitializationException('Unknown Failed to Initialize')

    def read_byte(self):
        b = self.serial.read()
        #print(b)
        return list(b)[0]

    def verify_serial(self):
        pass
        # if self.serial is None or not self.serial.is_open:
        #     raise SerialClosedException('Serial is not open')

    # Time-based programmable fly functions
    def fly_up(self):

        timeout = time.time() + FLY_UP_TIME
        while time.time() < timeout:
            self.set_throttle(FLY_UP_THROTTLE)
            self.set_pitch(127)
            self.set_roll(127)
            self.set_yaw(127)
            self.update_payload()
            time.sleep(0.1)

    def land(self):
        timeout = time.time() + LAND_TIME
        while time.time() < timeout:
            self.set_throttle(LAND_THROTTLE)
            self.set_pitch(127)
            self.set_roll(127)
            self.set_yaw(127)
            self.update_payload()
            time.sleep(0.1)

    def move_forward(self, seconds):
        timeout = time.time() + seconds
        while time.time() < timeout:
            self.set_throttle(DRIVE_THROTTLE)
            self.set_pitch(MOVE_FORWARD_VALUE)
            self.set_roll(127)
            self.set_yaw(127)
            self.update_payload()
            time.sleep(0.1)

    def move_backward(self, seconds):

        timeout = time.time() + seconds
        while time.time() < timeout:
            self.set_throttle(MOVE_BACKWARD_VALUE)
            self.set_pitch(127)
            self.set_roll(127)
            self.set_yaw(127)
            self.update_payload()
            time.sleep(0.1)

    def turn_left(self, seconds):

        timeout = time.time() + seconds
        while time.time() < timeout:
            self.set_throttle(DRIVE_THROTTLE)
            self.set_pitch(127)
            self.set_roll(127)
            self.set_yaw(TURN_LEFT_VALUE)
            self.update_payload()
            time.sleep(0.1)

    def turn_right(self, seconds):

        timeout = time.time() + seconds
        while time.time() < timeout:
            self.set_throttle(DRIVE_THROTTLE)
            self.set_pitch(127)
            self.set_roll(127)
            self.set_yaw(TURN_RIGHT_VALUE)
            self.update_payload()
            time.sleep(0.1)
