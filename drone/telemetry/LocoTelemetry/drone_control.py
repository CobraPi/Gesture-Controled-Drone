################# LocoDrone Python Control ##################

# Written by: Joey Orduna
# Filename: drone_control.py

##############################################################
# Import LocoRobo Libraries For Communication with Robot
from locorobo import LocoRobo
from locorobo import Data
from locorobo import MotorDirection
from locorobo import WaitType
from locorobo import Song
from locorobo import Note
from errors import DroneBindException, SerialClosedException, DroneUnbindException, DroneReconnectException
import math
import serial
import time
import threading

MAX_TILT_ANGLE = 60


def map_range(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)



class Drone:
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

        self.flyRun = 0  # 0 = Fly, 15 = Run

    def get_throttle(self):
        return int(self.throttle)

    def get_yaw(self):
        return int(self.yaw)

    def get_pitch(self):
        return int(self.pitch)

    def get_roll(self):
        return int(self.roll)

    def get_yaw_trim(self):
        return int(self.yawTrim)

    def get_pitch_trim(self):
        return int(self.pitchTrim)

    def get_roll_trim(self):
        return int(self.flyRun)

    def get_fly_run(self):
        return int(self.flyRun)

    def open_serial(self):
        self.serial = serial.Serial(self.port, self.baudrate, timeout=8)
        a = b'b'
        while a != b'a':
            a = self.serial.read()
            print(a)
        self.serial.write(b'a')
        print("Serial Port Opened")

    def bind(self):
        print("Binding to drone...")
        if self.serial is None:
            raise SerialClosedException('Serial is not open')
        msg = b'0'
        while msg != b'1':
            self.serial.write(b'1')
            msg = self.serial.read()

        if msg == b'1':
            self.bound = True
            print("Binding Successful!")
        else:
            raise DroneBindException('Drone failed to bind')

    def unbind(self):
        print("Unbinding from drone...")
        self.serial.write(b'2')
        msg = b'0'
        while msg != b'1':
            msg = self.serial.read()
        if msg == b'1':
            self.bound = False
            print("Unbinding Successful!")
        else:
            raise DroneUnbindException('Failed to Un-Bind with drone')

    def reconnect(self):
        print("Re-Connecting with drone...")
        self.serial.write(b'3')
        msg = b'0'
        while msg != b'1':
            msg = self.serial.read()
        if msg == b'1':
            self.bound = True
            print("Re-Connecting Successful!")
        else:
            raise DroneReconnectException('Failed to re-connect with drone')

    def fly_mode(self):
        print("Selecting Fly Mode...")
        self.serial.write(b'F')
        msg = b'0'
        while msg != b'1':
            msg = self.serial.read()
        if msg == b'1':
            self.flyRun = 0
            print("Fly Mode Selected!!")

    def run_mode(self):
        print("Selecting Run Mode...")
        self.serial.write(b'R')
        msg = b'0'
        while msg != b'1':
            msg = self.serial.read()
        if msg == b'1':
            self.flyRun = 15
            print("Run Mode Selected!")

    def set_throttle(self, value):
        if value > 255:
            self.throttle = 255
        elif value < 0:
            self.throttle = 0
        else:
            self.throttle = value

    def set_yaw(self, value):

        if value > 255:
            self.yaw = 255
        elif value < 0:
            self.yaw = 0
        else:
            self.yaw = value

    def set_pitch(self, value):
        if value > 255:
            self.pitch = 255
        elif value < 0:
            self.pitch = 0
        else:
            self.pitch = value
        print("Set_Pitch: ", self.pitch)

    def set_roll(self, value):
        if value > 255:
            self.roll = 255
        elif value < 0:
            self.roll = 0
        else:
            self.roll = value

    def update_payload(self):
        self.serial.write(b'4')

        payload = bytearray(9)
        payload[0] = b'8'
        payload[1] = self.get_throttle()
        payload[2] = self.get_yaw()
        payload[3] = self.get_yaw_trim()
        payload[4] = self.get_pitch()
        payload[5] = self.get_roll()
        payload[6] = self.get_pitch_trim()
        payload[7] = self.get_roll_trim()
        payload[8] = self.get_fly_run()

        for i in range(9):
            print(payload[i])
        print(bytes(payload))

        self.serial.write(bytes(payload))

        print(self.serial.readline())

################## Drone Control Functions #####################


# if cmd == 'b':
#     try:
#         drone.bind()
#     except DroneBindException:
#         print('Failed to bind try again')
#     except SerialClosedException:
#         drone.open_serial()


def get_robot(robots, name):
    robot = None

    # Search through robots found during the scan for
    # the one we want
    for r in robots.values():
        if r.name == name:
            robot = r

            # We found the robot, so stop the for loop
            break
    # If we did not find the robot during the scan, stop the program
    if not robot:
        raise Exception('Could not find robot with specified name')
    return robot



"""

def _throttle_control(self):
    self.throttle_ctrl_loop = True
    while self.throttle_ctrl_loop:
        acc =  self.robot.get_sensor_value(Data.ACCELEROMETER)
        x = acc['x']
        y = acc['y']
        z = acc['z']
        z_angle = math.atan(z / math.sqrt(y**2 + x**2))
        z_angle = max(-self.MAX_ANGLE, min(z_angle * 180 / math.pi, self.MAX_ANGLE))
        if   z_angle > 30  :    self.ser.write(bytes('7', 'utf-8')) # Throttle Inc
        elif z_angle < -30 :    self.ser.write(bytes('6', 'utf-8')) # Throttle Dec
        self.ser.flushInput()
        LocoRobo.wait(0.01)
"""

################# Main Function ###############################

def main():

    # Create the drone serial object
    drone = Drone('/dev/tty.usbmodem1A1241', 19200)

    # Open the serial port
    drone.open_serial()

    # Bind to the drone
    drone.bind()
    # Tell LocoRobo what serial port to use
    LocoRobo.setup("/dev/tty.usbmodem1")

    # Scan for robots for 2000 ms
    robots = LocoRobo.scan(2000)

    # Use get_robots to find robot with name "lr 67:c6" in the scan result
    robot = get_robot(robots, "robot4")

    robot.connect()
    robot.activate_motors()
    robot.enable_sensor(Data.Accelerometer, True)
    robot.enable_sensor(Data.RunningEncoders, True)
    LocoRobo.wait(2.0)
    #thread to get sensor values and map them to drone commands
    #threading.Thread(target = self._throttle_control).start()
    #thread to send commands to the drone
    # Main run loop
    while True:

        # Get accelerometer values from the control robot
        acc = robot.get_sensor_value(Data.Accelerometer)

        # Map the corresponding axis values to a variable
        x = acc['x']
        y = acc['y']
        z = acc['z']

        # Get the angle of the x-axis and y-axis in radians
        pitch_rad = math.atan(x / math.sqrt(y**2 + z**2))
        roll_rad = math.atan(y / math.sqrt(x**2 + z**2))

        # Convert the angles of the x-axis and y-axis to degrees
        pitch_deg = int((pitch_rad*180)/math.pi * -1)
        roll_deg = int((roll_rad*180)/math.pi)

        tickcounts = robot.get_sensor_value(Data.RunningEncoders)
        left_encoder = tickcounts['left']
        right_encoder = tickcounts['right']


        # print("Throttle: ", right_encoder)
        # print("Yaw: ", left_encoder)
        # print("Roll: ", roll_deg)
        # print("Pitch: ", pitch_deg)

        drone.set_throttle(127)
        drone.set_yaw(127)

        drone.set_roll(map_range(roll_deg,
                              -MAX_TILT_ANGLE,
                               MAX_TILT_ANGLE,
                               0,
                               255))

        drone.set_pitch(map_range(pitch_deg,
                                 -MAX_TILT_ANGLE,
                                 MAX_TILT_ANGLE,
                                 0,
                                 255))

        drone.update_payload()
        #LocoRobo.wait(0.1)


    # Unbind from drone
    drone.unbind()

    # Deactivate motors and disconnect from robot
    robot.deactivate_motors()
    robot.disconnect()

if __name__ == "__main__":

    try:
        main()
    except:
        LocoRobo.stop()
        raise
    LocoRobo.stop()


