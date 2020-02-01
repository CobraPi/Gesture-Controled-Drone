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
from locorobo.log import set_log
from errors import DroneBindException, SerialClosedException, \
                   DroneUnbindException, DroneReconnectException
import math
import time
from locodrone import LocoDrone


MAX_TILT_ANGLE = 80


def map_range(x, in_min, in_max, out_min, out_max):
    return int(((x - in_min) * (out_max - out_min)) / ((in_max - in_min) + out_min))


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

    # Tell LocoRobo what serial port to use
    LocoRobo.setup("/dev/tty.usbmodem1")

    # Scan for robots for 2000 ms
    robots = LocoRobo.scan(2000)

    # Use get_robots to find robot with name "lr 67:c6" in the scan result
    robot = get_robot(robots, "joey")

    robot.connect()
    robot.enable_sensor(Data.Accelerometer, True)
    LocoRobo.wait(2)
    # Create the drone serial object
    drone = LocoDrone('/dev/tty.usbmodem1A1211', 115200)
    # Open the serial port
    drone.open_serial()
    drone.serial_control()
    drone.bind()

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
        pitch_deg = -1 * (pitch_rad*180)/math.pi
        roll_deg = (roll_rad*180)/math.pi

        pitch = map_range(pitch_deg, -MAX_TILT_ANGLE, MAX_TILT_ANGLE, 0, 255)
        roll = map_range(roll_deg, -MAX_TILT_ANGLE, MAX_TILT_ANGLE, 0, 255)

        throttle = 30
        yaw = 127

        drone.set_throttle(throttle)
        drone.set_yaw(yaw)
        drone.set_roll(roll)
        drone.set_pitch(pitch)
        """
        print("Throttle: ", drone.get_throttle(),
              "   Yaw: ", drone.get_yaw(),
              "   Roll: ", drone.get_roll(),
              "   Pitch: ", drone.get_pitch())
        """

        drone.update_payload()

        debug_payload = []
        control = drone.read_byte()
        length = drone.read_byte()
        if control == 255 and length == 8:
            for i in range(8):
                debug_payload.append(drone.read_byte())
        print(debug_payload)
        LocoRobo.wait(0.05)
if __name__ == "__main__":

    try:
        main()
    except:
        LocoRobo.stop()
        raise
    LocoRobo.stop()



