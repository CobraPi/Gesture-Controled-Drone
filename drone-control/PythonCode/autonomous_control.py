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
import serial
import time
from control_codes import IncomingMessageType, OutgoingMessageType, \
                          OperatingModeType, ControlSourceType, InitializationFailureType
from locodrone import LocoDrone


#set_log(True, True, True)


MAX_TILT_ANGLE = 90



def map_range(x, in_min, in_max, out_min, out_max):
    return round(((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min), 0)


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


def user_control(drone):
    print("Press '1' to Bind to drone")
    print("Press '2' to Unbind from drone")
    print("Press '3' to Reconnect to drone")
    print("Press '4' to Toggle Serial Control")
    print("Press '5' to Toggle Accelerometer Control")
    print("Press '6' to Toggle Joystick Control")
    print("Press '7' to Calibrate Accelerometer")
    print("Press '8' to Toggle Drive Mode")
    print("Press '9' to Toggle Fly Mode")

    while True:
        user_input = input("")

        if user_input == '1':
            drone.bind()

        elif user_input == '2':
            drone.unbind()

        elif user_input == '3':
            drone.reconnect()

        elif user_input == '4':
            drone.serial_control()
            break

        elif user_input == '5':
            drone.accelerometer_control()

        elif user_input == '6':
            drone.joystick_control()

        elif user_input == '7':
            drone.request(OutgoingMessageType.CalibrateAccelerometer)

        elif user_input == '8':
            drone.drive_mode()

        elif user_input == '9':
            drone.fly_mode()

################# Main Function ###############################

def main():

    # Create the drone serial object
    drone = LocoDrone('/dev/tty.usbmodem1411', 115200)
    drone.open_serial()
    drone.serial_control()
    drone.bind()
    drone.drive_mode()
    drone.set_roll_trim(70)
    drone.set_pitch_trim(60)
    drone.fly_up()
    drone.turn_right(10)
    drone.land()
    drone.unbind()





if __name__ == "__main__":

    try:
        main()
    except:
        LocoRobo.stop()
        raise
    LocoRobo.stop()


