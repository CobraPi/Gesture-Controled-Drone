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
    drone = LocoDrone('/dev/tty.usbmodem1421', 115200)

    # Open the serial port
    drone.open_serial()

    user_control(drone)


    drone.set_throttle(int(throttle))
    drone.set_yaw(int(yaw))
    drone.set_pitch(int(pitch))
    drone.set_roll(int(roll))

    drone.update_payload()

except ZeroDivisionError as e:
    print('Got Zero Division', e)

LocoRobo.wait(0.01)


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


