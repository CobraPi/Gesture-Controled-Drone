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

    last_right_encoder = 0
    last_left_encoder = 0
    last_heading = 0

    # Create the drone serial object
    drone = LocoDrone('/dev/tty.usbmodem1411', 115200)

    # Open the serial port
    drone.open_serial()

    user_control(drone)

    # Tell LocoRobo what serial port to use
    LocoRobo.setup("/dev/tty.usbmodem1")

    # Scan for robots for 2000 ms
    robots = LocoRobo.scan(2000)

    # Use get_robots to find robot with name "lr 67:c6" in the scan result
    robot = get_robot(robots, "joey")

    robot.connect()
    robot.enable_sensor(Data.Accelerometer, True)
    robot.enable_sensor(Data.RunningEncoders, True)
    # robot.enable_sensor(Data.MagnetometerRaw, True)
    #robot.enable_sensor(Data.Heading, True)
    time.sleep(5)

    while True:

        try:

            # Get accelerometer values from the control robot
            acc = robot.get_sensor_value(Data.Accelerometer)

            # Get magnetometer data from sensor
            #heading = robot.get_sensor_value(Data.Heading)

            # Map the corresponding axis values to a variable
            x = acc['x']
            y = acc['y']
            z = acc['z']

            # Get the angle of the x-axis and y-axis in radians
            pitch_rad = math.atan(x / math.sqrt(y**2 + z**2))
            roll_rad = math.atan(y / math.sqrt(x**2 + z**2))

            # Convert the angles of the x-axis and y-axis to degrees
            pitch_deg = round(((pitch_rad*180)/math.pi), 0)
            roll_deg = round(((roll_rad*180)/math.pi), 0)
            pitch_deg *= -1

            # Get the encoder ticks for both the left and right encoders
            tickcounts = robot.get_sensor_value(Data.RunningEncoders)
            left_encoder = -(tickcounts['left'] / 20)
            right_encoder = -(tickcounts['right']/20)

            # Use the encoders to
            yaw_diff = left_encoder - last_left_encoder
            last_left_encoder = left_encoder

            throttle_diff = right_encoder - last_right_encoder
            last_right_encoder = right_encoder

            yaw = drone.get_yaw() - yaw_diff
            throttle = drone.get_throttle() - throttle_diff

            pitch = map_range(pitch_deg, -MAX_TILT_ANGLE, MAX_TILT_ANGLE, 0, 255)
            roll = map_range(roll_deg, -MAX_TILT_ANGLE, MAX_TILT_ANGLE, 0, 255)

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

    user_control()
    main()



