from __future__ import print_function
from __future__ import absolute_import

try:
    input = raw_input
except NameError:
    pass


def read_accelerometer(serial, calibration):
    """
    Reads the raw values from the Arduino, parses them into separate variables
    and uses the calibration data to normalize the data

    Args:
        serial: a reference to the serial connection with the Arduino
        calibration: a reference to the calibration object that holds the
                     values from the accelerometer calibration process
    Returns:
        (x_cal, y_cal, z_cal): a tuple of the normalized data
    """

    components = serial.read_str()
    # parses the string from the Arduino into three separate variables
    x_raw, y_raw, z_raw = tuple(map(float, components.split(',')))

    # normalizes the data using the calibration information
    x_cal = (x_raw - calibration.offset[0]) / (calibration.gain[0])
    y_cal = (y_raw - calibration.offset[1]) / (calibration.gain[1])
    z_cal = (z_raw - calibration.offset[2]) / (calibration.gain[2])

    return (x_cal, y_cal, z_cal)


def calibrate(serial):
    """
    The calibration process for the accelerometer, walks the user through
    calibrating the accelerometer

    Args:
        serial: a reference to the serial connection with the Arduino
    Returns:
        an instance of the Calibration object with calibration data for the
        current accelerometer
    """

    blank_calibration = Calibration((0, 0, 0), (1, 1, 1))

    print('Lay the accelerometer on a flat surface.')
    input('Press ENTER to continue')
    serial.write('A')
    gx_z, gy_z, gz_z = read_accelerometer(serial, blank_calibration)

    print('Stand the accelerometer on edge so that the X arrow points up.')
    input('Press ENTER to continue')
    serial.write('A')
    gx_x, gy_x, gz_x = read_accelerometer(serial, blank_calibration)

    print('Stand the accelerometer on edge so that the Y arrow points up.')
    input('Press ENTER to continue')
    serial.write('A')
    gx_y, gy_y, gz_y = read_accelerometer(serial, blank_calibration)

    offset_x = (gx_z + gx_y) / 2
    offset_y = (gy_x + gy_z) / 2
    offset_z = (gz_x + gz_y) / 2

    gain_x = gx_x - offset_x
    gain_y = gy_y - offset_y
    gain_z = gz_z - offset_z

    print('Calibration Complete')

    return Calibration((offset_x, offset_y, offset_z),
                       (gain_x, gain_y, gain_z)
                       )


class Calibration:

    def __init__(self, offset, gain):
        self.offset = offset
        self.gain = gain
