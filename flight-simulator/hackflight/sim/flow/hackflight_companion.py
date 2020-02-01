#!/usr/bin/env python
'''
   hackflight_companion.py : Companion-board Python code.  Runs in 
   Python2 instead of Python3, so we can install OpenCV without 
   major hassles.

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
'''

import sys
import cv2
import numpy as np
import threading

from msppg import MSP_Parser, serialize_ATTITUDE_Request, serialize_ALTITUDE_Request, serialize_SET_HEAD

def commsReader(comms_from_client, parser):

    while True:

        # Read one byte from the client and parse it
        bytes = comms_from_client.recv(1)
        if len(bytes) > 0:
            parser.parse(bytes[0])

def putTextInImage(image, text, x, y, scale, color, thickness=1):

    cv2.putText(image, text, (x,y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness)

def processImage(image, parser, comms_to_client):

    # Blur image to remove noise
    frame = cv2.GaussianBlur(image, (3, 3), 0)

    # Switch image from BGR colorspace to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range of blue color in HSV
    bluemin = (100,  50,  10)
    bluemax = (255, 255, 255)

    # Find where image is in blue range
    bluepart = cv2.inRange(hsv, bluemin, bluemax)

    # Find coordinates of blue pizels
    y, x = np.where(bluepart)

    # If a signficant fraction of the pixels are blue
    if len(x) / float(np.prod(bluepart.shape)) > 0.2:

        # Find the centroid of the blue component
        x,y = np.int(np.mean(x)), np.int(np.mean(y))

        # Label the centroid point as water
        putTextInImage(image, 'WATER', x, y, 1, (0,255,255), 2)

        # If we've just seen water for the first time, send a SET_HEADING message to the client
        if not parser.over_water:

            new_heading = parser.heading - 180
            print('set head: %d' % new_heading)

            if not comms_to_client is None:

                comms_to_client.send(serialize_SET_HEAD(new_heading))

        # Set a flag that we've seen water
        parser.over_water = True

    # Add text for altitude
    labelx = 5
    labely = 10
    labelw = 270
    labelh = 20
    labelm = 5 # margin
    cv2.rectangle(image, (labelx,labely), (labelx+labelw,labely+labelh), (255,255,255), -1) # filled white rectangle
    putTextInImage(image, 'ABL = %3.2f m | Heading = %d' % (parser.altitude/100., parser.heading),
            labelx+labelm, labely+labelh-labelm, .5, (255,0,0))


class MyParser(MSP_Parser):

    def __init__(self):

        MSP_Parser.__init__(self)

        self.altitude = 0
        self.heading = 0

        self.over_water = False

    def altitudeHandler(self, altitude, vario):

        self.altitude = altitude

    def attitudeHandler(self, pitch, roll, yaw):

        self.heading = yaw

if __name__ == '__main__':

    # Create an MSP parser and messages for telemetry requests
    parser = MyParser()
    parser.set_ATTITUDE_Handler(parser.attitudeHandler)
    parser.set_ALTITUDE_Handler(parser.altitudeHandler)

    # Serialize the telemetry message requests that we'll send to the "firwmare"
    attitude_request = serialize_ATTITUDE_Request()
    altitude_request = serialize_ALTITUDE_Request()

    # More than two command-line arguments means simulation mode.  First arg is camera-client port, 
    # second is MSP port, third is input image file name, fourth is outpt image file name.
    if len(sys.argv) > 2:

        from socket_server import serve_socket

        # Serve a socket for camera synching, and a socket for comms
        camera_client = serve_socket(int(sys.argv[1]))
        comms_to_client  = serve_socket(int(sys.argv[2]))
        comms_from_client  = serve_socket(int(sys.argv[3]))
        image_from_sim_name  = sys.argv[4]
        image_to_sim_name  = sys.argv[5]

        # Run serial comms telemetry reading on its own thread
        thread = threading.Thread(target=commsReader, args = (comms_from_client,parser))
        thread.daemon = True
        thread.start()

        while True:

            # Receive the camera sync byte from the client
            camera_client.recv(1)
         
            # Load the image from the temp file
            image = cv2.imread(image_from_sim_name, cv2.IMREAD_COLOR)

            # Process it
            processImage(image, parser, comms_to_client)

            # Write the processed image to a file for the simulator to display
            cv2.imwrite(image_to_sim_name, image)

            # Send an telemetry request messages to the client
            comms_to_client.send(attitude_request)
            comms_to_client.send(altitude_request)

    # Fewer than three arguments: live mode or camera-test mode
    else:

        commport = sys.arg[1] if len(sys.argv) > 1 else None

        cap = cv2.VideoCapture(0)

        while True:

            success, image = cap.read()

            if success:

                # Process image
                processImage(image, parser, None) 

                # Test mode; display image
                if commport is None:
                    cv2.imshow('OpenCV', image)
                    if cv2.waitKey(1) == 27:  # ESC
                        break


