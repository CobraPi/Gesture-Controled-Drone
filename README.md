This contains the source code of the LocoDrone project


concepts - includes the power point proposals explaining the features of both the gesture control and a more advanced autonomous
	   flight system. 


drone-control - the code for using a custom designed prototype transmitter that included Joystics and an accelerometer. Meant to
                be tethered to a the serial port of the computer, and commuincates with a python master control program.Also
                included in the Python code is a prototype feature, that allows the LocoRobo ground robots to communicate with
                the drone, allowing it's accelerometer to be used to control the attitude of the drone, and the moter encoders
                to control the throttle and yaw respectively. Also included is a Python script for open-loop pseudo autonomous
                operation of the drone.


gesture-control - the code for the wearable glove transmitter, which was the main prototype for the final product. Glove used a 
		  9 - DOF IMU to control the attitude of the drone and flex sensors in the fingers to control throttle and yaw
		  respectively. It also included a led light ring, which changed according to the position of the glove.


imu-3D-visualization - the code for the realtime visualisation of the absolute orientation of the drone, controlled by an 
 			accelerometer. Acclelerometer sends in it's positional data via the serial port of the computer, which
			is then used by a MATLAB script to homogenously transform an stl CAD file around the x and y axis, mimicking
			the absolute orientation of the accelerometer. Implemented with the directional cosine matrix in MATLAB in 
			the file: 3D Visualisation.m 
