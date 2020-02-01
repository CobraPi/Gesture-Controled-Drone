 # By Matt Fairfield
 # - mcf62@drexel.edu (you can email me if you have any questions about the code or how it works)
 # - 3/1/2016

from locorobo import LocoRobo
from locorobo import Data
from locorobo import MotorDirection
from locorobo import WaitType
from locorobo import Song
from locorobo import Note
from collections import deque
from struct import *
import threading
import math
import time
import serial
import random
import tkinter
import socket

ARD_com_port = '/dev/tty.usbmodem1411'        # Change to com port of arduino
BT_com_port = '/dev/tty.usbmodem1'         # Change to com port of LocoRobo bluetooth dongle
name = 'lr 67:cd'             # Change to LocoRobo robot's name
UDP_IP  = '192.168.173.149'  # Change to current IP Address
UDP_PORT = 1337              # Change to any open socket

class roboGUI(tkinter.Frame):

    def __init__(self,parent):
        tkinter.Frame.__init__(self,parent)
        self.parent = parent
        self.initUI()
        self.MAX_ANGLE = 75
        self.walk_state = 0
        self.connections = 0
        self.total_connections = 3 # Total connections needed to enable all control buttons
        self.fly_loop = False
        self.follow_loop = False
        self.phone_data_loop = False
        self.throttle_ctrl_loop = False
        self.phoneData = None
        self.ser = None

    #########################################
    ############### GUI SETUP  ##############
    def initUI(self):
        self.parent.title("LocoRobo Flybot GUI")
        self.parent.geometry('665x480+400+100')
        self.colSPACE = tkinter.Button(self.parent,state='disable', width=12, height=1,relief=tkinter.FLAT).grid(row = 1, column = 0)
        self.colSPACE = tkinter.Button(self.parent,state='disable', width=12, height=1,relief=tkinter.FLAT).grid(row = 1, column = 6)
        # Connections
        self.connectLabel = tkinter.Label(self.parent,text = "CONNECTIONS", fg="red").grid(row=0, column=0,columnspan=7)
        self.startPhone = tkinter.Button(self.parent,text="Connect Phone",fg="white", bg="red", width=12, height=1,command=self.CONNECT_PHONE)
        self.startPhone.grid(row=1, column=2)
        self.startBot = tkinter.Button(self.parent,text="Connect Robot",fg="white", bg="red", width=12, height=1,command=self.CONNECT_ROBOT)
        self.startBot.grid(row=1, column=3)
        self.startDrone = tkinter.Button(self.parent,text="Connect Drone",fg="white", bg="red", width=12, height=1,command=self.CONNECT_DRONE)
        self.startDrone.grid(row=1, column=4)
        # Fly/Follow Modes
        self.rowSPACE = tkinter.Label(self.parent,text = "   ").grid(row=2,columnspan=6)
        self.modesLabel = tkinter.Label(self.parent,text = "FLY/FOLLOW MODES", fg="green").grid(row=3, columnspan=7)
        self.startFly = tkinter.Button(self.parent,text="Fly Drone",fg="purple", width=12, height=1,command=self.MAKE_FLY)
        self.startFly.grid(row = 4, column = 2)
        self.startFly.configure(state='disable')
        self.startFollow = tkinter.Button(self.parent,text="Follow Robot",fg="purple", width=12, height=1,command=self.MAKE_FOLLOW)
        self.startFollow.grid(row = 4, column = 4)
        self.startFollow.configure(state='disable')
        self.stopFlight = tkinter.Button(self.parent,text="Stop Flight",fg="black", width=12, height=1,command=self.STOP_FLIGHT)
        self.stopFlight.grid(row = 4, column = 3)
        self.stopFlight.configure(state='disable')
        # Throttle Controls
        self.rowSPACE = tkinter.Label(self.parent,text = "   ").grid(row=5,columnspan=7)
        self.throttleLabel = tkinter.Label(self.parent,text = "THROTTLE CONTROLS", fg="green").grid(row=6, columnspan=7)
        self.stopThrottle = tkinter.Button(self.parent,text="Throttle Off",state='disable',fg="red", width=12, height=1,command=self.THROTTLE_OFF)
        self.stopThrottle.grid(row = 7, column = 1)
        self.startLaunchT = tkinter.Button(self.parent,text="Throttle Launch",state='disable',fg="green", width=12, height=1,command=self.THROTTLE_LAUNCH)
        self.startLaunchT.grid(row = 7, column = 2)
        self.freezeControlT = tkinter.Button(self.parent,text="Freeze Throttle",state='disable',fg="blue", width=12, height=1, command=self.FREEZE_CONTROL)
        self.freezeControlT.grid(row = 7, column = 4)
        self.startControlT = tkinter.Button(self.parent,text="Throttle Control",state='disable',fg="blue", width=12, height=1, command=self.CONTROL_THROTTLE)
        self.startControlT.grid(row = 7, column = 5)
        # Fine Adjustment Controls
        self.up_arrow = tkinter.PhotoImage(file="pics/up.gif")
        self.left_arrow = tkinter.PhotoImage(file="pics/left.gif")
        self.right_arrow = tkinter.PhotoImage(file="pics/right.gif")
        self.down_arrow = tkinter.PhotoImage(file="pics/down.gif")
        self.center = tkinter.PhotoImage(file="pics/center.gif")
        bSize = 50
        self.rowSPACE = tkinter.Label(self.parent,text = "   ").grid(row=8,columnspan=7)
        self.trimLabel = tkinter.Label(self.parent,text = "TRIM CONTROLS", fg="orange").grid(row=9,column=0,columnspan=3)
        self.motorLabel = tkinter.Label(self.parent,text = "MOTOR CONTROLS", fg="orange").grid(row=9,column=4,columnspan=3)
        self.trimUpArrow = tkinter.Button(self.parent,command=self.TRIM_FORWARD ,image=self.up_arrow,state='disable', width=bSize, height=bSize)
        self.trimUpArrow.grid(row = 10, column = 1)
        self.trimLeftArrow = tkinter.Button(self.parent,command=self.TRIM_LEFT ,image=self.left_arrow,state='disable', width=bSize, height=bSize)
        self.trimLeftArrow.grid(row = 11, column = 0)
        self.trimRightArrow = tkinter.Button(self.parent,command=self.TRIM_RIGHT ,image=self.right_arrow,state='disable', width=bSize, height=bSize)
        self.trimRightArrow.grid(row = 11, column = 2)
        self.trimDownArrow = tkinter.Button(self.parent,command=self.TRIM_BACKWARD ,image=self.down_arrow,state='disable', width=bSize, height=bSize)
        self.trimDownArrow.grid(row = 12, column = 1)
        self.motorUpArrow = tkinter.Button(self.parent,command=self.PITCH_FORWARD ,image=self.up_arrow,state='disable', width=bSize, height=bSize)
        self.motorUpArrow.grid(row = 10, column = 5)
        self.motorLeftArrow = tkinter.Button(self.parent,command=self.YAW_LEFT ,image=self.left_arrow,state='disable', width=bSize, height=bSize)
        self.motorLeftArrow.grid(row = 11, column = 4)
        self.motorRightArrow = tkinter.Button(self.parent,command=self.YAW_RIGHT ,image=self.right_arrow,state='disable', width=bSize, height=bSize)
        self.motorRightArrow.grid(row = 11, column = 6)
        self.motorDownArrow = tkinter.Button(self.parent,command=self.PITCH_BACKWARD ,image=self.down_arrow,state='disable', width=bSize, height=bSize)
        self.motorDownArrow.grid(row = 12, column = 5)
        self.motorCenter = tkinter.Button(self.parent,command=self.MOVE_CENTER ,image=self.center,state='disable', width=bSize, height=bSize)
        self.motorCenter.grid(row = 11, column = 5)
        # Shutdown Everything and Close (can be used at any point to close gracefully)
        self.rowSPACE = tkinter.Label(self.parent,text = "   ").grid(row=13,columnspan=7)
        self.shutdown = tkinter.Button(self.parent,text="Shutdown",font='bold',fg="red",bg="black", width=8, height=2,command=self.SHUTDOWN)
        self.shutdown.grid(row = 14, column = 6)

    def enableButtons(self):    # Called after all connections are established to enable all buttons
        self.startPhone.configure(state='normal')
        self.startBot.configure(state='normal')
        self.startDrone.configure(state='normal')
        self.startFly.configure(state='normal')
        self.startFollow.configure(state='normal')
        self.stopFlight.configure(state='normal')
        self.stopThrottle.configure(state='normal')
        self.startLaunchT.configure(state='normal')
        self.freezeControlT.configure(state='normal')
        self.startControlT.configure(state='normal')
        self.trimUpArrow.configure(state='normal')
        self.trimLeftArrow.configure(state='normal')
        self.trimRightArrow.configure(state='normal')
        self.trimDownArrow.configure(state='normal')
        self.motorUpArrow.configure(state='normal')
        self.motorLeftArrow.configure(state='normal')
        self.motorRightArrow.configure(state='normal')
        self.motorDownArrow.configure(state='normal')
        self.motorCenter.configure(state='normal')
    ############# GUI SETUP END  ############
    #########################################

    ################################################
    ############# GUI BUTTON FUNCTIONS  ############ These methods pause functionality of the GUI while executing(blocking functions)
    def CONNECT_PHONE(self):
        print ("--Locating Phone--\n")
        print ( "Receiver IP: ", UDP_IP, "  UDP Port #: ", UDP_PORT)
        print ( "\nOpen SensorUDP app on your phone and enable Orientation.")
        print ( "\nEnter IP address and port number shown then click 'SEND DATA'.\n")
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        connect_phone_loop = False
        try:
            self.s.bind((UDP_IP, UDP_PORT))
            self.s.settimeout(1)
            timeout = time.time() + 20
            connect_phone_loop = True
        except:
            print ("\n\n    Check for correct IP Address (Global Variables) \n\n    Use 'ipconfig' \n")
            time.sleep(2)
            print ("    SHUTTING DOWN \n\n")
            time.sleep(2)
            self.SHUTDOWN()
        while connect_phone_loop:
            print("  timeout in: ", '{0:2d}'.format(int(timeout)-int(time.time())), end="\r")
            try:
                recv_test = self.s.recv(96)
                if recv_test:
                    threading.Thread(target = self._phone_data).start()
                    print("<<--PHONE CONNECTED-->>\n")
                    self.startPhone.configure(state='disable', bg = "green")  # Update GUI
                    self.connections += 1
                    self.enableButtons()
                    if self.connections == self.total_connections:
                        self.enableButtons()
                    else:
                        self.startBot.configure(state='normal')   # Update GUI
                        self.startDrone.configure(state='normal') # Update GUI
                    break
            except:
                pass
            if time.time() > timeout:
                print("****Phone connection timeout please try again****\n")
                break

    def CONNECT_ROBOT(self):
        print("--Locating ", name, "--\n")
        LocoRobo.setup(BT_com_port)
        robots = LocoRobo.scan(2000)
        self.robot = None
        for r in robots.values():
            if r.name == name:
                self.robot = r
                break
        if self.robot != None:
            self.robot.connect()
            self.robot.activate_motors()
            self.robot.enable_sensor(Data.ACCELEROMETER, True)
            self.robot.enable_sensor(Data.ULTRASONIC, True)
            print("<<--", name, " CONNECTED-->>\n")
            self.startBot.configure(state='disable', bg = "green") # Update GUI
            self.connections += 1
            self.enableButtons()
            if self.connections == self.total_connections:
                self.enableButtons()
        else:
            print("****Could not find robot try again****\n")

    def CONNECT_DRONE(self):
        try:
            self.ser = serial.Serial(port=ARD_com_port ,baudrate=19200,timeout=1)
            print("--Locating Drone--\n")
            time.sleep(3)   # Wait for serial to initialize
            self.ser.write(bytes('0', 'utf-8')) # Drone binding character
            timeout = time.time() + 8
            ##### MUST HAVE MY MODIFIED HCD.ZIP ADDED TO ARDUINO LIBRARY TO RECIVE PROPER ACK's #####
            while 1:                                                                            #####
                ack = self.ser.read(3)                                                          #####
                self.ser.read(2) # cleanup serial trash from ACK                                #####
                if ack == bytes("ACK", 'utf-8'):                                                #####
                    while 1:                                                                    #####
                        ack2 = self.ser.read(3)                                                 #####
                        if ack2 == bytes("ACK", 'utf-8'):                                       #####
                            time.sleep(2)                                                       #####
                            print("<<--DRONE CONNECTED-->>\n")                                  #####
                            self.startDrone.configure(state='disable', bg = "green")            #####
                            self.connections += 1                                               #####
                            self.enableButtons()
                            if self.connections == self.total_connections:                      #####
                                self.enableButtons()                                            #####
                            break                                                               #####
           ##########################################################################################
                    break
                elif time.time() > timeout:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                    self.ser.close()
                    self.ser = None
                    print("****Could not find the Drone try again****\n")
                    break
                else:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                    self.ser.write(bytes('0', 'utf-8')) # Resend drone binding character
        except:
            print("****Could not open COM port check arduino****\n")

    def SHUTDOWN(self):
        self.fly_loop = False
        self.follow_loop = False
        self.throttle_ctrl_loop = False
        self.phone_data_loop = False
        try:
            self.ser.flushInput()
            self.ser.write(bytes('9', 'utf-8')) # Throttle drop to 0
            self.ser.close()
            self.robot.deactivate_motors()
            self.robot.disconnect()
            self.s.shutdown(socket.SHUT_RDWR)
            self.s.close()
        except:
            pass
        print("~ ~ ~ DISCONNECTED ~ ~ ~\n")
        time.sleep(1)
        self.parent.quit()

    def MAKE_FOLLOW(self):
        if (self.follow_loop == False and self.fly_loop == False):
            self.startFollow.configure( bg = "green") # Update GUI
            self.stopFlight.configure( bg = "orange") # Update GUI
            threading.Thread(target = self._follow).start()
        elif (self.fly_loop == True):
            print("In FLYING MODE, please press 'Stop Flight' button first.")
            time.sleep(0.5)
            print("Then place robot on ground for random walk.")
        else:
            print("Already Following")

    def MAKE_FLY(self):
        if (self.follow_loop == False and self.fly_loop == False):
            self.startFly.configure( bg = "green")    # Update GUI
            self.stopFlight.configure( bg = "orange") # Update GUI
            threading.Thread(target = self._fly).start()
        elif (self.follow_loop == True):
            print("In FOLLOW MODE, please press 'Stop Flight' button first.")
            time.sleep(0.5)
            print("Place robot vertically on table if throttle control is needed.")
        else:
            print("Already Flying")

    def CONTROL_THROTTLE(self):
        if self.follow_loop == True:
            print("In FOLLOW MODE, please press 'Stop Flight' button to use robot as throttle")
        elif (self.fly_loop == True):
            self.startControlT.configure( bg = "green")  # Update GUI
            self.freezeControlT.configure( bg = "white") # Update GUI
            threading.Thread(target = self._throttle_control).start()
        else:
            print("Select FLY MODE for drone control")
            self.startControlT.configure( bg = "green")  # Update GUI
            self.freezeControlT.configure( bg = "white") # Update GUI
            threading.Thread(target = self._throttle_control).start()

    def STOP_FLIGHT(self):
        self.fly_loop = False       # Stop Fly Mode Control
        self.follow_loop = False    # Stop Follow Mode Control
        self.startFly.configure( bg = "white")    # Update GUI
        self.startFollow.configure( bg = "white") # Update GUI
        self.stopFlight.configure( bg = "white")  # Update GUI

    def FREEZE_CONTROL(self):
        if self.throttle_ctrl_loop:
            self.throttle_ctrl_loop = False # Freeze Throttle Control
            self.startControlT.configure( bg = "yellow")  # Update GUI
            self.freezeControlT.configure( bg = "yellow") # Update GUI

    def THROTTLE_LAUNCH(self):
        self.throttle_ctrl_loop = False
        self.ser.write(bytes('8', 'utf-8')) # Throttle launch to 200 of 255
        self.startControlT.configure( bg = "white")  # Update GUI
        self.freezeControlT.configure( bg = "white") # Update GUI

    def THROTTLE_OFF(self):
        self.throttle_ctrl_loop = False
        self.ser.flushInput()
        self.ser.write(bytes('9', 'utf-8')) # Throttle drop to 0
        self.startControlT.configure( bg = "white")  # Update GUI
        self.freezeControlT.configure( bg = "white") # Update GUI

    # TRIM CONTROL BUTTONS
    def TRIM_FORWARD(self):
        self.ser.write(bytes('t', 'utf-8')) # Pitch trim inc
    def TRIM_BACKWARD(self):
        self.ser.write(bytes('g', 'utf-8')) # Pitch trim dec
    def TRIM_LEFT(self):
        self.ser.write(bytes('f', 'utf-8')) # Yaw trim inc
    def TRIM_RIGHT(self):
        self.ser.write(bytes('h', 'utf-8')) # Yaw trim dec

    # MOTOR CONTROL BUTTONS
    def PITCH_FORWARD(self):
        if self.fly_loop:
            for i in range(5):  self.ser.write(bytes('i', 'utf-8')) # Pitch inc by range amount
    def PITCH_BACKWARD(self):
        if self.fly_loop:
            for i in range(5):  self.ser.write(bytes('k', 'utf-8')) # Pitch dec by range amount
    def YAW_LEFT(self):
        if self.fly_loop:
            for i in range(5):  self.ser.write(bytes('j', 'utf-8')) # Yaw inc by range amount
    def YAW_RIGHT(self):
        if self.fly_loop:
            for i in range(5):  self.ser.write(bytes('l', 'utf-8')) # Yaw dec by range amount
    def MOVE_CENTER(self):
        self.ser.write(bytes('5', 'utf-8'))
    ############# BUTTON FUNCTIONS END #############
    ################################################

    ###################################
    ############# THREADS #############   Threads allow looping while still having control in the GUI(non-blocking functions)
    def _phone_data(self):
        self.phone_data_loop = True
        while self.phone_data_loop:
            try:
                self.phoneData = self.s.recv(96) # 96 = byte length of UDP data sent from phone
            except:
                print("****Phone Connection Lost Reconnect****\n")
                try:
                    self.ser.write(bytes('5', 'utf-8'))      # Stop current pitch and yaw. Throttle remains the same.
                except:
                    pass
                self.startPhone.configure(state='normal', bg="red") # Enable phone connect button
                self.connections -= 1
                self.startBot.configure(state='disable')            # Disable buttons untill phone is re-connected
                self.startDrone.configure(state='disable')          #
                self.startFly.configure(state='disable')            #
                self.startFollow.configure(state='disable')         #
                self.stopFlight.configure(state='disable')          #
                self.startLaunchT.configure(state='disable')        #
                self.freezeControlT.configure(state='disable')      #
                self.startControlT.configure(state='disable')       #
                try: # Phone Disconnect Tones (will not play if robot is not connected)
                    for i in range(6):
                        self.robot.play_note(38, 150, True) # '38' cooresponds to the highest pitch tone robot can make
                        time.sleep(0.05)
                    self.robot.play_note(Note.E, 800, True)
                except:
                    pass
                break

    def _fly(self):
        self.fly_loop = True
        NUM_POINTS = 4                  # Exponential moving average setup
        multp = (2/(NUM_POINTS+1))      #
        pitchList = [0] * NUM_POINTS    #
        pitchList = deque(pitchList)    #
        pitchEMA = 0                    #
        rollList = [0] * NUM_POINTS     #
        rollList = deque(rollList)      #
        yawEMA = 0                      #
        while self.fly_loop:
            pitch = unpack_from ('!f', self.phoneData, 40) # 40 = byte offset of Orientation Pitch
            roll = unpack_from ('!f', self.phoneData, 44)  # 44 = byte offset of Orientation Roll(used as Yaw)
            pitchList.pop()
            rollList.pop()
            pitchList.appendleft( float(pitch[0]) )
            rollList.appendleft( float(roll[0]) )
            pitchEMA = ( ( pitchList[0] - pitchEMA ) * multp + pitchEMA )   # Pitch Moving Average
            yawEMA = ( ( rollList[0] - yawEMA ) * multp + yawEMA )          # Yaw Moving Average
            pitch_angle = int(pitchEMA)
            yaw_angle = int(yawEMA)

            if pitch_angle > (self.MAX_ANGLE-10) :   self.ser.write(bytes('X', 'utf-8')) # Pitch Forward HI
            elif pitch_angle > 55 :                  self.ser.write(bytes('W', 'utf-8')) # Pitch Forward MED
            elif pitch_angle > 20 :                  self.ser.write(bytes('w', 'utf-8')) # Pitch Forward LOW
            elif pitch_angle < -(self.MAX_ANGLE-10) :self.ser.write(bytes('x', 'utf-8')) # Pitch Backward HI
            elif pitch_angle < -55 :                 self.ser.write(bytes('S', 'utf-8')) # Pitch Backward MED
            elif pitch_angle < -20 :                 self.ser.write(bytes('s', 'utf-8')) # Pitch Backward LOW
            else :                                   self.ser.write(bytes('3', 'utf-8')) # Pitch center

            if   yaw_angle > (self.MAX_ANGLE-10) :   self.ser.write(bytes('Z', 'utf-8')) # Yaw Right HI
            elif yaw_angle > 45:                     self.ser.write(bytes('D', 'utf-8')) # Yaw Right MED
            elif yaw_angle > 20:                     self.ser.write(bytes('d', 'utf-8')) # Yaw Right LOW
            elif yaw_angle < -(self.MAX_ANGLE-10) :  self.ser.write(bytes('z', 'utf-8')) # Yaw Left HI
            elif yaw_angle < -45 :                   self.ser.write(bytes('A', 'utf-8')) # Yaw Left MED
            elif yaw_angle < -20 :                   self.ser.write(bytes('a', 'utf-8')) # Yaw Left LOW
            else :                                   self.ser.write(bytes('1', 'utf-8')) # Yaw center
            LocoRobo.wait(0.01)

    def _follow(self):
        self.follow_loop = True
        threading.Thread(target = self._random_walk).start()
        while self.follow_loop:
            if   self.walk_state == 1 :  self.ser.write(bytes('o', 'utf-8')) # Follow Pitch
            elif self.walk_state == 2 :  self.ser.write(bytes('p', 'utf-8')) # Follow Yaw
            else:                        self.ser.write(bytes('5', 'utf-8')) # Follow Center All
            self.ser.flushInput()
            LocoRobo.wait(0.01)

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

    def _random_walk(self):
        for i in range(20):
            if self.follow_loop == False:
                return
            mov = int(random.random()*70000)
            self.walk_state = 1 # Follow Pitch
            self.robot.setup_wait(WaitType.DISTANCE, mov) or self.robot.setup_wait(WaitType.ULTRASONIC_LESS_THAN, 30000)
            self.robot.move(MotorDirection.FORWARD,
                       MotorDirection.FORWARD,
                       1,
                       1,
                       True)
            self.walk_state = 0 # Follow Center All
            if self.follow_loop == False:
                return
            rot = int((random.random()*10000)/8)
            self.walk_state = 2 # Follow Yaw
            self.robot.setup_wait(WaitType.ROTATION, 90000)
            self.robot.move(MotorDirection.FORWARD,
                       MotorDirection.BACKWARD,
                       0.40,
                       0.40,
                       True)
            self.walk_state = 0 # Follow Center All
            if self.follow_loop == False:
                return
    ########### THREADS END ###########
    ###################################
########### CLASS END #################################################################
#######################################################################################

def main():
    print("\n   WELCOME TO DRONE CONTROL    \n\n")

    root = tkinter.Tk()
    bot = roboGUI(parent=root)
    bot.mainloop()
    root.destroy()

# If we are on the main thread, run the program
if __name__ == "__main__":

    try:
        main()
        try:
            LocoRobo.stop()
        except:
            pass
    except KeyboardInterrupt:
        pass
