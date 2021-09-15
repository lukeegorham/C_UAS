import tkinter as tk
from tkinter import *
from unittest import case

from PIL import ImageTk,Image
import threading
import datetime
import pytz
import socket
from time import sleep, time
import struct
import numpy as np # Make sure NumPy is loaded before it is used in the callback
import math
#import pymavlink
import GUI
from threading import Thread, Event
from bitstring import BitArray
import keyboard
import argparse
import imutils
import sys
import cv2

#global variables
acoustic_array = [] #an array of the acoustic class objects
acoustic_target_array = [] #an array of target positions from the Pod Target estimator
true_UAV_pos = [] #an array of the positions of the UAV based on MAVLINK
radar_array_type34 = [] #an array of type 34 asterix messages
radar_Dictionary_type48 = {} #an Dictionary of type 48 asterix messages
podDictionary = {} #Dictionary of Acoustic Pod Objects
trueUAVDictionary = {}
TgtEstimatorDict ={}
radar_Dictionary_type34 = {}
discoveryDroneDict = {} #A Dictionary containing all of the information from the discovery drone

filename= 'LogTest.txt' #log file meant to record data from the dictionaries and flight tests


#c2todrones_array = [] #an array of C2toDrones class objects
#disctoc2_array = [] #an array of DisctoC2 class objects
radar_array = []
# Implementing a circular array to store the radar data
# Size (depth) of the circular array)
logDepth = 20
# Index of the most recent radar data entry
array_new = -1
# Index of the oldest radar data entry
array_old = 0
# Index of the most recent drone data pointer
# This would be the entry of the most recent radar data entry to have a drone detected
drone_new = -1
# Boolean that tells if the array has been filled all the way or not
looped = 0
#mode [0 = standby, 1 = hover at home, 2 = launch to location, 3 = search, 4 = follow-me, 5 = RTB, etc
mode = 2
# Variable that indicates if we have received messages from the various subsystems (1 is yes, -1 is no)
status = 1
#set distance between discovery drone and target in meters
standoffdist_m = 5.0
discoveryIP = 40 #the full ip address is 192.168.1.40

# server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#logfiles
#c2todrones_log = 'c2todrones_' + str(int(datetime.datetime.now(tz=pytz.utc).timestamp())) + '.bin'

syncTime_s = 0.1 #update the screen time every 1 second

class TimerThread(Thread):
    def __init__(self, event):
        Thread.__init__(self)
        self.stopped = event
#
    def run(self):
         while (not self.stopped.is_set()):
            start_server()


class RecvDroneMsgThread(Thread):
    def __init__(self, event):
        Thread.__init__(self)
        self.stopped = event
#
    def run(self):
         while (not self.stopped.is_set()):
            readDiscoverDroneMsg()

class RecvGndCamMsgThread(Thread):
    def __init__(self, event):
        Thread.__init__(self)
        self.stopped = event
#
    def run(self):
         while (not self.stopped.is_set()):
            readGndCamMsg()


class CntlGndCamMsgThread(Thread):
    def __init__(self, event):
        Thread.__init__(self)
        self.stopped = event
#
    def run(self):
         while (not self.stopped.is_set()):
            jogGndCamera()


    # try:
    #     discovery_img_recieve = 4555
    #     disc_img_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #     disc_img_socket.bind(('', discovery_img_recieve))
    #     print("Port Built")
    #
    # except:
    #     print("Port not built")
## Port Information for Discovery Drone Images##
HOST='';
PORT=4555;
jpeg_quality=80

#create socket
try:
    receiver=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
except socket.error:
    print('failed to create socket')
    sys.exit()

# bind to port
try:
    receiver.bind((HOST,PORT))
except socket.error:
    print("no bind port "+str(PORT))
    sys.exit()

## Updated Ground Code
HOSTG='';
PORTG=5566;
jpeg_qualityG=80

#create socket
try:
    receiverG=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
except socket.error:
    print('failed to create socket')
    sys.exit()

# bind to PORTG
try:
    receiverG.bind((HOSTG,PORTG))
except socket.error:
    print("no bind port "+str(PORTG))
    sys.exit()


# ## Port information from ground camera##
# HOST='';
# PORT=5566;
# jpeg_quality=80
#
# #create socket
# try:
#     receiver=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
# except socket.error:
#     print('failed to create socket')
#     sys.exit()
#
# # bind to port
# try:
#     receiver.bind((HOST,PORT))
# except socket.error:
#     print("no bind")
#     sys.exit()


## Begin Code for Camera Control
import cv2, queue, threading, time
global latitude_degC
latitude_degC=39.008942
global longitude_degC
longitude_degC=-104.879922
global elevation_m_MSLC
elevation_m_MSLC=2155.1
global mantoggle1
mantoggle1 = 0
# initialize the ImageSender object with the socket address of the
# server
hostC='192.168.1.255'
portC=46555
#senderC = imagezmq.ImageSender(connect_to='tcp://192.168.1.75:5555')
# create dgram udp socket

try:
    senderC = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    senderC.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    senderC.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
except socket.error:
	print('Failed to create socket')
	sys.exit()

rpiNameC=socket.gethostname()

noNewFileC=True
msgFormatC='<HHdiiiddddddddddd'
MsgHeadrC=int("1e91",16) # randomly chosen for experiment


## Send messages to discovery drone
# bufferSize=1024
# udpUplinkPort=45454
# udpDownlinkPort=50505

hostD='192.168.1.255'
portD=45454
global latitude_deg
latitude_deg=39.008942
global longitude_deg
longitude_deg=-104.879922
global elevation_m_MSL
elevation_m_MSL=2155.1


try:
    senderD = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    senderD.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    senderD.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
except socket.error:
	print('Failed to create socket')
	sys.exit()


def main():
    # Creates a new GUI object and stores appropriate info
    program = GUI.run_main(radar_array, array_old, array_new)
    global mantoggle
    mantoggle = 0
    # Constantly running the code
    global status
    global server_socket
    global disc_down_socket
    global client_socket
    global discovery_uplink_port
    global discovery_img_recieve
    global discovery_images_socket

        # Defines our IP's
        #radar_ip = '192.168.1.61'
    port_num = 55565
    # radar_message_length = 72
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(('', port_num))
    # server_socket.settimeout(1)

    # try:
    # ##discovery drone server
    #     discovery_uplink_port = 45454
    #     client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #     client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    #     print("Broadcast Port Set up")
    #
    # except:
    #     print('Broadcast Port not set up')
    #
    # discovery_downlink_port = 50505
    # disc_down_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # disc_down_socket.bind(('', discovery_downlink_port))

    # try:
    #     discovery_img_recieve = 4555
    #     disc_img_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #     disc_img_socket.bind(('', discovery_img_recieve))
    #     print("Port Built")
    #
    # except:
    #     print("Port not built")

#########################################
    # start threads
    stopFlag = Event()
    syncTimer = TimerThread(stopFlag)
    syncTimer.start()
    
    recvDroneMsg=RecvDroneMsgThread(stopFlag)
    recvDroneMsg.start()

    recvGndCamMsg=RecvGndCamMsgThread(stopFlag)
    recvGndCamMsg.start()
    
    cntlGndCamMsg=CntlGndCamMsgThread(stopFlag)
    cntlGndCamMsg.start()
#########################################
    
    ## Discovery Drone Control
    # try:
    #     print("setting up server")
    #     server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #     # bind to port
    #     print("binding port")
    #     try:
    #         server_socket.bind(('', udpDownlinkPort))
    #     except socket.error:
    #         print("no bind")
    #         sys.exit()
    #     print("set up client")
    #     client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #     client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    #     print("starting server thread")
    #     threading.Thread(target=recvMsg, args=(server_socket,)).start()
    #     lat = 38.9983249
    #     long = -104.6038018
    #     alt = 2250
    #     speed = 5
    #     while True:
    #         returnToLaunch(client_socket)
    #         sleep(5)
    #         goToWaypoint(client_socket, lat, long, alt, speed)
    #         sleep(5)
    #
    # except (KeyboardInterrupt):
    #     raise
    # except:
    #     e = sys.exc_info()[0]
    #     print(e)
    # server_socket.close()
    # client_socket.close()

    while True:
        # Receive and store messages into object classes

        # Update the correct variables in the GUI object
        ##program.update_log(radar_array, array_old, array_new, drone_new, acoustic_array, acoustic_target_array, true_UAV_pos, radar_array_type48, radar_array_type34)
        program.update_log(podDictionary, TgtEstimatorDict, trueUAVDictionary, radar_Dictionary_type48, radar_Dictionary_type34, discoveryDroneDict)
        # Indicate to the GUI that no messages have been received
        if(status==-1):
            program.indicate_no_connection()

        # Updating the GUI display after the variables were passed in
        program.runGUI()
        # # Tkinter method that actually updates the GUI
        program.window.update()
        sleep(0.5)
        #send_c2todrones()
        # try:
        #     if keyboard.is_pressed('r'):
        #         #returnToLaunch(client_socket)
        #         print('pressed')
        #         break
        #     else:
        #         pass
        # except:
        #     break
    ################################################
    # need to initiate these commands from a GUI button
        #goToWaypoint(senderD, latitude_deg, longitude_deg, elevation_m_MSL, 5)
        #returnToLaunch(senderD)
    stopFlag.set()

def readDiscoverDroneMsg():
    ##Image Recieve code from discovery Drone
        try:
        # receive RPi name and frame from the RPi and acknowledge
        # the receipt
            msgIn, (address, port) = receiver.recvfrom(65536)
            MsgHeadr = int("a59f", 16)  # unique message header
            metaFormat = '<HHddddddddHddddhhh'
        # verify correct message
            temp = msgIn[0:2]
            recvHdr = struct.unpack('<H', temp)[0]
            print(MsgHeadr, recvHdr)
            if (recvHdr == MsgHeadr):
            # get metadata size
                temp = msgIn[2:4]
                metaLen = struct.unpack('<H', temp)[0]
            # get metadata
                temp = msgIn[0:metaLen]
                metadata = struct.unpack(metaFormat, temp)
                jpg_buffer = msgIn[metaLen:]  # image is rest of message

                timestamp = metadata[2]
                azimuth = metadata[6]
                print(azimuth)
                frame1 = np.asarray(bytearray(jpg_buffer), dtype="uint8")

                frame1 = cv2.imdecode(frame1, cv2.IMREAD_COLOR)
                frame = imutils.resize(frame1, 640)
                cv2.imshow("frame", frame)
                cv2.imwrite("Picture1.jpg", frame)

        except KeyboardInterrupt:
            cv2.destroyAllWindows()  # not sure if this will work from thread


def readGndCamMsg():
    ## Imagery Recieved from the Ground Camera Code
        try:
                # receive RPi name and frameG from the RPi and acknowledge
                # the receipt
            msgInG, (address, PORTG) = receiverG.recvfrom(65536)
            MsgHeadrG = int("a22e", 16)  # unique message header
            metaFormatG = '<HHddddddddHdddd'
                # verify correct message
            tempG = msgInG[0:2]
            recvHdrG = struct.unpack('<H', tempG)[0]
            print(MsgHeadrG, recvHdrG)
            if (recvHdrG == MsgHeadrG):
                    # get metadataG size
                tempG = msgInG[2:4]
                metaLenG = struct.unpack('<H', tempG)[0]
                    # get metadataG
                tempG = msgInG[0:metaLenG]
                metadataG = struct.unpack(metaFormatG, tempG)
                jpg_bufferG = msgInG[metaLenG:]  # image is rest of message

                timestampG = metadataG[2]
                azimuthG = metadataG[6]
                print(azimuthG)
                frame1G = np.asarray(bytearray(jpg_bufferG), dtype="uint8")

                frame1G = cv2.imdecode(frame1G, cv2.IMREAD_COLOR)
                frameG = imutils.resize(frame1G, 640)
                cv2.imshow("frameG", frameG)

        except KeyboardInterrupt:
            cv2.destroyAllWindows()

def jogGndCamera():
    global mantoggle
    keyG = cv2.waitKey(1) & 0xFF
        # if the `q` keyG was pressed, break from the loop
    # if keyG == ord("q"):
    #     break
    if keyG == ord("w"):
        up = 1
    elif keyG == ord("s"):
        down = 1
    elif keyG == ord("a"):
        left = 1
    elif keyG == ord("d"):
        right = 1
    elif keyG == ord("h"):
        home = 1
    elif keyG == ord("f"):
        stop = 1
    elif keyG == ord("i"):
        zoomIn = 1
    elif keyG == ord("o"):
        zoomOut = 1
    elif keyG == ord("m"):
        if (mantoggle == 0):
            mantoggle = 1
        elif(mantoggle == 1):
            mantoggle = 0

    else:
        up = 0
        down = 0
        left = 0
        right = 0
        home = 0
        stop = 0
        zoomIn = 0
        zoomOut = 0
        #mantoggle = 0

    ## Camera Control
    # create metaDataC
    messageC = [MsgHeadrC]
    messageC.append(np.uint16(112))  # metaDataC size
    messageC.append(datetime.datetime.now(tz=pytz.utc).timestamp())
    messageC.append(int(latitude_degC * 1e7))
    messageC.append(int(longitude_degC * 1e7))
    messageC.append(int(elevation_m_MSLC * 10))
    messageC.append(up)  # Up
    messageC.append(down)  # Down
    messageC.append(left)  # Left
    messageC.append(right)  # Right
    messageC.append(home)  # Home
    messageC.append(zoomIn)  # Zoom_In
    messageC.append(zoomOut)  # Zoom_Out
    messageC.append(stop)  # Stop
    messageC.append(0)  # Pan_Speed
    messageC.append(0)  # Tilt_Speed
    messageC.append(mantoggle)
    metaDataC = struct.pack(msgFormatC, *messageC)

    print(len(metaDataC))
    senderC.sendto(metaDataC, (hostC, portC))




        # try:
        # # receive RPi name and frame from the RPi and acknowledge
        # # the receipt
        #     msgIn, (address, port) = receiver.recvfrom(65536)
        #     MsgHeadr = int("a22e", 16)  # unique message header
        #     metaFormat = '<HHddddddddHdddd'
        #     # verify correct message
        #     temp = msgIn[0:2]
        #     recvHdr = struct.unpack('<H', temp)[0]
        #     print(MsgHeadr, recvHdr)
        #     if (recvHdr == MsgHeadr):
        #     # get metadata size
        #         temp = msgIn[2:4]
        #         metaLen = struct.unpack('<H', temp)[0]
        #     # get metadata
        #         temp = msgIn[0:metaLen]
        #         metadata = struct.unpack(metaFormat, temp)
        #         jpg_buffer = msgIn[metaLen:]  # image is rest of message
        #
        #         timestamp = metadata[2]
        #         azimuth = metadata[6]
        #         print(azimuth)
        #         frame1 = np.asarray(bytearray(jpg_buffer), dtype="uint8")
        #
        #         frame1 = cv2.imdecode(frame1, cv2.IMREAD_COLOR)
        #         frame = imutils.resize(frame1, 640)
        #         cv2.imshow("frame", frame)
        #
        #     key = cv2.waitKey(1) & 0xFF
        # # if the `q` key was pressed, break from the loop
        #     if key == ord("q"):
        #         break
        #     elif key == ord('a'):
        #         keypress = 'a'
        # except KeyboardInterrupt:
        #     cv2.destroyAllWindows()
        #     break



class Pod_Data:
     def __init__(self):
         self.acoustics = []

     def store_acoustics(self, acoustic_data):
         index = acoustic_data.pod_id
         self.acoustics[index] = acoustic_data

#CLASSES OF MESSAGES: See Message structure in the google drive for an explanation of each variable
class Acoustic_Health:
    def __init__(self, msg_id, msg_size, msg_type,pod_id, time_stamp, pod_lat, pod_long, pod_alt, pod_bat):
        self.msg_id = msg_id
        self.msg_size = msg_size
        self.msg_type = msg_type
        self.pod_id = pod_id
        self.time_stamp = time_stamp
        self.pod_lat = pod_lat
        self.pod_long = pod_long
        self.pod_alt = pod_alt
        self.pod_bat = pod_bat

    def update_grid(self, new_x, new_y):
        self.grid_x = new_x
        self.grid_y = new_y

class Acoustic_Target:
    def __init__(self, msg_id, msg_size, msg_type,pod_id, time_stamp, pod_lat, pod_long, pod_alt, tgt_class, tgt_AoA):
        self.msg_id = msg_id
        self.msg_size = msg_size
        self.msg_type = msg_type
        self.pod_id = pod_id
        self.time_stamp = time_stamp
        self.pod_lat = pod_lat
        self.pod_long = pod_long
        self.pod_alt = pod_alt
        self.tgt_class = tgt_class
        self.tgt_AoA = tgt_AoA

class Acoustic_Pod:
    def __init__(self):
        self.time_stamp = 0
        self.pod_lat = 0
        self.pod_long = 0
        self.pod_alt = 0
        self.tgt_class = 0
        self.tgt_AoA = 0
        self.pod_health = False
        self.time_last_message = 0  # used in check to see if pods are still alive
        self.time_last_target_message = 0  # used in check to see if pod should no longer display found target
        self.tgt_active = False  # input to GUI as to whether to display target found

        # This method stores the x and y locations of the acoustic pod on the GUI map
    def update_grid(self, new_x, new_y):
        self.grid_x = new_x
        self.grid_y = new_y

    # This method stores the x and y locations of the acoustic pod on the GUI map
    def update_grid(self, new_x, new_y):
        self.grid_x = new_x
        self.grid_y = new_y


##Target Pod Estimator
class Pod_Target_Estimate: #class to assign the values of the Target Pod Estimator
    def __init__(self, est_msg_id, est_msg_size, est_tstamp, est_tgt_lat, est_tgt_long, est_tgt_alt, pod1_id, pod1_aoa, pod2_id, pod2_aoa, pod3_id, pod3_aoa):
        self.est_msg_id = est_msg_id
        self.est_msg_size = est_msg_size
        self.est_tstamp = est_tstamp
        self.est_tgt_lat= est_tgt_lat
        self.est_tgt_long = est_tgt_long
        self.est_tgt_alt = est_tgt_alt
        self.pod1_id = pod1_id
        self.pod1_aoa = pod1_aoa
        self.pod2_id = pod2_id
        self.pod2_aoa = pod2_aoa
        self.pod3_id = pod3_id
        self.pod3_aoa = pod3_aoa
        self.track_id = 1

    def update_grid(self, acousticx, acousticy):
        self.grid_x = acousticx
        self.grid_y = acousticy

class Pod_Target_Estimate_Track:  # class to assign the values of the Target Pod Estimator
        def __init__(self):
            self.est_tgt_lat = 0
            self.est_tgt_long = 0
            self.est_tgt_alt = 0
            self.pod1_id = 0
            self.pod1_aoa = 0
            self.pod2_id = 0
            self.pod2_aoa = 0
            self.pod3_id = 0
            self.pod3_aoa = 0
            self.time_last_message = 0

        def update_grid(self, acousticx, acousticy):
            self.grid_x_tgt = acousticx
            self.grid_y_tgt = acousticy


##True UAV Classes
class True_UAV:
    def __init__(self, msg_id, msg_size, num_tgts, tgt_index, UAV_num, UAV_timestamp, UAV_lat, UAV_long, altitude):
        self.msg_id = msg_id
        self.msg_size = msg_size
        self.num_tgts = num_tgts
        self.tgt_index = tgt_index
        self.UAV_num = UAV_num
        self.UAV_timestamp = UAV_timestamp
        self.UAV_lat = UAV_lat
        self.UAV_long = UAV_long
        self.altitude = altitude

    def update_true_grid(self, true_x, true_y):
        self.grid_true_x = true_x
        self.grid_true_y = true_y

class True_UAV_Quad:
    def __init__(self):
        self.time_stamp = 0
        self.UAV_lat = 0
        self.UAV_long = 0
        self.UAV_alt = 0
        self.time_last_message = 0  # used in check to see if pods are still alive

        # This method stores the x and y locations of the acoustic pod on the GUI map
    def update_grid(self, new_x, new_y):
        self.grid_x = new_x
        self.grid_y = new_y


##Type 48 Messages
class Radar_Asterix_48:
    def __init__(self, r_serial, radar_lat, radar_long, num_track): #need to add a parameter for the total number of tracks
        self.r_serial = r_serial
        self. radar_lat = radar_lat
        self.radar_long = radar_long
        self.num_track = num_track

    def update_grid(self, new_x, new_y):
        self.grid_x = new_x
        self.grid_y = new_y

class Radar_Asterix_48_Track:
    def __init__(self): #need to add a parameter for the total number of tracks
        self.r_serial = 0
        self. radar_lat = 0
        self.radar_long = 0
        self.num_track = 1

    def update_grid(self, new_x, new_y):
        self.grid_x_48 = new_x
        self.grid_y_48 = new_y

## Type 34 Classes

class Radar_Asterix_34: #class that stores relevant infromation from the Type 34 Asterix message
    def __init__(self, time_day, rot_period, source_alt, source_lat, source_long, track_id): #need to add a parameter for the total number of tracks
        self.time_day = time_day
        self. rot_period = rot_period
        self.source_alt = source_alt
        self.source_lat = source_lat
        self.source_long = source_long
        self.track_id = track_id

    def update_grid(self, new_x, new_y):
        self.grid_x_34 = new_x
        self.grid_y_34 = new_y

class Radar_Asterix_34_Track: #initialization class for the Type 34 dictionary
    def __init__(self):
        self.time_day = 0
        self.rot_period = 0
        self.source_alt = 0
        self.source_lat = 0
        self.source_long = 0

    def update_grid(self, new_x, new_y):
        self.grid_x_34 = new_x
        self.grid_y_34 = new_y

## Discovery Drone information classes
class Discovery_Drone_Information:
    def __init__(self, msg_id, msg_size, timestamp, disc_lat, disc_long, disc_alt, vx, vy, vz, heading, track_id):
        self.msg_id = msg_id
        self.msg_size = msg_size
        self.timestamp = timestamp
        self.disc_lat = disc_lat
        self.disc_long = disc_long
        self.disc_alt = disc_alt
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.heading = heading
        self.track_id = track_id

    def update_true_grid(self, true_x, true_y):
        self.grid_true_x = true_x
        self.grid_true_y = true_y

class Discovery_Drone:
    def __init__(self):
        self.time_stamp = 0
        self.disc_lat = 0
        self.disc_long = 0
        self.disc_alt = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.heading =0
        self.time_last_message = 0  # used in check to see if pods are still alive

        # This method stores the x and y locations of the acoustic pod on the GUI map
    def update_grid(self, new_x, new_y):
        self.grid_x = new_x
        self.grid_y = new_y



#creates the class object and appends the respective array of those class objects
# ie, creates a radar object and adds it to the radar array
def parse(packet, packet_type):
    # Ensuring we can update the global variables in this function
    global array_new, array_old, looped, drone_new
    global radar_array, acoustic_array
    # If the message was from the acoustic pod
    if packet_type =='Simulated Acoustic Health Message':
        new_acoustic = Acoustic_Health(packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6], packet[7],
                                packet[8])
        # test dictionary to see if pod exists
        #print(new_acoustic.pod_id)
        if not new_acoustic.pod_id in podDictionary:
            podDictionary[new_acoustic.pod_id] = Acoustic_Pod()  # add pod to dictionary

        # fill in current information with only items affected by health message
        podDictionary[new_acoustic.pod_id].time_stamp = new_acoustic.time_stamp
        podDictionary[new_acoustic.pod_id].pod_lat = new_acoustic.pod_lat
        podDictionary[new_acoustic.pod_id].pod_long = new_acoustic.pod_long
        podDictionary[new_acoustic.pod_id].pod_alt = new_acoustic.pod_alt
        podDictionary[new_acoustic.pod_id].pod_health = True  # since new health message, pod is healthy
        podDictionary[new_acoustic.pod_id].time_last_message = new_acoustic.time_stamp

        file = open(filename, "w")
        pod_id_string = repr(podDictionary[new_acoustic.pod_id])
        pod_id_lat = repr(podDictionary[new_acoustic.pod_id].time_stamp)

        file.write("Pod ID = " + pod_id_string + "\n")
        file.write("Pod Time = " + pod_id_lat + "\n")
        #file.write(str(podDictionary))
        #file.close()

    elif packet_type == 'Simulated Acoustic Target Message':
         new_acoustic = Acoustic_Target(packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6], packet[7],packet[8], packet[9])
         # test dictionary to see if pod exists

         if not new_acoustic.pod_id in podDictionary:
             podDictionary[new_acoustic.pod_id] = Acoustic_Pod()  # add pod to dictionary

         # fill in current information with only items affected by health message
         podDictionary[new_acoustic.pod_id].time_stamp = new_acoustic.time_stamp
         podDictionary[new_acoustic.pod_id].pod_lat = new_acoustic.pod_lat
         podDictionary[new_acoustic.pod_id].pod_long = new_acoustic.pod_long
         podDictionary[new_acoustic.pod_id].pod_alt = new_acoustic.pod_alt
         podDictionary[new_acoustic.pod_id].pod_health = True  # since new message, pod is healthy
         podDictionary[new_acoustic.pod_id].time_last_message = new_acoustic.time_stamp
         podDictionary[new_acoustic.pod_id].time_last_target_message = datetime.datetime.now(tz=pytz.utc).timestamp()
         podDictionary[new_acoustic.pod_id].tgt_class = new_acoustic.tgt_class
         podDictionary[new_acoustic.pod_id].tgt_AoA = new_acoustic.tgt_AoA
         podDictionary[new_acoustic.pod_id].tgt_active = True  # true since target message

         #print("{} message for pod {}, latitude: {}, longitude: {}".format(packet_type, new_acoustic.pod_id,
                                                                     # podDictionary[new_acoustic.pod_id].pod_lat,
                                                                     # podDictionary[new_acoustic.pod_id].pod_long))

    elif packet_type == 'Pod Target Estimator':
         new_acoustic_target = Pod_Target_Estimate(packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6], packet[7],packet[8], packet[9], packet[10], packet[11])

         if not new_acoustic_target.track_id in TgtEstimatorDict:
            TgtEstimatorDict[new_acoustic_target] = Pod_Target_Estimate_Track()
         #acoustic_target_array.append(new_acoustic_target)
         TgtEstimatorDict[new_acoustic_target].est_tgt_lat = packet[3]
         TgtEstimatorDict[new_acoustic_target].est_tgt_long = packet[4]
         TgtEstimatorDict[new_acoustic_target].est_tgt_alt = packet[5]
         TgtEstimatorDict[new_acoustic_target].pod1_id = packet[6]
         TgtEstimatorDict[new_acoustic_target].pod1_aoa = packet[7]
         TgtEstimatorDict[new_acoustic_target].pod2_id = packet[8]
         TgtEstimatorDict[new_acoustic_target].pod2_aoa = packet[9]
         TgtEstimatorDict[new_acoustic_target].pod3_id = packet[10]
         TgtEstimatorDict[new_acoustic_target].pod3_aoa = packet[11]
         TgtEstimatorDict[new_acoustic_target].time_last_message = datetime.datetime.now(tz=pytz.utc).timestamp()



    elif packet_type == 'True UAV Position':
        new_UAV_position = True_UAV(packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6], packet[7], packet[8])
        if not new_UAV_position.tgt_index in trueUAVDictionary:
            trueUAVDictionary[new_UAV_position.tgt_index] = True_UAV_Quad()

        #fills in the information relevant to the True UAV message
        for t in range(packet[2]):
            trueUAVDictionary[new_UAV_position.tgt_index].timestamp = packet[6*t+5]
            trueUAVDictionary[new_UAV_position.tgt_index].UAV_lat = packet[6*t+6]
            trueUAVDictionary[new_UAV_position.tgt_index].UAV_long = packet[6*t+7]
            trueUAVDictionary[new_UAV_position.tgt_index].UAV_alt = packet[6*t+8]
            trueUAVDictionary[new_UAV_position.tgt_index].time_last_message = packet[6*t+5]
        #true_UAV_pos.append(new_UAV_position)

    elif packet_type == 'Radar Type 48':
        ##Section to delineate between different types of Type 48 messages
        # AsterixLength = [2, 3, 1, 4, 2, 2, 1, 3, 6, 1, 2, 4, 4, 1, 4, 1, 2, 4, 2, 1, 2, 7, 1, 2, 1, 2, 76]
        #
        # count = 0
        # messageEnd = False
        # shift = 0
        # fspec_byte = 3
        # #bits1 = BitArray(8)
        # #one = bits1[0] = 1
        # one = bytes.fromhex('ff')
        # msgsize = 3 #from catagory and message length
        # while messageEnd == False:
        #     byte = radar_data_48[fspec_byte]
        #     b1 = bytes([byte])
        #     #print(b1)
        #     n=7
        #     while n != 0:
        #         if (b1[n] & one[n]) == 1:
        #             #msgsize = msgsize+AsterixLength[shift*8 + count]
        #             count += 1
        #             n -= 1
        #         else:
        #             n-= 1
        #     if (b1array & one) == 1:
        #         fspec_byte += 1
        #         shift += 1
        # print(count)

        #if count == 21

        new_type_48 = Radar_Asterix_48(radar_data_48[38:40], radar_data_48[40:44], radar_data_48[44:48], 1) #need to add byte area for the number of tracks
        #print(radar_data_48[38:40])
        if not new_type_48.num_track in radar_Dictionary_type48:
            radar_Dictionary_type48[new_type_48.num_track] = Radar_Asterix_48_Track()
        r = 0
        # # # for r in range(packet[2]): #need to alter value to where the umber of tracks is located
        radar_Dictionary_type48[new_type_48.num_track].r_serial = radar_data_48[38:40]
        radar_Dictionary_type48[new_type_48.num_track].radar_lat= radar_data_48[40:44]
        radar_Dictionary_type48[new_type_48.num_track].radar_long = radar_data_48[44:48]
        # radar_Dictionary_type48[new_type_48.num_track].r_serial = packet[107*r+38:107 * r + 40]
        # radar_Dictionary_type48[new_type_48.num_track].radar_lat= packet[107*r+40:107*r+44]
        # radar_Dictionary_type48[new_type_48.num_track].radar_long = packet[107*r+44:107*r+48]


    elif packet_type == 'Radar Type 34':
        new_type_34 = Radar_Asterix_34(radar_data_34[9:12], radar_data_34[12:14], radar_data_34[15:17], radar_data_34[17:20], radar_data_34[20:23], 1)

        if not new_type_34.track_id in radar_Dictionary_type34:
            radar_Dictionary_type34[new_type_34.track_id] = Radar_Asterix_34_Track() #add the type 34 message to the dictionary

        radar_Dictionary_type34[new_type_34.track_id].time_day = radar_data_34[9:12]
        radar_Dictionary_type34[new_type_34.track_id].rot_period = radar_data_34[12:14]
        radar_Dictionary_type34[new_type_34.track_id].source_alt = radar_data_34[15:17]
        radar_Dictionary_type34[new_type_34.track_id].source_lat = radar_data_34[17:20]
        radar_Dictionary_type34[new_type_34.track_id].source_long = radar_data_34[20:23]
    #     radar_array_type34.append(radar_data)


    #Populating the Discovery Drone Dictionary
    elif packet_type == 'Discovery Drone Information':
        #track_id is set to 1 because there is only one discovery drone to track. However, this way the dictionary will not continually expand
        new_DiscDrone_info = Discovery_Drone_Information(packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6], packet[7], packet[8], packet[9], 1)
        if not new_DiscDrone_info.track_id in discoveryDroneDict:
            discoveryDroneDict[new_DiscDrone_info.track_id] = Discovery_Drone()

        #fills in the information relevant to the Discovery Drone Messages
        discoveryDroneDict[new_DiscDrone_info.track_id].timestamp = packet[2]
        discoveryDroneDict[new_DiscDrone_info.track_id].disc_lat = packet[3]
        discoveryDroneDict[new_DiscDrone_info.track_id].disc_long = packet[4]
        discoveryDroneDict[new_DiscDrone_info.track_id].disc_alt = packet[5]
        discoveryDroneDict[new_DiscDrone_info.track_id].vx = packet[6]
        discoveryDroneDict[new_DiscDrone_info.track_id].vy = packet[7]
        discoveryDroneDict[new_DiscDrone_info.track_id].vz = packet[8]
        discoveryDroneDict[new_DiscDrone_info.track_id].heading = packet[9]
        discoveryDroneDict[new_DiscDrone_info.track_id].time_last_message = datetime.datetime.now(tz=pytz.utc).timestamp()



def start_server():
    """
    Creates and starts the UDP server to grap all of our data. It binds to a port and continually listens.
    :param shared_memory: the shared memory object across the server
    :return: radar data message
    """
    global status
    # #
    # #     # Defines our IP's
    # #     #radar_ip = '192.168.1.61'
    # port_num = 55565
    # # radar_message_length = 72
    # server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # server_socket.bind(('', port_num))
    # server_socket.settimeout(1)
    #sleep(0.5)
    #print("UDP SERVER: RUNNING")
    try:
        raw_data = 0  # reset to 0 so you know whether you received something new
        raw_data = server_socket.recv(100000)  # capture packet...100000 is the buffer size, or maximum amount of data that can be received
        #print("Server Open")
    except:
        print("Did not parse data")
        status = -1
    if (raw_data != 0):  # if successful capture
        status = 1
        length = len(raw_data)  # define data length
        #print(length)
        if (int(raw_data[0]) == 232) & (int(raw_data[1]) == 161): #this is a simulated pod message, byte 1 = 0xe8, byte 2 = 0xa1
            if int(raw_data[4]) == 0:  #this is a simulated pod health message
                form = '<HHHHddddd'
                data = struct.unpack(form, raw_data)
                parse(data, 'Simulated Acoustic Health Message')
            elif int(raw_data[4]) == 1: #implying that it is a target message
                form = '<HHHHddddHd'
                data = struct.unpack(form, raw_data)
                parse(data, 'Simulated Acoustic Target Message')

            else:
                print('Server Message Ignore')
        elif (int(raw_data[0]) == 48) & (int(raw_data[1]) == 0): #Asterix type 48 message
            #lat_data = raw_data[40:44]
            #form = '>I'
            #lat = struct.unpack(form, lat_data)
            #lat_correct = lat[0]/1e5
            #print(lat_correct)
            global radar_data_48
            radar_data_48 = raw_data
            parse(radar_data_48, 'Radar Type 48')
            #print("Type 48")

        elif (raw_data[0] == 0x22) & (raw_data[1] == 0x00): #Asterix Type 34 Message
            global radar_data_34
            radar_data_34 = raw_data
            parse(radar_data_34, 'Radar Type 34')
            #print("Type 34")

        elif (int(raw_data[0] == 110)) & (int(raw_data[1]) == 39): #pod estimator message
            form = '<HHdddddHdHdHd'
            data = struct.unpack(form, raw_data)
            parse(data, 'Pod Target Estimator')

        elif (int(raw_data[0] == 179)) & (int(raw_data[1]) == 229):
            tgt_byte = raw_data[4:6]
            numTgts = struct.unpack('<H', tgt_byte)
            form = '<3H'
            for t in range(numTgts[0]):
                form = form + '2H4d'
            data = struct.unpack(form, raw_data)
            parse(data, 'True UAV Position')
            #print("True UAV Pos")

        else:
                print('Some other type of message')

def discovery_server():
    try:
        disc_data = 0  # reset to 0 so you know whether you received something new
        disc_data = disc_socket.recv(100000)  # capture packet...100000 is the buffer size, or maximum amount of data that can be received
        print('Server Open')
    except:
        print("Did not parse Discovery Drone data")
    if (disc_data != 0):

        if (disc_data[0] == 0xac) & (disc_data[1] == 0x03):
            form = '2Hd3i3hH'
            data = struct.unpack(form, disc_data)
            parse(data, 'Discovery Drone Information')
        else:
            print("Not Discovery Drone Information")

# def discovery_image_process(self):
#     try:
#         # receive RPi name and frame from the RPi and acknowledge
#         # the receipt
#         msgIn, (address, port) = receiver.recvfrom(65536)
#         MsgHeadr = int("a59f", 16)  # unique message header
#         metaFormat = '<HHddddddddHddddhhh'
#         # verify correct message
#         temp = msgIn[0:2]
#         recvHdr = struct.unpack('<H', temp)[0]
#         print(MsgHeadr, recvHdr)
#         if (recvHdr == MsgHeadr):
#             # get metadata size
#             temp = msgIn[2:4]
#             metaLen = struct.unpack('<H', temp)[0]
#             # get metadata
#             temp = msgIn[0:metaLen]
#             metadata = struct.unpack(metaFormat, temp)
#             jpg_buffer = msgIn[metaLen:]  # image is rest of message
#
#             timestamp = metadata[2]
#             azimuth = metadata[6]
#             print(azimuth)
#             frame1 = np.asarray(bytearray(jpg_buffer), dtype="uint8")
#
#             frame1 = cv2.imdecode(frame1, cv2.IMREAD_COLOR)
#             frame = imutils.resize(frame1, 640)
#             cv2.imshow("frame", frame)
#
#         key = cv2.waitKey(1) & 0xFF
#         # if the `q` key was pressed, break from the loop
#         if key == ord("q"):
#             #break
#         elif key == ord('a'):
#             keypress = 'a'
#     except KeyboardInterrupt:
#         cv2.destroyAllWindows()
#         #break


# def discovery_image_server():
#     try:
#         disc_image_data = 0  # reset to 0 so you know whether you received something new
#         disc_image_data = discovery_images_socket.recv(100000)  # capture packet...100000 is the buffer size, or maximum amount of data that can be received
#         print("Server Open")
#     except:
#         print("Did not parse Discovery Drone data")
#     if (disc_image_data != 0):
#
#         if (disc_image_data[0] == 0xa5) & (disc_image_data[1] == 0x9F):
#             #form = '2Hd3i3hH'
#             #data = struct.unpack(form, disc_data)
#             #parse(data, 'Discovery Drone Information')
#             print('Image')
#         else:
#             print("Not Discovery Drone Images")

 ## Discovery Drone Control
def returnToLaunch(linkup):  # Return to home Discovery Drone functon
    msgHdr=int("ace1",16)
    msgConfirm=int("d478",16)
    msgFormat='<2H'
    message=[msgHdr]
    message.append(msgConfirm)
    linkup.sendto(struct.pack(msgFormat, *message),(hostD,portD))


def goToWaypoint(linkup, lat, long, alt, speed):
    msgHdr = int("abf2", 16)
    msgLen = 26
    msgFormat = '<HHdlllh'
    message = [msgHdr]
    message.append(np.uint16(msgLen))
    message.append(datetime.datetime.now(tz=pytz.utc).timestamp())
    message.append(np.int32(lat * 1e7))
    message.append(np.int32(long * 1e7))
    message.append(np.int32(alt * 1e3))
    if (speed >= 0):
        sp = speed * 100
    else:
        sp = -1
    message.append(np.int16(sp))
    linkup.sendto(struct.pack(msgFormat, *message), (hostD, portD))

# def recvMsg(downlink):
#     msgHdr = int("ac03", 16)
#     msgFormat = '<HHdlllhhhH'
#     while True:
#         try:
#             #            print("waiting for message")
#             msgIn, ipSource = downlink.recvfrom(bufferSize)
#             temp = msgIn[:2]
#             msgId = struct.unpack('<H', temp)[0]
#             if (msgId == msgHdr):
#                 data = struct.unpack(msgFormat, msgIn)
#                 timestamp = data[2]
#                 latitude = data[3] / 1e7
#                 longitude = data[4] / 1e7
#                 altitude = data[5] / 1e3
#                 velX = data[6] / 100
#                 velY = data[7] / 100
#                 velZ = data[8] / 100
#                 heading = data[9] / 100
#                 print(timestamp, latitude, longitude, altitude, velX, velY, velZ, heading)
#         except (KeyboardInterrupt):
#             raise
#         except:
#             e = sys.exc_info()[0]
#             print(e)




        #form = '<' + str(int(length / 8)) + 'd'  # data structure is little endian and in doubles
        #data = struct.unpack(form, raw_data)
        #header = int(data[0])
        #identifies what type of message it is
        #if header == 1:
            #packet_type = "Acoustic"
            #parse(data, packet_type)
        #elif header == 2:
            #packet_type = "Radar"
            #parse(data, packet_type)
        #elif header == 4:
            #packet_type = "DisctoC2"
            #parse(data, packet_type)
        #else:
            #packet_type = "Unknown"

# def send_c2todrones():
#     #open a socket
#     port_num = 11752
#     server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     server_socket.bind(('', port_num))
#     server_socket.settimeout(1)
#     sleep(0.5)
#
#
#     #we never used this, but instead you might just want to send MAVLINK messages directly to the drone instead of this to the on-board raspberry pi
#     #if radar is populated, then we can populate C2toDrones
#     if(len(radar_array) > 0):
#         myradar = radar_array[-1] #get the most recent radar message
#
#         # right now we take the most recent drone detected, because in theory the last drone detected should be the enemy drone
#         # but, you will need to handle multiple enemy drones and assign them to one discovery drone.
#         #you'll also need to distinguish between friendly drones and enemy drones. You can do this by comparing the location
#         #that our drone is sending up to the blip on the radar. If it is the same, ignore that one it's not an enemy.
#         if(radar_array[-1].numTgts > 0):
#             flyalt_mmsl = myradar.drones[-1].range_m*math.sin(np.radians(myradar.drones[-1].elevation_deg)) #takes elevation and range to calculate altitude
#             c2todrones_array.append(C2toDrones(3, discoveryIP, myradar.time_unix_sec, mode, myradar.drones[-1].flylat_deg, myradar.drones[-1].flylong_deg, flyalt_mmsl, 0, 0, 0, standoffdist_m))
#             print(c2todrones_array[-1])
#
#     if(len(c2todrones_array) > 0):
#         myobject = c2todrones_array[-1] #send the last c2todrones object
#         msg = np.zeros(11, dtype='float')
#         msg = (float(myobject.header), float(myobject.src_ip), float(myobject.time_unix_sec), float(myobject.mode), float(myobject.flylat_deg),
#                float(myobject.flylong_deg), float(myobject.flyalt_mmsl), float(myobject.nefvx_ms), float(myobject.nefvy_ms), float(myobject.nefvz_ms),
#                float(myobject.standoffdist_m))
#
#         packed_msg = struct.pack('<11d', *msg)
#         server_socket.sendto(packed_msg, ("192.168.1.40", 11752)) #send the message
#
#         with open(c2todrones_log,'ab') as f: #ab to append
#                 np.array(msg).tofile(f)
#     else:
#         print("There's no object in c2todrones_array")

if __name__ == '__main__':
    main()

