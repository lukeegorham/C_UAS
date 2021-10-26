import tkinter as tk
from tkinter import *
# from unittest import case

# from PIL import ImageTk, Image
# import threading
import datetime
# from datetime import datetime
import pytz
import socket
from time import sleep, time
import struct
import numpy as np  # Make sure NumPy is loaded before it is used in the callback
# import math
# import pymavlink
import GUI_Master
from threading import Thread, Event
# from bitstring import BitArray
# import keyboard
# import argparse
import imutils
import sys
# import cv2
import csv
from pyproj import Proj
# from pandas import DataFrame
import capstone2018_sensor_fusion2 as Sensor

# import LLtoUTM

# # # # # INSERTED FOLLOWING LINES in place of LLtoUTM
# Found code snipped at : http://www.geo.uib.no/polarhovercraft/uploads/Main/LatLongUTMconversion.py

from math import pi, sin, cos, tan, sqrt

# Defense Mapping Agency. 1987b. DMA Technical Report: Supplement to Department of Defense World Geodetic System
# 1984 Technical Report. Part I and II. Washington, DC: Defense Mapping Agency
# def LLtoUTM(int ReferenceEllipsoid, const double Lat, const double Long,
# double &UTMNorthing, double &UTMEasting, char* UTMZone)

_deg2rad = pi / 180.0
_rad2deg = 180.0 / pi

_EquatorialRadius = 2
_eccentricitySquared = 3

_ellipsoid = [
    #  id, Ellipsoid name, Equatorial Radius, square of eccentricity
    # first once is a placeholder only, To allow array indices to match id numbers
    [-1, "Placeholder", 0, 0],
    [1, "Airy", 6377563, 0.00667054],
    [2, "Australian National", 6378160, 0.006694542],
    [3, "Bessel 1841", 6377397, 0.006674372],
    [4, "Bessel 1841 (Nambia] ", 6377484, 0.006674372],
    [5, "Clarke 1866", 6378206, 0.006768658],
    [6, "Clarke 1880", 6378249, 0.006803511],
    [7, "Everest", 6377276, 0.006637847],
    [8, "Fischer 1960 (Mercury] ", 6378166, 0.006693422],
    [9, "Fischer 1968", 6378150, 0.006693422],
    [10, "GRS 1967", 6378160, 0.006694605],
    [11, "GRS 1980", 6378137, 0.00669438],
    [12, "Helmert 1906", 6378200, 0.006693422],
    [13, "Hough", 6378270, 0.00672267],
    [14, "International", 6378388, 0.00672267],
    [15, "Krassovsky", 6378245, 0.006693422],
    [16, "Modified Airy", 6377340, 0.00667054],
    [17, "Modified Everest", 6377304, 0.006637847],
    [18, "Modified Fischer 1960", 6378155, 0.006693422],
    [19, "South American 1969", 6378160, 0.006694542],
    [20, "WGS 60", 6378165, 0.006693422],
    [21, "WGS 66", 6378145, 0.006694542],
    [22, "WGS-72", 6378135, 0.006694318],
    [23, "WGS-84", 6378137, 0.00669438]
]


def LLtoUTM(ReferenceEllipsoid, Lat, Long):
    # converts lat/long to UTM coords.  Equations from USGS Bulletin 1532
    # East Longitudes are positive, West longitudes are negative.
    # North latitudes are positive, South latitudes are negative
    # Lat and Long are in decimal degrees
    # Written by Chuck Gantz- chuck.gantz@globalstar.com

    a = _ellipsoid[ReferenceEllipsoid][_EquatorialRadius]
    eccSquared = _ellipsoid[ReferenceEllipsoid][_eccentricitySquared]
    k0 = 0.9996

    # Make sure the longitude is between -180.00 .. 179.9
    LongTemp = (Long + 180) - int((Long + 180) / 360) * 360 - 180  # -180.00 .. 179.9

    LatRad = Lat * _deg2rad
    LongRad = LongTemp * _deg2rad

    ZoneNumber = int((LongTemp + 180) / 6) + 1

    if Lat >= 56.0 and Lat < 64.0 and LongTemp >= 3.0 and LongTemp < 12.0:
        ZoneNumber = 32

    # Special zones for Svalbard
    if Lat >= 72.0 and Lat < 84.0:
        if LongTemp >= 0.0 and LongTemp < 9.0:
            ZoneNumber = 31
        elif LongTemp >= 9.0 and LongTemp < 21.0:
            ZoneNumber = 33
        elif LongTemp >= 21.0 and LongTemp < 33.0:
            ZoneNumber = 35
        elif LongTemp >= 33.0 and LongTemp < 42.0:
            ZoneNumber = 37

    LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3  # +3 puts origin in middle of zone
    LongOriginRad = LongOrigin * _deg2rad

    # compute the UTM Zone from the latitude and longitude
    UTMZone = "%d%c" % (ZoneNumber, _UTMLetterDesignator(Lat))

    eccPrimeSquared = (eccSquared) / (1 - eccSquared)
    N = a / sqrt(1 - eccSquared * sin(LatRad) * sin(LatRad))
    T = tan(LatRad) * tan(LatRad)
    C = eccPrimeSquared * cos(LatRad) * cos(LatRad)
    A = cos(LatRad) * (LongRad - LongOriginRad)

    M = a * ((1
              - eccSquared / 4
              - 3 * eccSquared * eccSquared / 64
              - 5 * eccSquared * eccSquared * eccSquared / 256) * LatRad
             - (3 * eccSquared / 8
                + 3 * eccSquared * eccSquared / 32
                + 45 * eccSquared * eccSquared * eccSquared / 1024) * sin(2 * LatRad)
             + (15 * eccSquared * eccSquared / 256 + 45 * eccSquared * eccSquared * eccSquared / 1024) * sin(4 * LatRad)
             - (35 * eccSquared * eccSquared * eccSquared / 3072) * sin(6 * LatRad))

    UTMEasting = (k0 * N * (A + (1 - T + C) * A * A * A / 6
                            + (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) * A * A * A * A * A / 120)
                  + 500000.0)

    UTMNorthing = (k0 * (M + N * tan(LatRad) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24
                                                + (61
                                                   - 58 * T
                                                   + T * T
                                                   + 600 * C
                                                   - 330 * eccPrimeSquared) * A * A * A * A * A * A / 720)))

    if Lat < 0:
        UTMNorthing = UTMNorthing + 10000000.0;  # 10000000 meter offset for southern hemisphere
    return (UTMZone, UTMEasting, UTMNorthing)


def _UTMLetterDesignator(Lat):
    # This routine determines the correct UTM letter designator for the given latitude
    # returns 'Z' if latitude is outside the UTM limits of 84N to 80S
    # Written by Chuck Gantz- chuck.gantz@globalstar.com

    if 84 >= Lat >= 72:
        return 'X'
    elif 72 > Lat >= 64:
        return 'W'
    elif 64 > Lat >= 56:
        return 'V'
    elif 56 > Lat >= 48:
        return 'U'
    elif 48 > Lat >= 40:
        return 'T'
    elif 40 > Lat >= 32:
        return 'S'
    elif 32 > Lat >= 24:
        return 'R'
    elif 24 > Lat >= 16:
        return 'Q'
    elif 16 > Lat >= 8:
        return 'P'
    elif 8 > Lat >= 0:
        return 'N'
    elif 0 > Lat >= -8:
        return 'M'
    elif -8 > Lat >= -16:
        return 'L'
    elif -16 > Lat >= -24:
        return 'K'
    elif -24 > Lat >= -32:
        return 'J'
    elif -32 > Lat >= -40:
        return 'H'
    elif -40 > Lat >= -48:
        return 'G'
    elif -48 > Lat >= -56:
        return 'F'
    elif -56 > Lat >= -64:
        return 'E'
    elif -64 > Lat >= -72:
        return 'D'
    elif -72 > Lat >= -80:
        return 'C'
    else:
        return 'Z'  # if the Latitude is outside the UTM limits


# # # # # # END OF INSERTED LINES


# global variables
# latVal, longVal, altVal= GUI.C2GUI().waypoint()
mantoggle = 0
acoustic_array = []  # an array of the acoustic class objects
acoustic_target_array = []  # an array of target positions from the Pod Target estimator
true_UAV_pos = []  # an array of the positions of the UAV based on MAVLINK
radar_array_type34 = []  # an array of type 34 asterix messages
radar_Dictionary_type48 = {}  # an Dictionary of type 48 asterix messages
predictedPositions_Dictionary = {}  # Dictionary containing the sensor fused positions from the radar
DiscoveryDroneOffset_dict = {}  # Dictionary containing the desired Discovery Drone positions (from sensor fusion) with offset enabled.
podDictionary = {}  # Dictionary of Acoustic Pod Objects
trueUAVDictionary = {}
TgtEstimatorDict = {}
radar_Dictionary_type34 = {}
discoveryDroneDict = {}  # A Dictionary containing all of the information from the discovery drone
filter_exists = False
filtersDictionary = {}

LattimeOffset = [None, None, None]
LongtimeOffset = [None, None, None]
AlttimeOffset = [None, None, None]

filename = 'LogTest.txt'  # log file meant to record data from the dictionaries and flight tests

# filenameBase=str(int(datetime.datetime.now(tz=pytz.utc).timestamp())) #filename base with utc seconds
filenameBase = str(
    datetime.datetime.fromtimestamp(datetime.datetime.now(tz=pytz.utc).timestamp()).strftime('%Y-%m-%d %H:%M:%S'))

historicalPositionsSaved = 10  # number of breadcrumbs to be saved

# legacy variables that do not impact on the system functionality. Needs to be removed by changing some file parameters
radar_array = []
array_new = -1
array_old = 0
status = 1

# logfiles
# c2todrones_log = 'c2todrones_' + str(int(datetime.datetime.now(tz=pytz.utc).timestamp())) + '.bin'

global imageCtr
imageCtr = 0

global disc_lat
global disc_long
global disc_alt

global waypoint_test

global RadarUTMx
global RadarUTMy
global correct_alt_34

# global RadarFollow  # variable to turn automatic tracking on and off
# RadarFollow = False
# global filter_exists
# filter_exists = False
# global filter_id
# filter_id = 1
global conversion
global GroundStationUTMx, GroundStationUTMy, GroundStationUTMz
# global filter_Expiration
# filter_Expiration = False
global new_type_48

# global followMeSelection
# followMeSelection = 1

# global acousticToggle
# acousticToggle = 0
# global True_UAV_Index
# True_UAV_Index = 1
syncTime_s = 0.1  # update the screen time every 1 second


# Thread Class that dictates the Situational Awareness Map Refresh
class TimerThread(Thread):
    def __init__(self, event, program):
        Thread.__init__(self)
        self.stopped = event
    #   self.program = program
    def run(self):
        while not self.stopped.wait(syncTime_s):
            start_server()


# Thread that dictates the rate at which messages are received from the Discovery Drone
class RecvDroneMsgThread(Thread):
    def __init__(self, event):
        Thread.__init__(self)
        self.stopped = event

    def run(self):
        while not self.stopped.is_set():
            readDiscoverDroneMsg()


# Thread that dictates the rate at which C2 receives messages from the Ground Camera
class RecvGndCamMsgThread(Thread):
    def __init__(self, event):
        Thread.__init__(self)
        self.stopped = event

    #
    def run(self):
        while not self.stopped.is_set():
            readGndCamMsg()


# Thread that dictates the rate at which C2 sends messages to the Ground Camera
class CntlGndCamMsgThread(Thread):
    def __init__(self, event):
        Thread.__init__(self)
        self.stopped = event

    #
    def run(self):
        while not self.stopped.is_set():
            jogGndCamera()


class CntlDroneMsgThread(Thread):
    def __init__(self, event):
        Thread.__init__(self)
        self.stopped = event

    #
    def run(self):
        while (not self.stopped.is_set()):
            DiscoveryDroneControl()
            sleep(0.1)


## Port Information for Discovery Drone Images ##
HOST = '';
PORT = 44555;
jpeg_quality = 80

# create socket for Discovery Drone Images
try:
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
except socket.error:
    print('failed to create socket')
    sys.exit()

# bind to port
try:
    receiver.bind((HOST, PORT))
except socket.error:
    print("no bind")
    sys.exit()

## Creation of Port for the reception of Ground Camera Messages ##
HOSTG = '';
PORTG = 5566;
jpeg_qualityG = 80

# create socket for ground camera messages
try:
    receiverG = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
except socket.error:
    print('failed to create socket')
    sys.exit()

# bind to PORTG
try:
    receiverG.bind((HOSTG, PORTG))  # Ryan will never find this
except socket.error:
    print("no bind")
    sys.exit()

# ############################################################################################
# Begin Code for Camera Control
# define the global variables that will be used to move the camera based on lattitude, longitude, and altitude calculations
import cv2, queue, threading, time

global latitude_degC
# latitude_degC=39.008942
# latitude_degC = disc_lat
global longitude_degC
# longitude_degC=-104.879922
# longitude_degC = disc_long
global elevation_m_MSLC

# Creation of the port for sending messages from C2 to the Ground Camera
hostC = '192.168.1.255'
portC = 46555  # System defined port to handle communications from C2 to Ground Camera
# senderC = imagezmq.ImageSender(connect_to='tcp://192.168.1.75:5555')
# create dgram udp socket

try:
    senderC = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    senderC.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    senderC.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
except socket.error:
    print('Failed to create socket')
    sys.exit()

try:
    senderD = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    senderD.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    senderD.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
except socket.error:
    print('Failed to create socket')
    sys.exit()

rpiNameC = socket.gethostname()

noNewFileC = True
msgFormatC = '<HHdiiiHHHHHHHHHHHHiiiHHHH'  # defines the message format for the messages sent to the Ground Camera
MsgHeadrC = int("1e91",
                16)  # randomly chosen for experiment, system defined message header for Ground Camera Control messages

# Creation of the port to send messages to the Discovery Drone from C2
hostD = '192.168.1.255'
portD = 45454  # defined in the Message Structure between C2 and Discovery Drone file
global latitude_deg
latitude_deg = 0
# latitude_deg=39.008942
# latitude_input = input("Discover Lat =: ") #command line input for first Discovery Drone waypoint
# latitude_deg =float(latitude_input) #input conversion to decimal
# latitude_deg = float(GUI_Master.lat_val)

# waypoint_test = GUI_Master.C2GUI.waypoint(0, 0)
# print(waypoint_test)


# print(type(latitude_deg))
global longitude_deg
longitude_deg = 0
# longitude_deg=-104.879922

# long_input = input("Discover Long =: ")#command line input for first Discovery Drone waypoint
# longitude_deg =float(long_input) #input conversion to decimal
# longitude_deg = float(GUI_Master.long_val)
global elevation_m_MSL
elevation_m_MSL = 0
# elevation_m_MSL=2155.1
# alt_int = input("Discover Alt =: ")#command line input for first Discovery Drone waypoint
# elevation_m_MSL =float(long_input) #input conversion to decimal
# elevation_m_MSL = float(GUI_Master.alt_val)


# global latitude_deg2
# latitude_deg=39.008942
# latitude_input2 = input("Discover Lat2 =: ") #command line input for second Discovery Drone waypoint
# latitude_deg2 =float(latitude_input2) #input conversion to decimal
latitude_deg2 = 1
# print(type(latitude_deg2))
# global longitude_deg2
# longitude_deg=-104.879922
# long_input2 = input("Discover Long2 =: ")#command line input for second Discovery Drone waypoint
# longitude_deg2 =float(long_input2) #input conversion to decimal
longitude_deg2 = 1
# global elevation_m_MSL2
# elevation_m_MSL=2155.1
# alt_int2 = input("Discover Alt2 =: ")#command line input for second Discovery Drone waypoint
# elevation_m_MSL2 =float(long_input2) #input conversion to decimal
elevation_m_MSL2 = 1

# Section code actually builds the Discovery Drone communication port
try:
    senderD = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    senderD.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    senderD.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
except socket.error:
    print('Failed to create socket')
    sys.exit()

# #################################################################################################
# Discovery Drone Control Section
# #################################################################################################
msgFormatD = '<HHdHHHHddd'  # defines the message format for the messages sent to the Ground Camera
MsgHeadrD = int("a33e",
                16)  # randomly chosen for experiment, system defined message header for Ground Camera Control messages

## Creation of the port to send messages to the Discovery Drone from C2
hostDctrl = '192.168.1.255'
portDctrl = 5666  # defined in the Message Structure between C2 and Discovery Drone file


def main():
    # Creates a new GUI object and stores appropriate info
    program = GUI_Master.run_main(radar_array, array_old, array_new)  # pulling in the GUI file for use with C2
    # latVal, longVal, altVal = program.waypoint() #attempt to bring lat, long, and alt in from GUI code
    GUI_Master.C2GUI.print_hello(5, 5)
    # Constantly running the code
    global status
    global server_socket
    global disc_down_socket
    global client_socket
    global discovery_uplink_port
    global discovery_img_recieve
    global discovery_images_socket

    # construction of port needed to communicated with simulated Radar
    port_num = 55565
    # radar_message_length = 72
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        server_socket.bind(('192.168.1.50', port_num))
    except:
        print("Unable to connect to 192.168.1.50 - connecting to localhost...")
        server_socket.bind(('', port_num))
    # server_socket.settimeout(1)

    # event and thread activations for Radar Simulator Communication
    stopFlag = Event()
    syncTimer = TimerThread(stopFlag, program)
    syncTimer.start()

    # event and thread activations for receiving messages from the Discovery Drone
    recvDroneMsg = RecvDroneMsgThread(stopFlag)
    recvDroneMsg.start()

    # Event and Thread activations for receiving messages from the Ground Camera
    recvGndCamMsg = RecvGndCamMsgThread(stopFlag)
    recvGndCamMsg.start()

    # Event and Thread activations for sending messages to the Ground Camera
    cntlGndCamMsg = CntlGndCamMsgThread(stopFlag)
    cntlGndCamMsg.start()

    cntlDroneMsg = CntlDroneMsgThread(stopFlag)
    cntlDroneMsg.start()

    while True:
        # Receive and store messages into object classes
        global latitude_deg
        global longitude_deg
        global elevation_m_MSL
        if (GUI_Master.lat_val != 0):
            print("Current Latitude Value: ", GUI_Master.lat_val)
        latitude_deg = float(GUI_Master.lat_val)
        longitude_deg = float(GUI_Master.long_val)
        elevation_m_MSL = float(GUI_Master.alt_val)
        RadarFilterPropagation()
        # Update the correct variables in the GUI object
        ##program.update_log(radar_array, array_old, array_new, drone_new, acoustic_array, acoustic_target_array, true_UAV_pos, radar_array_type48, radar_array_type34)
        program.update_log(podDictionary, TgtEstimatorDict, trueUAVDictionary, radar_Dictionary_type48,
                           radar_Dictionary_type34, discoveryDroneDict, trueUAVDictionary, TgtEstimatorDict,
                           predictedPositions_Dictionary,
                           DiscoveryDroneOffset_dict)  # sending the dictionary infromation found below
        # to the GUI for processing and display
        # Indicate to the GUI that no messages have been received; not currently supported, but could be implemented later
        if (status == -1):
            program.indicate_no_connection()

        # Updating the GUI display with new data
        if not program.runGUI():
            break
        program.window.update()
        sleep(0.5)

    stopFlag.set()


def readDiscoverDroneMsg():
    global latitude_deg
    global longitude_deg
    global elevation_m_MSL
    global RadarFollow
    # Image Receive code from discovery Drone
    # try:
    # receive RPi name and frame from the RPi and acknowledge
    # the receipt
    msgIn, (address, port) = receiver.recvfrom(65536)
    MsgHeadr = int("a59f", 16)  # unique message header
    metaFormat = '<HHddddddddHddddhhh'  # message format determined by specification
    # print(len(msgIn))

    # verify correct message
    temp = msgIn[0:2]
    recvHdr = struct.unpack('<H', temp)[0]  # extracting the message header from the recieved message
    # print(MsgHeadr, recvHdr)
    if (recvHdr == MsgHeadr):
        # get metadata size
        temp = msgIn[2:4]
        metaLen = struct.unpack('<H', temp)[0]  # determining the size of the data
        # get metadata
        temp = msgIn[0:metaLen]
        metadata = struct.unpack(metaFormat, temp)
        jpg_buffer = msgIn[metaLen:]  # image is rest of message
        parse(metadata, 'Discovery Drone Information')
        timestamp = metadata[2]
        azimuth = metadata[6]
        # extraction of the dicovery drone lat, long and alt info for use by ground camera and GUI
        global disc_lat
        disc_lat = metadata[3] / 1e7
        # print(disc_lat)
        global disc_long
        disc_long = metadata[4] / 1e7
        global disc_alt
        disc_alt = metadata[5] / 10
        # print(azimuth)

        # Beginning of logic to display the Discovery Drone in an Open CV window
        frame1 = np.asarray(bytearray(jpg_buffer), dtype="uint8")

        frame1 = cv2.imdecode(frame1, cv2.IMREAD_COLOR)
        frame = imutils.resize(frame1, 640)
        cv2.imshow("frame", frame)
        # cv2.imwrite("Picture1.jpg", frame)
        # cv2.imwrite('DiscoveryDrone_' + str(int(datetime.datetime.now(tz=pytz.utc).timestamp())) +
        #             '_image_' + str(imageCtr) + '.jpg', frame)
        # using the open CV message to send a return to launch message to discovery drone by pressing 'r'
        # if (cv2.waitKey(1) & 0xFF == ord('r')):
        # cv2.destroyAllWindows()
        # returnToLaunch(senderD)
        # RadarFollow = False #End automatic tracking by returning the drone to base
        # print(f"rtb_flag = {GUI_Master.rtb_flag}")
        if GUI_Master.rtb_flag == 1:
            print(f"rtb_flag = {GUI_Master.rtb_flag}")
            # cv2.destroyAllWindows()
            returnToLaunch(senderD)
            print("returning to launch")
            RadarFollow = False  # End automatic tracking by returning the drone to base
            GUI_Master.rtb_flag = 0
        # open CV to direct Discovery Drone to Follow Asterix 48 estimates
        if GUI_Master.follow_me_flag == 1:  # Press P on the keyboard to enact automatic tracking
            # cv2.destroyAllWindows()
            RadarFollow = True
            print(RadarFollow)
            GUI_Master.follow_me_flag = 0
        # using the open CV message to send waypoint 1 message to discovery drone by pressing '1'
        # if(cv2.waitKey(1) & 0xFF == ord('1')):
        #    cv2.destroyAllWindows()
        #    goToWaypoint(senderD, latitude_deg, longitude_deg, elevation_m_MSL, 5)
        # using the open CV message to send waypoint 2 message to discovery drone by pressing '2'
        # if(cv2.waitKey(1) & 0xFF == ord('2')):
        #    cv2.destroyAllWindows()
        #    goToWaypoint(senderD, latitude_deg2, longitude_deg2, elevation_m_MSL2, 5)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
    # except clause to break the loop if needed


#       except KeyboardInterrupt:
#           cv2.destroyAllWindows()  # not sure if this will work from thread

# Receiving Imagery and information from the Ground Camera
def readGndCamMsg():
    global imageCtr
    try:
        # receive RPi name and frameG from the RPi and acknowledge
        # the receipt
        msgInG, (address, PORTG) = receiverG.recvfrom(65536)  # receive message from the port
        MsgHeadrG = int("a22e", 16)  # unique message header
        metaFormatG = '<HHddddddddHddddddHHH'  # based on specification
        # verify correct message
        tempG = msgInG[0:2]
        recvHdrG = struct.unpack('<H', tempG)[0]  # extract header
        # print(MsgHeadrG, recvHdrG)
        if recvHdrG == MsgHeadrG:  # confirm header matches expected message type
            # get metadataG size
            tempG = msgInG[2:4]
            metaLenG = struct.unpack('<H', tempG)[0]
            # get metadataG
            tempG = msgInG[0:metaLenG]
            metadataG = struct.unpack(metaFormatG, tempG)
            jpg_bufferG = msgInG[metaLenG:]  # image is rest of message

            timestampG = metadataG[2]
            azimuthG = metadataG[6]
            # print(azimuthG)
            # Display imagery from the message in open Cv
            frame1G = np.asarray(bytearray(jpg_bufferG), dtype="uint8")

            frame1G = cv2.imdecode(frame1G, cv2.IMREAD_COLOR)
            frameG = imutils.resize(frame1G, 640)
            cv2.imshow("frameG", frameG)

            imageCtr += 1

            # cv2.imwrite('ts_' + str(int(datetime.datetime.now(tz=pytz.utc).timestamp())) + '_image_' + str(
            #                    imageCtr) + '.jpg', frameG) press 'g' in order to close the openCV windows
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()

    except KeyboardInterrupt:
        cv2.destroyAllWindows()


# Creating and sending messages to the Ground Camera
def jogGndCamera():
    global mantoggle
    global latitude_deg
    global longitude_deg  # This is me typing during the PA video. I hope you find this later Ryan
    global trueUAVDictionary
    # global True_UAV_Index
    # openCV with display a camera icon that must be clicked on to maneuver the camera
    buster = cv2.imread('cameraicon.jpg')
    busterResize = imutils.resize(buster, 300)
    cv2.imshow("Control", busterResize)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
    keyG = cv2.waitKey(1) & 0xFF
    # if the `q` keyG was pressed, break from the loop
    # if keyG == ord("q"):
    #     break
    up = 0
    down = 0
    left = 0
    right = 0
    home = 0
    stop = 0
    zoomIn = 0
    zoomOut = 0
    # mantoggle = 0
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
    elif GUI_Master.cam_flag == 1:  # toggles control from automatic control to manuel control
        if mantoggle == 0:
            mantoggle = 1
        elif mantoggle == 1:
            mantoggle = 0
        GUI_Master.cam_flag = 0

    # print(mantoggle)
    # Camera Control
    # create metaDataC and form the message to be sent to the camera
    try:
        # if (int(trueUAVDictionary[0] != None)):
        # enemyUAVLat = int(trueUAVDictionary[0].UAV_lat * 1e7)
        # enemyUAVLong = int(trueUAVDictionary[0].UAV_long * 1e7)
        # enemyUAVAlt = int(trueUAVDictionary[0].UAV_alt * 10)
        # else:
        # enemyUAVLat = int(39 * 1e7)
        # enemyUAVLong = int(-140 * 1e7)
        # enemyUAVAlt = int(2160 * 10)
        messageC = [MsgHeadrC, np.uint16(68), datetime.datetime.now(tz=pytz.utc).timestamp()]
        # Three different methods for automatic control: hardcode, user input, information from TrueUAV Dictionary
        # messageC.append(int(39.008429 * 1e7))
        # messageC.append(int(-104.882098 * 1e7))
        # messageC.append(int(2175 * 10))
        # messageC.append(int(latitude_deg * 1e7))
        # messageC.append(int(longitude_deg * 1e7))
        # messageC.append(int(elevation_m_MSL * 10))
        # global target_lat_correct, target_long_correct, target_alt_correct
        if GUI_Master.followme_select == 2:
            messageC.append(int(target_lat_correct * 1e7))
            messageC.append(int(target_long_correct * 1e7))
            messageC.append(int(target_alt_correct * 10))

        if GUI_Master.followme_select == 1:
            messageC.append(int(trueUAVDictionary[True_UAV_Index].UAV_lat * 1e7))
            messageC.append(int(trueUAVDictionary[True_UAV_Index].UAV_long * 1e7))
            messageC.append(int(trueUAVDictionary[True_UAV_Index].UAV_alt * 10))
        # print(int(trueUAVDictionary[0].UAV_long * 1e7))
        # print(enemyUAVLat)
        # messageC.append(enemyUAVLat)
        # messageC.append(enemyUAVLong)
        # messageC.append(enemyUAVAlt)
        if GUI_Master.followme_select == 3:
            messageC.append(int(predictedPositions_Dictionary[1].lat_est * 1e7))
            messageC.append(int(predictedPositions_Dictionary[1].long_est * 1e7))
            messageC.append(int(predictedPositions_Dictionary[1].alt_est * 10))

        # Appending the message with control information
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
        messageC.append(1)
        messageC.append(int(39.009170 * 1e7))  # Ground Station latitude
        messageC.append(int(-104.878982 * 1e7))  # Ground Station Longitude
        messageC.append(int(2163 * 10))  # Ground Station Altitude
        messageC.append(0)
        messageC.append(GUI_Master.gnd_direction)  # Facing of the Ground Camera (Cardinal Dir) (0=N; 1=E; 2=S; 3=W)
        messageC.append(GUI_Master.ground_img_processing)  # Image Processing Toggle
        messageC.append(GUI_Master.ground_pic_cont)
        metaDataC = struct.pack(msgFormatC, *messageC)

        # print(mantoggle)
        # print(metaDataC[7])
        # print(len(metaDataC))
        senderC.sendto(metaDataC, (hostC, portC))  # pass the message to the established port
    except:
        no_cam = 1
        # print("No message to camera")


def DiscoveryDroneControl():
    # print(mantoggle)
    # Camera Control
    # create metaDataC and form the message to be sent to the camera
    # MsgHeadrD = "a33e"
    try:
        messageD = [MsgHeadrD, np.uint16(44), datetime.datetime.now(tz=pytz.utc).timestamp(), 0, GUI_Master.yaw,
                    GUI_Master.drone_img_processing, GUI_Master.drone_pic_cont, 0, 0, 0]
        metaDataD = struct.pack(msgFormatD, *messageD)

        # print(metaDataC[7])
        # print(len(metaDataC))
        senderD.sendto(metaDataD, (hostDctrl, portDctrl))  # pass the message to the established port
    except:
        no_cam = 1
        # print("No message to camera")


class Pod_Data:
    def __init__(self):
        self.acoustics = []

    def store_acoustics(self, acoustic_data):
        index = acoustic_data.pod_id
        self.acoustics[index] = acoustic_data


# CLASSES OF MESSAGES: See Message structure in the google drive for an explanation of each variable
class Acoustic_Health:  # class used to define the Acoustic Health Message Structure from the Simulated Radar
    def __init__(self, msg_id, msg_size, msg_type, pod_id, time_stamp, pod_lat, pod_long, pod_alt, pod_bat):
        self.grid_x = None
        self.grid_y = None
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


class Acoustic_Target:  # Class to define the Acoustic Target Message from the Simulated Radar Pod
    def __init__(self, msg_id, msg_size, msg_type, pod_id, time_stamp, pod_lat, pod_long, pod_alt, tgt_class, tgt_AoA):
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


class Acoustic_Pod:  # Class used to populate the Acoustic Pod Dictionary; combines Health and Target messages
    def __init__(self):
        self.time_stamp = 0
        self.pod_id = 0
        self.pod_lat = 0
        self.pod_long = 0
        self.pod_alt = 0
        self.tgt_class = 0
        self.tgt_AoA = 0
        self.pod_health = False
        self.time_last_message = 0  # used in check to see if pods are still alive
        self.time_last_target_message = 0  # used in check to see if pod should no longer display found target
        self.tgt_active = False  # input to GUI as to whether to display target found
        self.grid_x = None
        self.grid_y = None

    # # This method stores the x and y locations of the acoustic pod on the GUI map
    # def update_grid(self, new_x, new_y):
    #     self.grid_x = new_x
    #     self.grid_y = new_y

    # This method stores the x and y locations of the acoustic pod on the GUI map
    def update_grid(self, new_x, new_y):
        self.grid_x = new_x
        self.grid_y = new_y


# #################################################################################################
# Target Pod Estimator
# #################################################################################################
class Pod_Target_Estimate:  # class to assign the values of the Target Pod Estimator
    grid_x = 0
    grid_y = 0

    def __init__(self, est_msg_id, est_msg_size, est_tstamp, est_tgt_lat, est_tgt_long, est_tgt_alt, pod1_id, pod1_aoa,
                 pod2_id, pod2_aoa, pod3_id, pod3_aoa):
        self.est_msg_id = est_msg_id
        self.est_msg_size = est_msg_size
        self.est_tstamp = est_tstamp
        self.est_tgt_lat = est_tgt_lat
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
    grid_x_tgt = 0
    grid_y_tgt = 0

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
        self.tgt_posBuffer = []

    def update_grid(self, acousticx, acousticy):
        self.grid_x_tgt = acousticx
        self.grid_y_tgt = acousticy


# #################################################################################################
# True UAV Classes
# #################################################################################################
class True_UAV:  # class used to assign different parts of the True_UAV message
    grid_true_x = 0
    grid_true_y = 0

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


class True_UAV_Quad:  # class used to populate the True UAV dictionary
    grid_x = 0
    grid_y = 0

    def __init__(self):
        self.time_stamp = 0
        self.UAV_lat = 0
        self.UAV_long = 0
        self.UAV_alt = 0
        self.time_last_message = 0
        self.posBuffer = []

        # This method stores the x and y locations of the acoustic pod on the GUI map

    def update_grid(self, new_x, new_y):
        self.grid_x = new_x
        self.grid_y = new_y


class True_UAV_Pos_Hist:  # class used to store previous True UAV positions for use by breadcrumps and logging
    def __init__(self, timestamp, lat, long):
        self.timestamp = timestamp
        self.lat = lat
        self.long = long


class Acoustic_Pos_Hist:  # class used to store previous acoustic UAV positions for use by breadcrumps and logging
    def __init__(self, timestamp, lat, long):
        self.timestamp = timestamp
        self.lat = lat
        self.long = long


# #################################################################################################
# Type 48 Messages
# #################################################################################################
class Radar_Asterix_48:  # class to extract select informatin from the ASTERIX 48 message
    grid_x = 0
    grid_y = 0

    def __init__(self, r_serial, radar_lat, radar_long, radar_alt, radar_velx, radar_vely, radar_velz,
                 num_track):  # need to add a parameter for the total number of tracks
        self.r_serial = r_serial
        self.radar_lat = radar_lat
        self.radar_long = radar_long
        self.radar_alt = radar_alt
        self.radar_velx = radar_velx
        self.radar_vely = radar_vely
        self.radar_velz = radar_velz
        self.num_track = num_track

    def update_grid(self, new_x, new_y):
        self.grid_x = new_x
        self.grid_y = new_y


class Radar_Asterix_48_Track:  # class used to populate the ASTERIX 48 Dictionary
    grid_x_48 = 0
    grid_y_48 = 0

    def __init__(self):  # need to add a parameter for the total number of tracks
        self.r_serial = 0
        self.radar_lat = 0
        self.radar_long = 0
        self.radar_alt = 0
        self.radar_velx = 0
        self.radar_vely = 0
        self.radar_velz = 0
        self.num_track = 1
        self.end_of_track = 'False'
        self.last_radar_message = 0

    def update_grid(self, new_x, new_y):
        self.grid_x_48 = new_x
        self.grid_y_48 = new_y


class Sensor_estimate:
    grid_x_sensor = 0
    grid_y_sensor = 0

    def __init__(self):
        self.xpos = 0
        self.ypos = 0
        self.zpos = 0
        self.lat_est = 0
        self.long_est = 0
        self.alt_est = 0
        self.num_track = 1
        self.sensorTime = 0
        self.filterExpiration = False

    def update_grid(self, new_x, new_y):
        self.grid_x_sensor = new_x
        self.grid_y_sensor = new_y


# #################################################################################################
# Type 34 Classes
# #################################################################################################
class Radar_Asterix_34:  # class that stores relevant infromation from the Type 34 Asterix message
    grid_x_34 = 0
    grid_y_34 = 0

    def __init__(self, time_day, rot_period, source_alt, source_lat, source_long,
                 track_id):  # need to add a parameter for the total number of tracks
        self.time_day = time_day
        self.rot_period = rot_period
        self.source_alt = source_alt
        self.source_lat = source_lat
        self.source_long = source_long
        self.track_id = track_id

    def update_grid(self, new_x, new_y):
        self.grid_x_34 = new_x
        self.grid_y_34 = new_y


class Radar_Asterix_34_Track:  # initialization class for the Type 34 dictionary
    grid_x_34 = 0
    grid_y_34 = 0

    def __init__(self):
        self.time_day = 0
        self.rot_period = 0
        self.source_alt = 0
        self.source_lat = 0
        self.source_long = 0

    def update_grid(self, new_x, new_y):
        self.grid_x_34 = new_x
        self.grid_y_34 = new_y


# #################################################################################################
# Discovery Drone information classes
# #################################################################################################
class Discovery_Drone_Information:  # class used to assign metadata from the Discovery Drone for use in the program
    grid_true_x = 0
    grid_true_y = 0

    def __init__(self, msg_id, msg_size, timestamp, disc_lat, disc_long, disc_alt, azimuth, elevation, zoom, pixel_Size,
                 blob_present, blob_size, blob_location, object_classification, class_certainty, vx, vy, vz, track_id):
        self.msg_id = msg_id
        self.msg_size = msg_size
        self.timestamp = timestamp
        self.disc_lat = disc_lat
        self.disc_long = disc_long
        self.disc_alt = disc_alt
        self.azimuth = azimuth
        self.elevation = elevation
        self.zoom = zoom
        self.pixel_Size = pixel_Size
        self.blob_present = blob_present
        self.blob_size = blob_size
        self.blob_location = blob_location
        self.object_classification = object_classification
        self.class_certainty = class_certainty
        self.vx = vx
        self.vy = vy
        self.vz = vz
        # self.reserved1 = reserved1
        # self.reserved2 = reserved2
        self.track_id = track_id

    def update_true_grid(self, true_x, true_y):
        self.grid_true_x = true_x
        self.grid_true_y = true_y


class Discovery_Drone:  # class used to populate the Discovery Drone Metadata Dictionary
    grid_x = 0
    grid_y = 0

    def __init__(self):
        self.time_stamp = 0
        self.disc_lat = 0
        self.disc_long = 0
        self.disc_alt = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.heading = 0
        self.time_last_message = 0  # used in check to see if pods are still alive

        # This method stores the x and y locations of the acoustic pod on the GUI map

    def update_grid(self, new_x, new_y):
        self.grid_x = new_x
        self.grid_y = new_y


class Discovery_Drone_Offset:
    grid_x_est_offset = 0
    grid_y_est_offset = 0

    def __init__(self):
        self.xpos_offset = 0
        self.ypos_offset = 0
        self.zpos_offset = 0
        self.lat_est_offset = 0
        self.long_est_offset = 0
        self.alt_est_offset = 0
        self.num_track = 1
        self.sensorTime = 0

    def update_grid(self, new_x, new_y):
        self.grid_x_est_offset = new_x
        self.grid_y_est_offset = new_y


class Filters:
    def __init__(self):
        self.filter_id = 0
        self.stateMatrix = 0
        self.covarianceMatrix = 0
        self.filterTime = datetime.datetime.now(tz=pytz.utc).timestamp()
        self.timeLastMeasurement = 0


# creates the class object and appends the respective array of those class objects
# ie, creates a radar object and adds it to the radar array
def parse(packet, packet_type):
    # Ensuring we can update the global variables in this function
    global array_new, array_old  # , looped, drone_new  # old variables, not currently supported
    global radar_array, acoustic_array  # old variables, not currently supported

    # Pull in Base Location for the Purpose of establishing Ground Zero
    global GroundStationUTMx, GroundStationUTMy, GroundStationUTMz
    GroundStationUTMx, GroundStationUTMy, GroundStationUTMz = BaseLocation()

    # If the message was from the acoustic pod
    if packet_type == 'Simulated Acoustic Health Message':
        new_acoustic = Acoustic_Health(packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6],
                                       packet[7],
                                       packet[8])
        # test dictionary to see if pod exists
        # print(new_acoustic.pod_id)
        if not new_acoustic.pod_id in podDictionary:
            podDictionary[new_acoustic.pod_id] = Acoustic_Pod()  # add pod to dictionary

        # fill in current information with only items affected by health message
        podDictionary[new_acoustic.pod_id].time_stamp = new_acoustic.time_stamp
        podDictionary[new_acoustic.pod_id].pod_id = new_acoustic.pod_id
        podDictionary[new_acoustic.pod_id].pod_lat = new_acoustic.pod_lat
        podDictionary[new_acoustic.pod_id].pod_long = new_acoustic.pod_long
        podDictionary[new_acoustic.pod_id].pod_alt = new_acoustic.pod_alt
        podDictionary[new_acoustic.pod_id].pod_health = True  # since new health message, pod is healthy
        podDictionary[new_acoustic.pod_id].time_last_message = new_acoustic.time_stamp

        # Begin Simulated Health Message Log
        with open('Simulated Pod Health TST.csv', 'a', newline='') as file:
            fieldnames = ['TIMESTAMP', 'POD ID', 'LATITUDE', 'LONGITUDE', 'ALTITUDE', 'POD HEALTH', 'TIME LAST MESSAGE']
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerow({'TIMESTAMP': datetime.datetime.fromtimestamp(
                int(podDictionary[new_acoustic.pod_id].time_stamp)).strftime('%Y-%m-%d %H:%M:%S'),
                             'POD ID': podDictionary[new_acoustic.pod_id].pod_id,
                             'LATITUDE': podDictionary[new_acoustic.pod_id].pod_lat,
                             'LONGITUDE': podDictionary[new_acoustic.pod_id].pod_long,
                             'ALTITUDE': podDictionary[new_acoustic.pod_id].pod_alt,
                             'POD HEALTH': podDictionary[new_acoustic.pod_id].pod_health,
                             'TIME LAST MESSAGE': podDictionary[new_acoustic.pod_id].time_last_message})

        # #Code used to save Pod information in an excel file
        # file = open(filename, "w")
        # pod_id_string = repr(podDictionary[new_acoustic.pod_id])
        # pod_id_lat = repr(podDictionary[new_acoustic.pod_id].time_stamp)
        #
        # file.write("Pod ID = " + pod_id_string + "\n")
        # file.write("Pod Time = " + pod_id_lat + "\n")
        # #file.write(str(podDictionary))
        # #file.close()

    elif packet_type == 'Simulated Acoustic Target Message':  # parsing of Simulated Target Message and population of Dictionary
        new_acoustic = Acoustic_Target(packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6],
                                       packet[7], packet[8], packet[9])
        # test dictionary to see if pod exists

        if not new_acoustic.pod_id in podDictionary:
            podDictionary[new_acoustic.pod_id] = Acoustic_Pod()  # add pod to dictionary

        # fill in current information with only items affected by health message
        podDictionary[new_acoustic.pod_id].time_stamp = new_acoustic.time_stamp
        podDictionary[new_acoustic.pod_id].pod_id = new_acoustic.pod_id
        podDictionary[new_acoustic.pod_id].pod_lat = new_acoustic.pod_lat
        podDictionary[new_acoustic.pod_id].pod_long = new_acoustic.pod_long
        podDictionary[new_acoustic.pod_id].pod_alt = new_acoustic.pod_alt
        podDictionary[new_acoustic.pod_id].pod_health = True  # since new message, pod is healthy
        podDictionary[new_acoustic.pod_id].time_last_message = new_acoustic.time_stamp
        podDictionary[new_acoustic.pod_id].time_last_target_message = datetime.datetime.now(tz=pytz.utc).timestamp()
        podDictionary[new_acoustic.pod_id].tgt_class = new_acoustic.tgt_class
        podDictionary[new_acoustic.pod_id].tgt_AoA = new_acoustic.tgt_AoA
        podDictionary[new_acoustic.pod_id].tgt_active = True  # true since target message

        # print("{} message for pod {}, latitude: {}, longitude: {}".format(packet_type, new_acoustic.pod_id,
        # podDictionary[new_acoustic.pod_id].pod_lat,
        # podDictionary[new_acoustic.pod_id].pod_long))
        # Begin Simulated Health Message Log
        with open('Simulated Pod Target Messages 2TST.csv', 'a', newline='') as file:
            fieldnames = ['TIMESTAMP', 'POD ID', 'LATITUDE', 'LONGITUDE', 'ALTITUDE', 'POD HEALTH', 'TIME LAST MESSAGE',
                          'TIME LAST TARGET MESSAGE', 'TARGET CLASS', 'TARGET AOA', 'TARGET ACTIVE']
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerow({'TIMESTAMP': datetime.datetime.fromtimestamp(
                int(podDictionary[new_acoustic.pod_id].time_stamp)).strftime('%Y-%m-%d %H:%M:%S'),
                             'POD ID': podDictionary[new_acoustic.pod_id].pod_id,
                             'LATITUDE': podDictionary[new_acoustic.pod_id].pod_lat,
                             'LONGITUDE': podDictionary[new_acoustic.pod_id].pod_long,
                             'ALTITUDE': podDictionary[new_acoustic.pod_id].pod_alt,
                             'POD HEALTH': podDictionary[new_acoustic.pod_id].pod_health,
                             'TIME LAST MESSAGE': podDictionary[new_acoustic.pod_id].time_last_message,
                             'TIME LAST TARGET MESSAGE': podDictionary[new_acoustic.pod_id].time_last_target_message,
                             'TARGET CLASS': podDictionary[new_acoustic.pod_id].tgt_class,
                             'TARGET AOA': podDictionary[new_acoustic.pod_id].tgt_AoA,
                             'TARGET ACTIVE': podDictionary[new_acoustic.pod_id].tgt_active})

    elif packet_type == 'Pod Target Estimator':  # parsing and population of Pod Target Estimator Dictionary
        new_acoustic_target = Pod_Target_Estimate(packet[0], packet[1], packet[2], packet[3], packet[4], packet[5],
                                                  packet[6], packet[7], packet[8], packet[9], packet[10], packet[11])

        if not new_acoustic_target.track_id in TgtEstimatorDict:
            TgtEstimatorDict[new_acoustic_target.track_id] = Pod_Target_Estimate_Track()
        # acoustic_target_array.append(new_acoustic_target)
        TgtEstimatorDict[new_acoustic_target.track_id].est_tgt_lat = packet[3]
        TgtEstimatorDict[new_acoustic_target.track_id].est_tgt_long = packet[4]
        TgtEstimatorDict[new_acoustic_target.track_id].est_tgt_alt = packet[5]
        TgtEstimatorDict[new_acoustic_target.track_id].pod1_id = packet[6]
        TgtEstimatorDict[new_acoustic_target.track_id].pod1_aoa = packet[7]
        TgtEstimatorDict[new_acoustic_target.track_id].pod2_id = packet[8]
        TgtEstimatorDict[new_acoustic_target.track_id].pod2_aoa = packet[9]
        TgtEstimatorDict[new_acoustic_target.track_id].pod3_id = packet[10]
        TgtEstimatorDict[new_acoustic_target.track_id].pod3_aoa = packet[11]
        TgtEstimatorDict[new_acoustic_target.track_id].time_last_message = datetime.datetime.now(
            tz=pytz.utc).timestamp()

        temp_pos = Acoustic_Pos_Hist(TgtEstimatorDict[new_acoustic_target.track_id].time_last_message,
                                     TgtEstimatorDict[new_acoustic_target.track_id].est_tgt_lat,
                                     TgtEstimatorDict[new_acoustic_target.track_id].est_tgt_long)
        if len(TgtEstimatorDict[new_acoustic_target.track_id].tgt_posBuffer) < historicalPositionsSaved:
            TgtEstimatorDict[new_acoustic_target.track_id].tgt_posBuffer.append(temp_pos)
        else:
            temp_buf = TgtEstimatorDict[new_acoustic_target.track_id].tgt_posBuffer[
                       1:]  # contains the first item for the trail
            temp_buf.append(temp_pos)  # adds new position information
            TgtEstimatorDict[
                new_acoustic_target.track_id].tgt_posBuffer = temp_buf  # overwrites buffer and copies over latest value

        # Beginning of Target Pod Estimator log
        with open('PodEstimator2TgtTST.csv', 'a', newline='') as file:
            fieldnames = ['ESTTGTLAT', 'ESTTGTLONG', 'ESTTGTALT', 'PODID', 'POD1AOA', 'PODID', 'POD2AOA', 'PODID',
                          'POD3AOA', 'TIMESTAMP']
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerow({'ESTTGTLAT': TgtEstimatorDict[new_acoustic_target.track_id].est_tgt_lat,
                             'ESTTGTLONG': TgtEstimatorDict[new_acoustic_target.track_id].est_tgt_long,
                             'ESTTGTALT': TgtEstimatorDict[new_acoustic_target.track_id].est_tgt_alt,
                             'PODID': TgtEstimatorDict[new_acoustic_target.track_id].pod1_id,
                             'POD1AOA': TgtEstimatorDict[new_acoustic_target.track_id].pod1_aoa,
                             'PODID': TgtEstimatorDict[new_acoustic_target.track_id].pod2_id,
                             'POD2AOA': TgtEstimatorDict[new_acoustic_target.track_id].pod2_aoa,
                             'PODID': TgtEstimatorDict[new_acoustic_target.track_id].pod3_id,
                             'POD3AOA': TgtEstimatorDict[new_acoustic_target.track_id].pod3_aoa,
                             'TIMESTAMP': datetime.datetime.fromtimestamp(
                                 int(TgtEstimatorDict[new_acoustic_target.track_id].time_last_message)).strftime(
                                 '%Y-%m-%d %H:%M:%S')})

        if GUI_Master.toggle_acoustic == 1:
            # Begin Sensor Fusion Estimate for Acoustic Pods##
            #  Step 1 Unit conversion to UTM followed by setting reference
            try:
                global conversion
                conversion = Proj(proj='utm', zone=13, datum='WGS84')
                AcousticTargetUTMx, AcousticTargetUTMy = conversion(
                    TgtEstimatorDict[new_acoustic_target.track_id].est_tgt_long,
                    TgtEstimatorDict[new_acoustic_target.track_id].est_tgt_lat)
                # print(AcousticTargetUTMx)
                # print(AcousticTargetUTMx)
                if len(TgtEstimatorDict) > 0:
                    global AcousticTgtDistX, AcousticTgtDisty, AcousticTgtAlt
                    # global GroundStationUTMx, GroundStationUTMy, GroundStationUTMz
                    # TgtDistX = float(TargetUTMx) - float(RadarUTMx)
                    # TgtDisty = float(TargetUTMy) - float(RadarUTMy)
                    AcousticTgtDistX = float(AcousticTargetUTMx) - float(GroundStationUTMx)
                    AcousticTgtDisty = float(AcousticTargetUTMy) - float(GroundStationUTMy)
                    AcousticTgtAlt = float(TgtEstimatorDict[new_acoustic_target.track_id].est_tgt_alt) - float(
                        GroundStationUTMz)
                current_time_sensor = datetime.datetime.now(tz=pytz.utc).timestamp()
                acoustic_est = (current_time_sensor, AcousticTgtDistX, AcousticTgtDisty, AcousticTgtAlt, -5, 0, 0)

                global filter_exists
                if not filter_exists:  # Sensor Fusion Update for Acoustic Pod Measurement
                    global filter_id
                    # filter_id = 1 #dictionary index to be altered by sensor fusion
                    if not filter_id in filtersDictionary:
                        filtersDictionary[filter_id] = Filters()

                    global target_filter
                    target_filter = Sensor.generate_new_filter_radar(acoustic_est)
                    filter_exists = True
                else:
                    predicted_state = Sensor.update_with_radar_position_estimate(target_filter, acoustic_est, True)
                    # radar_est = (current_time_sensor, 0, 0, 0, 0, 0, 0)
                    # pulling out the predicted values
                    predictedX = predicted_state.X[0]
                    predictedY = predicted_state.X[1]
                    predictedZ = predicted_state.X[3]

                    # Begin reconversion
                    predictedUTMX = float(predictedX) + GroundStationUTMx
                    predictedUTMY = float(predictedY) + GroundStationUTMy
                    predictedUTMZ = float(predictedZ) + GroundStationUTMz
                    # print(predictedUTMX)

                    # Conversion back to Lat/Long
                    # df = DataFrame(np.c_[x,y], columns=['Meters East', 'Meters South'])
                    predictedLong, predictedLat = conversion(predictedUTMX, predictedUTMY, inverse=True)
                    # print(predictedLat)

                    # Populate the Dictionary that holds the predicted values
                    if not filter_id in predictedPositions_Dictionary:
                        predictedPositions_Dictionary[filter_id] = Sensor_estimate()

                    predictedPositions_Dictionary[filter_id].xpos = predictedUTMX
                    predictedPositions_Dictionary[filter_id].ypos = predictedUTMY
                    predictedPositions_Dictionary[filter_id].zpos = predictedUTMZ
                    predictedPositions_Dictionary[filter_id].long_est = predictedLong
                    predictedPositions_Dictionary[filter_id].lat_est = predictedLat
                    predictedPositions_Dictionary[filter_id].alt_est = predictedUTMZ
                    predictedPositions_Dictionary[filter_id].sensorTime = current_time_sensor

                    # print(predictedPositions_Dictionary[filter_id].lat_est)
                    # sendSensorFusionWaypoint(filter_id)

                    # Populate the filter information for future use
                    filtersDictionary[filter_id].stateMatrix = predicted_state.X
                    filtersDictionary[filter_id].covarianceMatrix = predicted_state.P
                    filtersDictionary[filter_id].filterTime = predicted_state.current_time
                    filtersDictionary[filter_id].timeLastMeasurement = datetime.datetime.now(tz=pytz.utc).timestamp()
                    # print(filtersDictionary[filter_id].stateMatrix)

                    # Time Delay by using a buffer to put Discovery Drone behind Target (Courtesy of C1C Erickson)
                    global LattimeOffset
                    global LongtimeOffset
                    global AlttimeOffset

                    if LattimeOffset[0] is not None:
                        DiscoveryDroneOffsetlat = LattimeOffset[0]
                    LattimeOffset[0] = LattimeOffset[1]
                    LattimeOffset[1] = LattimeOffset[2]
                    LattimeOffset[2] = predictedLat

                    if LongtimeOffset[0] is not None:
                        DiscoveryDroneOffsetlong = LongtimeOffset[0]
                    LongtimeOffset[0] = LongtimeOffset[1]
                    LongtimeOffset[1] = LongtimeOffset[2]
                    LongtimeOffset[2] = predictedLong

                    if AlttimeOffset[0] is not None:
                        DiscoveryDroneOffsetalt = AlttimeOffset[0]
                    AlttimeOffset[0] = AlttimeOffset[1]
                    AlttimeOffset[1] = AlttimeOffset[2]
                    AlttimeOffset[2] = predictedUTMZ

                    # Addition of the Offset with orientation instead of only following (Advanced controls)##
                    # offset = 10 #10 meters of offset added, will eventually be user input
                    # if (predictedLat >= 39.0085455) & (predictedLong <= -104.879958): #top of the screen pos, left
                    #     DiscoveryDroneOffsetlat = predictedLat + (offset/111111) #subtr 10 meter from the predict pos
                    #     DiscoveryDroneOffsetlong = predictedLong + (offset/(111111*np.cos(DiscoveryDroneOffsetlat)))
                    #     DiscoveryDroneOffsetalt = predictedUTMZ + 3
                    # elif (predictedLat >= 39.0085455) & (predictedLong >= -104.879958): #  top right side
                    #     DiscoveryDroneOffsetlat = predictedLat - (offset/111111) #subtr 10 meter from the predict pos
                    #     DiscoveryDroneOffsetlong = predictedLong + (offset/(111111*np.cos(DiscoveryDroneOffsetlat)))
                    #     DiscoveryDroneOffsetalt = predictedUTMZ + 3
                    #
                    # elif (predictedLat <= 39.0085455) & (predictedLong >= -104.879958): #  bottom right side
                    #     DiscoveryDroneOffsetlat = predictedLat - (offset/111111) #subtr 10 meter from the predict pos
                    #     DiscoveryDroneOffsetlong = predictedLong + (offset/(111111*np.cos(DiscoveryDroneOffsetlat)))
                    #     DiscoveryDroneOffsetalt = predictedUTMZ + 3
                    #
                    # else: # bottom left side
                    #     DiscoveryDroneOffsetlat = predictedLat - (offset/111111) #subtr 10 meter from the pred pos
                    #     DiscoveryDroneOffsetlong = predictedLong + (offset/(111111*np.cos(DiscoveryDroneOffsetlat)))
                    #     DiscoveryDroneOffsetalt = predictedUTMZ + 3
                    if (LattimeOffset[0] is not None) & (LongtimeOffset[0] is not None) & (
                            AlttimeOffset[0] is not None):
                        if not filter_id in DiscoveryDroneOffset_dict:
                            DiscoveryDroneOffset_dict[filter_id] = Discovery_Drone_Offset()

                        DiscoveryDroneOffset_dict[filter_id].xpos_offset = predictedUTMX - 10
                        DiscoveryDroneOffset_dict[filter_id].ypos_offset = predictedUTMY - np.cos(10)
                        DiscoveryDroneOffset_dict[filter_id].zpos_offset = predictedUTMZ + 3
                        DiscoveryDroneOffset_dict[filter_id].long_est_offset = DiscoveryDroneOffsetlong
                        DiscoveryDroneOffset_dict[filter_id].lat_est_offset = DiscoveryDroneOffsetlat
                        DiscoveryDroneOffset_dict[filter_id].alt_est_offset = DiscoveryDroneOffsetalt
                        DiscoveryDroneOffset_dict[filter_id].sensorTime = datetime.datetime.now(tz=pytz.utc).timestamp()

                        # Predicted Discovery Drone Position
                        with open('Expected_Discovery_Drone_Position_TST.csv', 'a', newline='') as file:
                            fieldnames = ['TIMESTAMP', 'LATITUDE', 'LONGITUDE', 'ALTITUDE']
                            writer = csv.DictWriter(file, fieldnames=fieldnames)
                            writer.writeheader()
                            writer.writerow({'TIMESTAMP': datetime.datetime.fromtimestamp(
                                int(DiscoveryDroneOffset_dict[filter_id.num_track].sensorTime)).strftime(
                                '%Y-%m-%d %H:%M:%S'),
                                'LATITUDE': DiscoveryDroneOffset_dict[filter_id.num_track].lat_est_offset,
                                'LONGITUDE': DiscoveryDroneOffset_dict[filter_id.num_track].long_est_offset,
                                'ALTITUDE': DiscoveryDroneOffset_dict[filter_id.num_track].alt_est_offset})
            except:
                print("No Acoustic Sensor Fusion Measurement")

        # else: #Else statement and associated print can be used for
        # print("Acoustic Sensor Fusion Turned Off")

        # sendSensorFusionWaypoint()

    elif packet_type == 'True UAV Position':
        for i in range(packet[2]):
            # new_UAV_position = True_UAV(packet[0], packet[1], packet[2], packet[3], packet[4], packet[5],
            #                             packet[6], packet[7], packet[8])
            new_UAV_position = True_UAV(packet[0], packet[1], packet[2], packet[6 * i + 3], packet[6 * i + 4],
                                        packet[6 * i + 5], packet[6 * i + 6], packet[6 * i + 7], packet[6 * i + 8])
            if not new_UAV_position.tgt_index in trueUAVDictionary:
                print(new_UAV_position.tgt_index)
                trueUAVDictionary[new_UAV_position.tgt_index] = True_UAV_Quad()

            # fills in the information relevant to the True UAV message
            # for t in range(packet[2]):
            trueUAVDictionary[new_UAV_position.tgt_index].timestamp = packet[6 * i + 5]
            print(new_UAV_position.tgt_index)
            trueUAVDictionary[new_UAV_position.tgt_index].UAV_lat = packet[6 * i + 6]
            trueUAVDictionary[new_UAV_position.tgt_index].UAV_long = packet[6 * i + 7]
            trueUAVDictionary[new_UAV_position.tgt_index].UAV_alt = packet[6 * i + 8]
            trueUAVDictionary[new_UAV_position.tgt_index].time_last_message = packet[6 * i + 5]
            temp_pos = True_UAV_Pos_Hist(trueUAVDictionary[new_UAV_position.tgt_index].timestamp,
                                         trueUAVDictionary[new_UAV_position.tgt_index].UAV_lat,
                                         trueUAVDictionary[new_UAV_position.tgt_index].UAV_long)
            if len(trueUAVDictionary[new_UAV_position.tgt_index].posBuffer) < historicalPositionsSaved:
                trueUAVDictionary[new_UAV_position.tgt_index].posBuffer.append(temp_pos)
            else:
                temp_buf = trueUAVDictionary[new_UAV_position.tgt_index].posBuffer[
                           1:]  # contains the first item for the trail
                temp_buf.append(temp_pos)  # adds new position information
                trueUAVDictionary[
                    new_UAV_position.tgt_index].posBuffer = temp_buf  # overwrites buffer and copies over latest value
            # # conversion from computer timestamp to readable format
            # true_UAV_pos.append(new_UAV_position) print(datetime.datetime.fromtimestamp(int(trueUAVDictionary[
            #   new_UAV_position.tgt_index].timestamp)).strftime('%Y-%m-%d %H:%M:%S'))

            # Beginning of True UAV log
            with open('TrueUAVInfo2TSTNew.csv', 'a', newline='') as file:
                fieldnames = ['UAVID', 'TIMESTAMP', 'LATITUDE', 'LONGITUDE']
                writer = csv.DictWriter(file, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerow({'UAVID': new_UAV_position.tgt_index, 'TIMESTAMP': datetime.datetime.fromtimestamp(
                    int(trueUAVDictionary[new_UAV_position.tgt_index].timestamp)).strftime('%Y-%m-%d %H:%M:%S'),
                                 'LATITUDE': trueUAVDictionary[new_UAV_position.tgt_index].UAV_lat,
                                 'LONGITUDE': trueUAVDictionary[new_UAV_position.tgt_index].UAV_long})
        # for i in range (len(trueUAVDictionary)):

    elif packet_type == 'Radar Type 34':  # parsing of ASTERIX 34 message and population of Dict with select information
        new_type_34 = Radar_Asterix_34(radar_data_34[9:12], radar_data_34[12:14], radar_data_34[15:17],
                                       radar_data_34[17:20], radar_data_34[20:23], 1)

        if not new_type_34.track_id in radar_Dictionary_type34:
            radar_Dictionary_type34[
                new_type_34.track_id] = Radar_Asterix_34_Track()  # add the type 34 message to the dictionary

        radar_Dictionary_type34[new_type_34.track_id].time_day = radar_data_34[9:12]
        radar_Dictionary_type34[new_type_34.track_id].rot_period = radar_data_34[12:14]
        radar_Dictionary_type34[new_type_34.track_id].source_alt = radar_data_34[15:17]
        radar_Dictionary_type34[new_type_34.track_id].source_lat = radar_data_34[17:20]
        radar_Dictionary_type34[new_type_34.track_id].source_long = radar_data_34[20:23]
        #     radar_array_type34.append(radar_data)

        # Beginning of Asterix 34 Message Log
        with open('Asterix34_TST.csv', 'a', newline='') as file:
            try:
                fieldnames = ['TIME DAY', 'ROTATION PERIOD', 'SOURCE ALTITUDE', 'SOURCE LATITUDE', 'SOURCE LONGITUDE']
                long_34 = int.from_bytes(radar_Dictionary_type34[new_type_34.track_id].source_long, "big",
                                         signed=False)  # Longitude Unpack
                correct_long_34 = (long_34 * (180 / (2 ** 23))) - 360
                lat_34 = int.from_bytes(radar_Dictionary_type34[new_type_34.track_id].source_lat, "big",
                                        signed=False)  # Latitude Unpack
                correct_lat_34 = lat_34 * (180 / (2 ** 23))
                alt_type = '>h'  # the altitude position is reported as a 16 bit short

                alt_34 = struct.unpack(alt_type, radar_Dictionary_type34[
                    new_type_34.track_id].source_alt)  # unpacking the altitude of the radar
                global correct_alt_34
                correct_alt_34 = alt_34[0]
                writer = csv.DictWriter(file, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerow({'TIME DAY': radar_Dictionary_type34[new_type_34.track_id].time_day,
                                 'ROTATION PERIOD': radar_Dictionary_type34[new_type_34.track_id].rot_period,
                                 'SOURCE ALTITUDE': radar_Dictionary_type34[new_type_34.track_id].source_alt,
                                 'SOURCE LATITUDE': correct_lat_34,
                                 'SOURCE LONGITUDE': correct_long_34})
            except:
                print("Can not log Radar Position Data")

        # Lat/Long to XYZ Conversion

        # Using 2020s LLto UTM does not provide the proper values compared to the Matlab value
        # UTM_letter = LLtoUTM.UTMLetterDesignator(correct_lat_34)
        # UTM = LLtoUTM.LLtoUTM_Degrees(correct_lat_34, correct_long_34)
        # print(UTM.UTMNorthing)
        # print(UTM.UTMEasting)
        # print(str(UTM.UTMZoneNumber))
        # print(UTM.UTMZoneLatDes)

        # using pyproj library for cartographers in order to convert to UTM. Verified by Matlab program deg2utm.m
        # global conversion
        conversion = Proj(proj='utm', zone=13, datum='WGS84')
        global RadarUTMx, RadarUTMy
        RadarUTMx, RadarUTMy = conversion(correct_long_34, correct_lat_34)

        # print(RadarUTMx)
        # print(RadarUTMy)

    elif packet_type == 'Radar Type 48':
        # # ##Section to delineate between different types of Type 48 messages
        AsterixLength = [2, 3, 1, 4, 2, 2, 1, 0, 3, 6, 1, 2, 4, 4, 1, 0, 4, 1, 2, 4, 2, 1, 2, 0, 7, 1, 2, 1, 2, 76, 1,
                         0]

        count = 0
        messageEnd = False
        shift = 0
        fspec_byte = 3
        # bits1 = BitArray(8)
        # one = bits1[0] = 1
        # one = bytes.fromhex('ff')
        one_num = int('ff', 16)  # convert hex 'ff' to decimal
        one = str(bin(one_num))  # make a string of '1s'
        one_map = [char for char in one]  # separate characters in a string
        one_str = one_map[2:10]  # remove '0b from beginning of string
        ones = [int(s) for s in one_str]

        msgsize = 3  # from catagory and message length
        while not messageEnd:
            byte = int(radar_data_48[fspec_byte])
            # b1 = bin[byte]
            b1 = str(bin(byte))  # convert bits to a string
            b_map = [char for char in b1]  # separate characters in a string
            bits_str = b_map[2:10]  # remove '0b from beginning of string
            bits = [int(s) for s in bits_str]
            if len(bits) < 8:  # check to make sure that the bit string actually contains 8 values
                while 8 - len(bits) > 0:
                    bits.insert(0, 0)  # appends a zero to the beginning of byte list
            # print(b1)
            n = 0
            while n != 8:
                if (bits[n] & ones[n]) == 1:
                    count += 1
                    n += 1
                    msgsize = msgsize + AsterixLength[shift * 8 + count - 1]
                    # print(msgsize)
                else:
                    n += 1
                    count += 1  # the count needs to increase whether ands successful or not
                    # this is because a count that only increments on successful
                    # ANDs will not account for logic that returns a zero
            # messageEnd = True # for debugging
            if (bits[7] & ones[7]) == 1:  # indicating an extension of the bit field by the
                fspec_byte += 1
                shift += 1
                # print(count)
                count = 0
            else:
                messageEnd = True
        # print(count)
        print(msgsize)

        if msgsize == 15:
            asterix_type = 'End Track Plot'
        else:
            asterix_type = 'Send Track'

        # Parsing of ASTERIX 48 messages and populating Dictionary with select information
        if asterix_type == 'Send Track':
            # if len(radar_data_48) < 150:
            global new_type_48
            new_type_48 = Radar_Asterix_48(radar_data_48[38:40], radar_data_48[40:44], radar_data_48[44:48],
                                           radar_data_48[76:80], radar_data_48[70:72], radar_data_48[72:74],
                                           radar_data_48[74:76], 1)  # need to add byte area for the number of tracks
            # if len(radar_data_48) > 200:
            #     second_new_type_48 = new_type_48 = Radar_Asterix_48(radar_data_48[38:40], radar_data_48[147:151],
            #        radar_data_48[151:155], radar_data_48[183:185], radar_data_48[185:187], radar_data_48[187:189],
            #        radar_data_48[189:191], 2)
            #     if not second_new_type_48.num_track in radar_Dictionary_type48:
            #         radar_Dictionary_type48[second_new_type_48.num_track] = Radar_Asterix_48_Track()
            # print(radar_data_48[38:40])
            if not new_type_48.num_track in radar_Dictionary_type48:
                radar_Dictionary_type48[new_type_48.num_track] = Radar_Asterix_48_Track()

            # if asterix_type = 'End Track Plot': #determined above by Asterix message analysis
            # plot_status = True #end of track; will turn off in GUI
            # else:
            # plot_status = False #beginning of track will turn on in GUI

            # # # for r in range(packet[2]): #need to alter value to where the umber of tracks is located
            radar_Dictionary_type48[new_type_48.num_track].r_serial = radar_data_48[38:40]
            radar_Dictionary_type48[new_type_48.num_track].radar_lat = radar_data_48[40:44]
            radar_Dictionary_type48[new_type_48.num_track].radar_long = radar_data_48[44:48]
            radar_Dictionary_type48[new_type_48.num_track].radar_alt = radar_data_48[76:80]
            radar_Dictionary_type48[new_type_48.num_track].radar_velx = radar_data_48[70:72]
            radar_Dictionary_type48[new_type_48.num_track].radar_vely = radar_data_48[72:74]
            radar_Dictionary_type48[new_type_48.num_track].radar_velz = radar_data_48[74:76]
            radar_Dictionary_type48[new_type_48.num_track].end_of_track = False
            radar_Dictionary_type48[new_type_48.num_track].last_radar_message = datetime.datetime.now(
                tz=pytz.utc).timestamp()  # for the purpose of testing, will hopefully be removed

            # radar_Dictionary_type48[new_type_48.num_track].end_of_track = plot_status

            # Second Target Radar Information
            if len(radar_data_48) > 200:
                new_type_48 = new_type_48 = Radar_Asterix_48(radar_data_48[145:147], radar_data_48[147:151],
                                                             radar_data_48[151:155], radar_data_48[183:185],
                                                             radar_data_48[185:187], radar_data_48[187:189],
                                                             radar_data_48[189:191], 2)
                if not new_type_48.num_track in radar_Dictionary_type48:
                    radar_Dictionary_type48[new_type_48.num_track] = Radar_Asterix_48_Track()

                radar_Dictionary_type48[new_type_48.num_track].r_serial = radar_data_48[145:147]
                radar_Dictionary_type48[new_type_48.num_track].radar_lat = radar_data_48[147:151]
                radar_Dictionary_type48[new_type_48.num_track].radar_long = radar_data_48[151:155]
                radar_Dictionary_type48[new_type_48.num_track].radar_alt = radar_data_48[183:187]
                radar_Dictionary_type48[new_type_48.num_track].radar_velx = radar_data_48[187:189]
                radar_Dictionary_type48[new_type_48.num_track].radar_vely = radar_data_48[189:191]
                radar_Dictionary_type48[new_type_48.num_track].radar_velz = radar_data_48[191:193]
                radar_Dictionary_type48[new_type_48.num_track].end_of_track = False
                radar_Dictionary_type48[new_type_48.num_track].last_radar_message = datetime.datetime.now(
                    tz=pytz.utc).timestamp()  # for the purpose of testing, will hopefully be removed

            # radar_Dictionary_type48[new_type_48.num_track].r_serial = packet[107*r+38:107 * r + 40]
            # radar_Dictionary_type48[new_type_48.num_track].radar_lat= packet[107*r+40:107*r+44]
            # radar_Dictionary_type48[new_type_48.num_track].radar_long = packet[107*r+44:107*r+48]

            # Beginning of Asterix 48 Message Log
            with open('Asterix48_TST.csv', 'a', newline='') as file:
                try:
                    fieldnames = ['TARGET ID', 'TARGET LATITUDE', 'TARGET LONGITUDE', 'TARGET ALTITUDE',
                                  'TARGET X VELOCITY', 'TARGET Y VELOCITY', 'TARGET Z VELOCITY']
                    form = '>i'  # signed 32 byte integer
                    for rr in radar_Dictionary_type48:
                        global target_long_correct, target_lat_correct, target_alt_correct
                        target_longitude = struct.unpack(form, radar_Dictionary_type48[
                            rr].radar_long)  # unpacks the bytes into an integer value
                        target_long_correct = target_longitude[0] / (1e5)
                        # print(long_correct)
                        target_latitude = struct.unpack(form, radar_Dictionary_type48[
                            rr].radar_lat)  # unpacks the bytes into an integer value
                        target_lat_correct = target_latitude[0] / (1e5)
                        target_altitude = struct.unpack(form, radar_Dictionary_type48[rr].radar_alt)
                        target_alt_correct = target_altitude[0]

                        velocity_form = '>h'
                        target_xvelocity = struct.unpack(velocity_form, radar_Dictionary_type48[rr].radar_velx)
                        target_xvel_correct = target_xvelocity[0]
                        target_yvelocity = struct.unpack(velocity_form, radar_Dictionary_type48[rr].radar_vely)
                        target_yvel_correct = target_yvelocity[0]
                        target_zvelocity = struct.unpack(velocity_form, radar_Dictionary_type48[rr].radar_velz)
                        target_zvel_correct = target_zvelocity[0]
                        # print(target_zvel_correct)

                        # Data Association to determine which radar measurement is the Discovery Drone and which is not
                        # if (target_lat_correct - discoveryDroneDict[0].disc_lat/1e7 > 5/111111):
                        # #if the measured latitude is greater than 5m from the Discovery Drone location this is target
                        # if (target_lat_correct - discoveryDroneDict[0].disc_lat / 1e7 > 5 / 111111):
                        sensorMeasLat = target_lat_correct
                        sensorMeasLong = target_long_correct
                        sensorMeasAlt = target_alt_correct

                        writer = csv.DictWriter(file, fieldnames=fieldnames)
                        writer.writeheader()
                        writer.writerow({'TARGET ID': rr, 'TARGET LATITUDE': target_lat_correct,
                                         'TARGET LONGITUDE': target_long_correct, 'TARGET ALTITUDE': target_alt_correct,
                                         'TARGET X VELOCITY': target_xvel_correct,
                                         'TARGET Y VELOCITY': target_yvel_correct,
                                         'TARGET Z VELOCITY': target_zvel_correct})
                except:
                    print('Can not Log Dictionary Data')

                    # Conversion of Lat/Long to UTM for Track Messages

            # Data Association to determine which track is the Dsicovery Drone and which is not
            try:
                conversion = Proj(proj='utm', zone=13, datum='WGS84')
                # TargetUTMx, TargetUTMy = conversion(target_long_correct, target_lat_correct)
                TargetUTMx, TargetUTMy = conversion(sensorMeasLong, sensorMeasLat)
                # print(TargetUTMx)
                # print(TargetUTMx)
                if len(radar_Dictionary_type34) > 0:
                    global TgtDistX, TgtDisty, TgtAlt
                    # TgtDistX = float(TargetUTMx) - float(RadarUTMx)
                    # TgtDisty = float(TargetUTMy) - float(RadarUTMy)
                    TgtDistX = float(TargetUTMx) - float(GroundStationUTMx)
                    TgtDisty = float(TargetUTMy) - float(GroundStationUTMy)
                    # TgtAlt = float(target_alt_correct) - float(GroundStationUTMz)
                    TgtAlt = float(sensorMeasAlt) - float(GroundStationUTMz)
                current_time_sensor = datetime.datetime.now(tz=pytz.utc).timestamp()
                radar_est = (current_time_sensor, TgtDistX, TgtDisty, TgtAlt, target_xvel_correct, target_yvel_correct,
                             target_zvel_correct)
                # print(radar_est[0])

                # #################################################################################
                # # Radar Sensor Fusion # #
                # global filter_exists
                # global filter_id
                if filter_exists is False:
                    # global target_filter
                    target_filter = Sensor.generate_new_filter_radar(radar_est)
                    filter_exists = True
                else:
                    predicted_state = Sensor.update_with_radar_position_estimate(target_filter, radar_est, True)
                    # radar_est = (current_time_sensor, 0, 0, 0, 0, 0, 0)
                    # pulling out the predicted values
                    predictedX = predicted_state.X[0]
                    predictedY = predicted_state.X[1]
                    predictedZ = predicted_state.X[3]

                    # Begin reconversion
                    predictedUTMX = float(predictedX) + GroundStationUTMx
                    predictedUTMY = float(predictedY) + GroundStationUTMy
                    predictedUTMZ = float(predictedZ) + GroundStationUTMz
                    # print(predictedUTMX)

                    # Conversion back to Lat/Long
                    # df = DataFrame(np.c_[x,y], columns=['Meters East', 'Meters South'])
                    predictedLong, predictedLat = conversion(predictedUTMX, predictedUTMY, inverse=True)
                    # print(predictedLat)

                    # Populate the Dictionary that holds the predicted values
                    if not filter_id in predictedPositions_Dictionary:
                        predictedPositions_Dictionary[filter_id] = Sensor_estimate()

                    predictedPositions_Dictionary[filter_id].xpos = predictedUTMX
                    predictedPositions_Dictionary[filter_id].ypos = predictedUTMY
                    predictedPositions_Dictionary[filter_id].zpos = predictedUTMZ
                    predictedPositions_Dictionary[filter_id].long_est = predictedLong
                    predictedPositions_Dictionary[filter_id].lat_est = predictedLat
                    predictedPositions_Dictionary[filter_id].alt_est = predictedUTMZ
                    predictedPositions_Dictionary[filter_id].sensorTime = current_time_sensor
                    # print(predictedRadarPositions_Dictionary[new_type_48.num_track].long_est)
                    # print(TgtAlt)
                    sendSensorFusionWaypoint(filter_id)

                    # Populate the filter information for future use
                    filtersDictionary[filter_id].stateMatrix = predicted_state.X
                    filtersDictionary[filter_id].covarianceMatrix = predicted_state.P
                    filtersDictionary[filter_id].filterTime = predicted_state.current_time
                    filtersDictionary[filter_id].timeLastMeasurement = datetime.datetime.now(tz=pytz.utc).timestamp()
                    # print(filtersDictionary[filter_id].stateMatrix)

                    # Predicted Sensor Fusion Log after Radar Update
                    with open('SensorFusion_12April.csv', 'a', newline='') as file:
                        fieldnames = ['TIMESTAMP', 'LATITUDE', 'LONGITUDE', 'ALTITUDE']
                        writer = csv.DictWriter(file, fieldnames=fieldnames)
                        writer.writeheader()
                        writer.writerow({'TIMESTAMP': datetime.datetime.fromtimestamp(
                            int(predictedPositions_Dictionary[filter_id].sensorTime)).strftime('%Y-%m-%d %H:%M:%S'),
                                         'LATITUDE': predictedPositions_Dictionary[filter_id].lat_est,
                                         'LONGITUDE': predictedPositions_Dictionary[filter_id].long_est,
                                         'ALTITUDE': predictedPositions_Dictionary[filter_id].alt_est})

                    # Time Delay by using a buffer to put Discovery Drone behind Target (Courtesy of C1C Erickson)
                    # global LattimeOffset
                    # global LongtimeOffset
                    # global AlttimeOffset

                    if LattimeOffset[0] is not None:
                        DiscoveryDroneOffsetlat = LattimeOffset[0]
                    LattimeOffset[0] = LattimeOffset[1]
                    LattimeOffset[1] = LattimeOffset[2]
                    LattimeOffset[2] = predictedLat

                    if LongtimeOffset[0] is not None:
                        DiscoveryDroneOffsetlong = LongtimeOffset[0]
                    LongtimeOffset[0] = LongtimeOffset[1]
                    LongtimeOffset[1] = LongtimeOffset[2]
                    LongtimeOffset[2] = predictedLong

                    if AlttimeOffset[0] is not None:
                        DiscoveryDroneOffsetalt = AlttimeOffset[0]
                    AlttimeOffset[0] = AlttimeOffset[1]
                    AlttimeOffset[1] = AlttimeOffset[2]
                    AlttimeOffset[2] = predictedUTMZ

                    # Addition of the Offset with orientation instead of only following (Advanced controls)##
                    # offset = 10 #10 meters of offset added, will eventually be user input
                    # if (predictedLat >= 39.0085455) & (predictedLong <= -104.879958): #top of the screen pos, left
                    #     DiscoveryDroneOffsetlat = predictedLat + (offset/111111) #subtr 10 meter from the predict pos
                    #     DiscoveryDroneOffsetlong = predictedLong + (offset/(111111*np.cos(DiscoveryDroneOffsetlat)))
                    #     DiscoveryDroneOffsetalt = predictedUTMZ + 3
                    # elif (predictedLat >= 39.0085455) & (predictedLong >= -104.879958): #  top right side
                    #     DiscoveryDroneOffsetlat = predictedLat - (offset/111111) #subtr 10 meter from the predict pos
                    #     DiscoveryDroneOffsetlong = predictedLong + (offset/(111111*np.cos(DiscoveryDroneOffsetlat)))
                    #     DiscoveryDroneOffsetalt = predictedUTMZ + 3
                    #
                    # elif (predictedLat <= 39.0085455) & (predictedLong >= -104.879958): #  bottom right side
                    #     DiscoveryDroneOffsetlat = predictedLat - (offset/111111) #subtr 10 meter from the predict pos
                    #     DiscoveryDroneOffsetlong = predictedLong + (offset/(111111*np.cos(DiscoveryDroneOffsetlat)))
                    #     DiscoveryDroneOffsetalt = predictedUTMZ + 3
                    #
                    # else: # bottom left side
                    #     DiscoveryDroneOffsetlat = predictedLat - (offset/111111) #subtr 10 meter from the predict pos
                    #     DiscoveryDroneOffsetlong = predictedLong + (offset/(111111*np.cos(DiscoveryDroneOffsetlat)))
                    #     DiscoveryDroneOffsetalt = predictedUTMZ + 3
                    if (LattimeOffset[0] is not None) & LongtimeOffset[0] is not None & (AlttimeOffset[0] is not None):
                        if not filter_id in DiscoveryDroneOffset_dict:
                            DiscoveryDroneOffset_dict[filter_id] = Discovery_Drone_Offset()

                        DiscoveryDroneOffset_dict[filter_id].xpos_offset = predictedUTMX - 10
                        DiscoveryDroneOffset_dict[filter_id].ypos_offset = predictedUTMY - np.cos(10)
                        DiscoveryDroneOffset_dict[filter_id].zpos_offset = predictedUTMZ + 3
                        DiscoveryDroneOffset_dict[filter_id].long_est_offset = DiscoveryDroneOffsetlong
                        DiscoveryDroneOffset_dict[filter_id].lat_est_offset = DiscoveryDroneOffsetlat
                        DiscoveryDroneOffset_dict[filter_id].alt_est_offset = DiscoveryDroneOffsetalt
                        DiscoveryDroneOffset_dict[filter_id].sensorTime = datetime.datetime.now(tz=pytz.utc).timestamp()

                        # Predicted Discovery Drone Position
                        with open('Expected_Discovery_Drone_Position_12_April.csv', 'a', newline='') as file:
                            fieldnames = ['TIMESTAMP', 'LATITUDE', 'LONGITUDE', 'ALTITUDE']
                            writer = csv.DictWriter(file, fieldnames=fieldnames)
                            writer.writeheader()
                            writer.writerow({'TIMESTAMP': datetime.datetime.fromtimestamp(
                                int(DiscoveryDroneOffset_dict[filter_id.num_track].sensorTime)).strftime(
                                '%Y-%m-%d %H:%M:%S'),
                                'LATITUDE': DiscoveryDroneOffset_dict[filter_id.num_track].lat_est_offset,
                                'LONGITUDE': DiscoveryDroneOffset_dict[filter_id.num_track].long_est_offset,
                                'ALTITUDE': DiscoveryDroneOffset_dict[filter_id.num_track].alt_est_offset})

                    # print(predictedLong)
                    # print(DiscoveryDroneOffset_dict[filter_id.num_track].long_est_offset)

            except:
                print("No Radar Measurement")

            # sendSensorFusionWaypoint()

            # global RadarFollow
            # try:
            #     if RadarFollow == True:
            #         #Determine offset
            #         goToWaypoint(senderD, predictedPositions_Dictionary[filter_id].lat_est, predictedPositions_Dictionary[filter_id].long_est,
            #                      predictedPositions_Dictionary[filter_id].alt_est, 5)
            #         print(RadarFollow)
            #
            # except:
            #     print("No Radar For the Discovery Drone to Follow")

            ## Acoustic Pod Fusion Section ##

    # Populating the Discovery Drone Dictionary
    elif packet_type == 'Discovery Drone Information':
        # track_id is set to 1 because there is only one discovery drone to track.
        #       However, this way the dictionary will not continually expand
        new_DiscDrone_info = Discovery_Drone_Information(packet[0], packet[1], packet[2], packet[3], packet[4],
                                                         packet[5], packet[6], packet[7], packet[8], packet[9],
                                                         packet[10], packet[11], packet[12], packet[13], packet[14],
                                                         packet[15], packet[16], packet[17], 1)
        if not new_DiscDrone_info.track_id in discoveryDroneDict:
            discoveryDroneDict[new_DiscDrone_info.track_id] = Discovery_Drone()

        # fills in the information relevant to the Discovery Drone Messages
        discoveryDroneDict[new_DiscDrone_info.track_id].timestamp = packet[2]
        discoveryDroneDict[new_DiscDrone_info.track_id].disc_lat = packet[3]
        discoveryDroneDict[new_DiscDrone_info.track_id].disc_long = packet[4]
        # discoveryDroneDict[new_DiscDrone_info.track_id].disc_lat = disc_lat
        # discoveryDroneDict[new_DiscDrone_info.track_id].disc_long = disc_long
        discoveryDroneDict[new_DiscDrone_info.track_id].disc_alt = packet[5]
        discoveryDroneDict[new_DiscDrone_info.track_id].vx = packet[6]
        discoveryDroneDict[new_DiscDrone_info.track_id].vy = packet[7]
        discoveryDroneDict[new_DiscDrone_info.track_id].vz = packet[8]
        discoveryDroneDict[new_DiscDrone_info.track_id].heading = packet[9]
        discoveryDroneDict[new_DiscDrone_info.track_id].time_last_message = datetime.datetime.now(
            tz=pytz.utc).timestamp()

        # Beginning of Discovery Drone Log
        with open('DiscoveryDrone TST.csv', 'a', newline='') as file:
            fieldnames = ['TIMESTAMP', 'LATITUDE', 'LONGITUDE', 'ALTITUDE', 'X Velocity', 'Y Velocity', 'Z Velocity',
                          'HEADING']
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerow({'TIMESTAMP': datetime.datetime.fromtimestamp(
                int(discoveryDroneDict[new_DiscDrone_info.track_id].timestamp)).strftime('%Y-%m-%d %H:%M:%S'),
                             'LATITUDE': discoveryDroneDict[new_DiscDrone_info.track_id].disc_lat,
                             'LONGITUDE': discoveryDroneDict[new_DiscDrone_info.track_id].disc_long,
                             'ALTITUDE': discoveryDroneDict[new_DiscDrone_info.track_id].disc_alt,
                             'X Velocity': discoveryDroneDict[new_DiscDrone_info.track_id].vx,
                             'Y Velocity': discoveryDroneDict[new_DiscDrone_info.track_id].vy,
                             'Z Velocity': discoveryDroneDict[new_DiscDrone_info.track_id].vz,
                             'HEADING': discoveryDroneDict[new_DiscDrone_info.track_id].heading})


def start_server():
    # """
    # Creates and starts the UDP server to grab all of our data. It binds to a port and continually listens.
    # :param shared_memory: the shared memory object across the server
    # :return: radar data message
    # """
    global status
    # receiving messages from the Simulated RADAR system and determining their type based on the message header
    try:
        raw_data = 0  # reset to 0 so you know whether you received something new
        raw_data = server_socket.recv(
            100000)  # capture packet...100000 is the buffer size, or maximum amount of data that can be received
        # print("Server Open")
    except:
        print("Did not parse data")
        status = -1
    if raw_data != 0:  # if successful capture
        status = 1
        length = len(raw_data)  # define data length
        # print(length)
        if (int(raw_data[0]) == 232) & (
                int(raw_data[1]) == 161):  # this is a simulated pod message, byte 1 = 0xe8, byte 2 = 0xa1
            if int(raw_data[4]) == 0:  # this is a simulated pod health message
                form = '<HHHHddddd'  # unpack based on specification
                data = struct.unpack(form, raw_data)
                parse(data, 'Simulated Acoustic Health Message')
            elif int(raw_data[4]) == 1:  # implying that it is a target message
                form = '<HHHHddddHd'  # unpack based on specification
                data = struct.unpack(form, raw_data)
                parse(data, 'Simulated Acoustic Target Message')

            else:
                print('Server Message Ignore')
        elif (int(raw_data[0]) == 48) & (
                int(raw_data[1]) == 0):  # Asterix type 48 message, First byte = 48, second byte = 0
            # lat_data = raw_data[40:44]
            # form = '>I'
            # lat = struct.unpack(form, lat_data)
            # lat_correct = lat[0]/1e5
            # print(lat_correct)
            global radar_data_48
            radar_data_48 = raw_data
            parse(radar_data_48, 'Radar Type 48')
            # print("Type 48")

        elif (raw_data[0] == 0x22) & (raw_data[1] == 0x00):  # Asterix Type 34 Message
            global radar_data_34
            radar_data_34 = raw_data
            parse(radar_data_34, 'Radar Type 34')
            # print("Type 34")

        elif (int(raw_data[0] == 110)) & (int(raw_data[1]) == 39):  # pod estimator message
            form = '<HHdddddHdHdHd'  # unpack based on specification
            data = struct.unpack(form, raw_data)
            parse(data, 'Pod Target Estimator')

        elif (int(raw_data[0] == 179)) & (int(raw_data[1]) == 229):
            tgt_byte = raw_data[4:6]
            numTgts = struct.unpack('<H', tgt_byte)
            form = '<3H'
            for t in range(
                    numTgts[0]):  # determine the number of targets and parse message based on flexible message size
                form = form + '2H4d'
            data = struct.unpack(form, raw_data)
            parse(data, 'True UAV Position')
            # print("True UAV Pos")

        else:
            print('Some other type of message')


# #################################################################################################
# Discovery Drone Control
# #################################################################################################
def returnToLaunch(linkup):  # Return to home Discovery Drone functon
    msgHdr = int("ace1", 16)  # based on message specification
    msgConfirm: int = int("d478", 16)  # based on message specification
    msgFormat = '<2H'
    message = [msgHdr, msgConfirm]
    linkup.sendto(struct.pack(msgFormat, *message), (hostD, portD))  # passing return to home message to port


def goToWaypoint(linkup, lat, long, alt, speed):
    msgHdr = int("abf2", 16)  # based on message specification
    msgLen = 26  # based on message specification
    msgFormat = '<HHdlllh'  # based on message specification
    # construction of the fields of a go to waypoint message, based on specification
    message = [msgHdr, np.uint16(msgLen), datetime.datetime.now(tz=pytz.utc).timestamp(), np.int32(lat * 1e7),
               np.int32(long * 1e7), np.int32(alt * 1e3)]
    if speed >= 0:
        sp = speed * 100
    else:
        sp = -1
    message.append(np.int16(sp))
    linkup.sendto(struct.pack(msgFormat, *message), (hostD, portD))  # passing waypoint message to port


def SetRoi(linkup, lat, long, alt):
    msgHdr = int("add0", 16)  # based on message specification
    msgLen = 24  # based on message specification
    msgFormat = '<HHdlll'  # based on message specification
    # construction of the fields of a go to waypoint message, based on specification
    message = [msgHdr, np.uint16(msgLen), datetime.datetime.now(tz=pytz.utc).timestamp(), np.int32(lat * 1e7),
               np.int32(long * 1e7), np.int32(alt * 1e3)]
    linkup.sendto(struct.pack(msgFormat, *message), (hostD, portD))  # passing waypoint message to port


def RadarFilterPropagation():  # function to propagate estimate in time even when no Radar Measurement present
    global filter_exists
    global GroundStationUTMx, GroundStationUTMy, baseAltitude, conversion, new_type_48, filter_id
    try:
        if filter_exists:
            current_time_sensor = datetime.datetime.now(tz=pytz.utc).timestamp()
            sensor_est = (current_time_sensor, 0, 0, 0, 0, 0, 0)
            predicted_state = Sensor.update_with_radar_position_estimate(target_filter, sensor_est,
                                                                         False)  # Measurement Flag set to false

            predictedX = predicted_state.X[0]
            predictedY = predicted_state.X[1]
            predictedZ = predicted_state.X[3]

            # Begin reconversion
            predictedUTMX = float(predictedX) + GroundStationUTMx
            predictedUTMY = float(predictedY) + GroundStationUTMy
            predictedUTMZ = float(predictedZ) + GroundStationUTMz
            # print(predictedUTMX)

            # Conversion back to Lat/Long
            # df = DataFrame(np.c_[x,y], columns=['Meters East', 'Meters South'])
            predictedLong, predictedLat = conversion(predictedUTMX, predictedUTMY, inverse=True)
            # print(predictedLat)

            # Populate the Dictionary that holds the predicted values
            if not filter_id in predictedPositions_Dictionary:
                predictedPositions_Dictionary[filter_id] = Sensor_estimate()

            predictedPositions_Dictionary[filter_id].xpos = predictedUTMX
            predictedPositions_Dictionary[filter_id].ypos = predictedUTMY
            predictedPositions_Dictionary[filter_id].zpos = predictedUTMZ
            predictedPositions_Dictionary[filter_id].long_est = predictedLong
            predictedPositions_Dictionary[filter_id].lat_est = predictedLat
            predictedPositions_Dictionary[filter_id].alt_est = predictedUTMZ
            predictedPositions_Dictionary[filter_id].sensorTime = current_time_sensor

            # Populate the filter information for future use
            filtersDictionary[filter_id].stateMatrix = predicted_state.X
            filtersDictionary[filter_id].covarianceMatrix = predicted_state.P
            filtersDictionary[filter_id].filterTime = predicted_state.current_time
            # print(filtersDictionary[filter_id].stateMatrix)
            print(filter_id)

            if current_time_sensor - filtersDictionary[filter_id].timeLastMeasurement > 10:
                predictedPositions_Dictionary[filter_id].filterExpiration = True

                # Time Delay by using a buffer to put Discovery Drone behind Target (Courtesy of C1C Erickson)
                global LattimeOffset
                global LongtimeOffset
                global AlttimeOffset

                if LattimeOffset[0] is not None:
                    DiscoveryDroneOffsetlat = LattimeOffset[0]
                LattimeOffset[0] = LattimeOffset[1]
                LattimeOffset[1] = LattimeOffset[2]
                LattimeOffset[2] = predictedLat

                if LongtimeOffset[0] is not None:
                    DiscoveryDroneOffsetlong = LongtimeOffset[0]
                LongtimeOffset[0] = LongtimeOffset[1]
                LongtimeOffset[1] = LongtimeOffset[2]
                LongtimeOffset[2] = predictedLong

                if AlttimeOffset[0] is not None:
                    DiscoveryDroneOffsetalt = AlttimeOffset[0]
                AlttimeOffset[0] = AlttimeOffset[1]
                AlttimeOffset[1] = AlttimeOffset[2]
                AlttimeOffset[2] = predictedUTMZ

                # Addition of the Offset with orientation instead of only following (Advanced controls)##
                # offset = 10 #10 meters of offset added, will eventually be user input
                # if (predictedLat >= 39.0085455) & (predictedLong <= -104.879958): #top of the screen position, left
                #     DiscoveryDroneOffsetlat = predictedLat + (offset/111111) #subtr 10 meter from the predict position
                #     DiscoveryDroneOffsetlong = predictedLong + (offset/(111111*np.cos(DiscoveryDroneOffsetlat)))
                #     DiscoveryDroneOffsetalt = predictedUTMZ + 3
                # elif (predictedLat >= 39.0085455) & (predictedLong >= -104.879958): #  top right side
                #     DiscoveryDroneOffsetlat = predictedLat - (offset/111111) #subtr 10 meter from the predict position
                #     DiscoveryDroneOffsetlong = predictedLong + (offset/(111111*np.cos(DiscoveryDroneOffsetlat)))
                #     DiscoveryDroneOffsetalt = predictedUTMZ + 3
                #
                # elif (predictedLat <= 39.0085455) & (predictedLong >= -104.879958): #  bottom right side
                #     DiscoveryDroneOffsetlat = predictedLat - (offset/111111) #subtr 10 meter from the predict position
                #     DiscoveryDroneOffsetlong = predictedLong + (offset/(111111*np.cos(DiscoveryDroneOffsetlat)))
                #     DiscoveryDroneOffsetalt = predictedUTMZ + 3
                #
                # else: # bottom left side
                #     DiscoveryDroneOffsetlat = predictedLat - (offset/111111) #subtr 10 meter from the predict position
                #     DiscoveryDroneOffsetlong = predictedLong + (offset/(111111*np.cos(DiscoveryDroneOffsetlat)))
                #     DiscoveryDroneOffsetalt = predictedUTMZ + 3
                if (LattimeOffset[0] is not None) & (LongtimeOffset[0] is not None) & (AlttimeOffset[0] is not None):
                    if not filter_id in DiscoveryDroneOffset_dict:
                        DiscoveryDroneOffset_dict[filter_id] = Discovery_Drone_Offset()

                    DiscoveryDroneOffset_dict[filter_id].xpos_offset = predictedUTMX - 10
                    DiscoveryDroneOffset_dict[filter_id].ypos_offset = predictedUTMY - np.cos(10)
                    DiscoveryDroneOffset_dict[filter_id].zpos_offset = predictedUTMZ + 3
                    DiscoveryDroneOffset_dict[filter_id].long_est_offset = DiscoveryDroneOffsetlong
                    DiscoveryDroneOffset_dict[filter_id].lat_est_offset = DiscoveryDroneOffsetlat
                    DiscoveryDroneOffset_dict[filter_id].alt_est_offset = DiscoveryDroneOffsetalt
                    DiscoveryDroneOffset_dict[filter_id].sensorTime = datetime.datetime.now(tz=pytz.utc).timestamp()

                    # Predicted Discovery Drone Position
                    with open('Expected_Discovery_Drone_Position_TST.csv', 'a', newline='') as file:
                        fieldnames = ['TIMESTAMP', 'LATITUDE', 'LONGITUDE', 'ALTITUDE']
                        writer = csv.DictWriter(file, fieldnames=fieldnames)
                        writer.writeheader()
                        writer.writerow({'TIMESTAMP': datetime.datetime.fromtimestamp(
                            int(DiscoveryDroneOffset_dict[new_type_48.num_track].sensorTime)).strftime(
                            '%Y-%m-%d %H:%M:%S'),
                            'LATITUDE': DiscoveryDroneOffset_dict[new_type_48.num_track].lat_est_offset,
                            'LONGITUDE': DiscoveryDroneOffset_dict[new_type_48.num_track].long_est_offset,
                            'ALTITUDE': DiscoveryDroneOffset_dict[new_type_48.num_track].alt_est_offset})
        sleep(0.1)
    except:
        print("Problem in Radar Propagation")

    # sendSensorFusionWaypoint(filter_id) commented out so data is not inudated


def BaseLocation():
    baseLatitude = 39.009170  # Home Base Latitude
    baseLongitude = -104.878982  # Home Base Longitude
    baseAltitude = 2153.00  # Home Base Altitude

    # unit conversion to UTM
    conversion = Proj(proj='utm', zone=13, datum='WGS84')
    GroundStationUTMx, GroundStationUTMy = conversion(baseLongitude, baseLatitude)
    return GroundStationUTMx, GroundStationUTMy, baseAltitude


def sendSensorFusionWaypoint(filter_number):
    global RadarFollow, target_lat_correct, target_long_correct, target_alt_correct, followMeSelection, True_UAV_Index
    # try:
    if RadarFollow == True:
        # Determine offset
        if GUI_Master.followme_select == 2:  # follow me selection for using radar
            goToWaypoint(senderD, target_lat_correct - (10 / 111111), target_long_correct,
                         target_alt_correct - 2, 5)
            print('Following Target Based on Radar')
            SetRoi(senderD, target_lat_correct, target_long_correct, target_alt_correct)

        if GUI_Master.followme_select == 1:  # follow me mode selection for True UAV
            goToWaypoint(senderD, trueUAVDictionary[True_UAV_Index].UAV_lat - (10 / 111111),
                         trueUAVDictionary[True_UAV_Index].UAV_lat,
                         trueUAVDictionary[True_UAV_Index].UAV_lat, 5)

            print("Following Target Based on True UAV")
        if GUI_Master.followme_select == 3:  # follow me mode using Sensor Fusion Estimate
            goToWaypoint(senderD, predictedPositions_Dictionary[filter_number].lat_est - (10 / 111111),
                         predictedPositions_Dictionary[filter_number].long_est,
                         predictedPositions_Dictionary[filter_number].alt_est, 5)
            print("Following Target Based on Sensor Fusion")
        print('Sending Waypoint')
    # print(filter_id)

    # except:
    # print("No Radar For the Discovery Drone to Follow")


if __name__ == '__main__':
    main()  # I bet Ryan broke the code
