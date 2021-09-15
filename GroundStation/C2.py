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

#global variables
acoustic_array = [] #an array of the acoustic class objects
radar_array = [] #an array of radar class objects
c2todrones_array = [] #an array of C2toDrones class objects
disctoc2_array = [] #an array of DisctoC2 class objects
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
c2todrones_log = 'c2todrones_' + str(int(datetime.datetime.now(tz=pytz.utc).timestamp())) + '.bin'

def main():
    # Creates a new GUI object and stores appropriate info
    program = GUI.run_main(radar_array, array_old, array_new)
    # Constantly running the code
    while True:
        # Receive and store messages into object classes
        start_server()
        # Update the correct variables in the GUI object
        program.update_log(radar_array, array_old, array_new, drone_new, acoustic_array)
        # Indicate to the GUI that no messages have been received
        if(status==-1):
            program.indicate_no_connection()
        # Updating the GUI display after the variables were passed in
        program.runGUI()
        # Tkinter method that actually updates the GUI
        program.window.update()
        # send_c2todrones()

# class Pod_Data:
#     def __init__(self):
#         self.acoustics = []
#
#     def store_acoustics(self, acoustic_data):
#         index = acoustic_data.pod_id
#         self.acoustics[index] = acoustic_data

#CLASSES OF MESSAGES: See Message structure in the google drive for an explanation of each variable
#class Acoustic_Health:
    #def __init__(self, msg_id, msg_size, msg_type,pod_id, time_stamp, pod_lat, pod_long, pod_alt, pod_bat)
        #self.msg_id = msg_id
        #self.msg_size = msg_size
        #self.msg_type = msg_type
        #self.pod_id = pod_id
        #self.time_stamp = time_stamp
        #self.pod_lat = pod_lat
        #self.pod_long = pod_long
        #self.pod_alt = pod_alt
        #self.pod_bat = pod_bat

#class Acoustic_Target:
    #def __init__(self, msg_id, msg_size, msg_type,pod_id, time_stamp, pod_lat, pod_long, pod_alt, pod_bat)
        #self.msg_id = msg_id
        #self.msg_size = msg_size
        #self.msg_type = msg_type
        #self.pod_id = pod_id
        #self.time_stamp = time_stamp
        #self.pod_lat = pod_lat
        #self.pod_long = pod_long
        #self.pod_alt = pod_alt
        #self.pod_bat = pod_bat


class Acoustic:
    def __init__(self, header, pod_id, time_unix_sec, podlat_deg, podlong_deg, podalt_mmsl, tgttype, anglearrival_deg, batterylife):
        self.header = header
        self.pod_id = pod_id
        self.time_unix_sec = time_unix_sec
        self.podlat_deg = podlat_deg
        self.podlong_deg = podlong_deg
        self.podalt_mmsl = podalt_mmsl
        self.tgttype = tgttype
        self.anglearrival_deg = anglearrival_deg
        self.batterylife = batterylife
        self.grid_x = 0
        self.grid_y = 0

    # This method stores the x and y locations of the acoustic pod on the GUI map
    def update_grid(self, new_x, new_y):
        self.grid_x = new_x
        self.grid_y = new_y

# This class stores all of the info received from the radar, as well as some info we calculate using radar data
class Radar:
    # This method creates a new radar object and initializes everything to 0
    def __init__(self):
        self.header = 0
        self.radar_id = 0
        self.time_unix_sec = 0
        self.radarLat_deg = 0
        self.radarLong_deg = 0
        self.radarAlt_mMSL = 0
        self.numTgts = 0
        self.drone_new = drone_new
        self.drones = []

    # This method stores the messages from the radar into the radar object
    # locs is an array containing information sent by the radar software
    def store_info(self, locs):
        # Storing given data
        self.header = locs[0]
        self.radar_id = locs[1]
        self.time_unix_sec = locs[2]
        self.radarLat_deg = locs[3]
        self.radarLong_deg = locs[4]
        self.radarAlt_mMSL = locs[5]
        self.numTgts = locs[6]
        # Determining the location of drone_new
        # (You might want to do bug testing with drone_new by turning the radar drones
        #   off and back on again while it is still sending messages. It is possible it
        #   needs a bit more work to be perfect)
        if(self.numTgts>0):
            self.drone_new = drone_new + 1
        # Looping drone_new back to 0 if it is at the end of the circular array
        if (self.drone_new > logDepth):
            self.drone_new = 0
        # Dynamically storing any number of drones that the radar sends us
        for j in range(int(self.numTgts)):
            # Getting the correct info from the radar message array
            drone_info = locs[7 + (3 * j):10 + (3 * j)]
            # Creating a new drone_radar object (drone data from radar)
            new_drone = Drone_Radar()
            # Storing the appropriate info into the object
            new_drone.create_drone(drone_info, self.time_unix_sec)
            # Appending the drone onto the drone array in the radar data object
            self.drones.append(new_drone)

# This class stores drone data that was received from the radar
class Drone_Radar():
    # This method creates a new drone_radar object (everything is initially set to 0)
    # It defines what variables will be stored within the object
    def __init__(self):
        self.time_unix_sec = 0
        self.range_m = 0
        self.azimuth_deg = 0
        self.elevation_deg = 0
        self.grid_x = 0
        self.grid_y = 0
        self.flylat_deg = 0
        self.flylong_deg = 0

    # This method stores the appropriate drone data
    # info is an array containing range, azimuth, and elevation from the radar
    # time is the timestamp of the radar message (in Unix seconds)
    # It returns itself (the drone object that now has the data properly stored)
    def create_drone(self, info, time):
        self.range_m = info[0]
        self.azimuth_deg = info[1]
        self.elevation_deg = info[2]
        self.time_unix_sec = time
        return self

    # This method stores the x and y coordinates of each drone, and estimates a latitude and longitude for them as well
    # x is the x location of the drone on the GUI map
    # y is the y location of the drone on the GUI map
    # width is the width (in pixels/x-coordinates) of the GUI map image
    # height is the height (in pixels/y-coordinates) of the GUI map image
    def store_grid_loc(self, x, y, width, height):
        # Storing the x and y locations
        self.grid_x = x
        self.grid_y = y
        # Estimating a latitude and longitude for the drone
        self.find_lat_long(width, height)

    # This method estimates a rough latitude and longitude by reverse engineering the x and y locations of the GUI map
    def find_lat_long(self, width, height):
        # Getting the relative y position
        rel_y = self.grid_y/height
        # Converting that into relative latitude numbers
        rel_y *= (39.010327-39.007801)
        # Finding and storing the actual latitude estimate (remember that the y coordinates are inverted on the GUI so 0 is at the top)
        self.flylat_deg = 39.010327 - rel_y
        # Getting the relative x position
        rel_x = self.grid_x/width
        # Converting that into a relative longitude
        rel_x *= (104.884606-104.878072)
        # Finding and storing the actual longitude estimate
        self.flylong_deg = -104.884606 + rel_x

class C2toDrones:
    def __init__(self, header, src_ip, time_unix_sec, mode, flylat_deg, flylong_deg, flyalt_mmsl, nefvx_ms, nefvy_ms, nefvz_ms,
                 standoffdist_m):
        self.header = header
        self.src_ip = src_ip
        self.time_unix_sec = time_unix_sec
        self.mode = mode
        self.flylat_deg = flylat_deg
        self.flylong_deg = flylong_deg
        self.flyalt_mmsl = flyalt_mmsl
        self.nefvx_ms = nefvx_ms
        self.nefvy_ms = nefvy_ms
        self.nefvz_ms = nefvz_ms
        self.standoffdist_m = standoffdist_m

class DiscoverytoC2:
    def __init__(self, src_ip, time_unix_sec, disclat_deg, disclong_deg, discalt_mmsl, discvx_ms, discvy_ms, discvz_ms, yaw_deg,
                 timetostandoff_sec):
        self.src_ip = src_ip
        self.time_unix_sec = time_unix_sec
        self.disclat_deg = disclat_deg
        self.disclong_deg = disclong_deg
        self.discalt_mmsl = discalt_mmsl
        self.discvx_ms = discvx_ms
        self.discvy_ms = discvy_ms
        self.discvz_ms = discvz_ms
        self.yaw_deg = yaw_deg
        self.timetostandoff_sec = timetostandoff_sec

# we did not get to the mitigate drone, but here is what it would send you once you get to it
class MitigatetoC2:
    def __init__(self, header, src_ip, time_unix_sec, intLat_deg, intLong_deg, intAlt_deg, v_ms):
        self.header = header
        self.src_ip = src_ip
        self.time_unix_sec = time_unix_sec
        self.intLat_deg = intLat_deg
        self.intLong_deg = intLong_deg
        self.intAlt_deg = intAlt_deg
        self.v_ms = v_ms


#creates the class object and appends the respective array of those class objects
# ie, creates a radar object and adds it to the radar array
def parse(packet, packet_type):
    # Ensuring we can update the global variables in this function
    global array_new, array_old, looped, drone_new
    global radar_array, acoustic_array
    # If the message was from the acoustic pod

        ###if packet_type = 'Simulated Acoustic Health Message'
        #new_acoustic = Acoustic_Health(packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6], packet[7],
                               # packet[8], packet[9])
        ###if packet_type = 'Simulated Acoustic Target Message'
        # new_acoustic = Acoustic_Target(packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6], packet[7],
        # packet[8], packet[9], packet[10])


    if packet_type == "Acoustic":
        # Creating a new acoustic object and storing the appropriate info in it
        new_acoustic = Acoustic(packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6], packet[7], packet[8])
        # Using the index to determine which pod the message came from
        index = int(new_acoustic.pod_id)
        # Appending the message if we have not heard from this pod yet
        # (Note that pod numbering should start at 1 to accommodate this)
        # (Also note that this method of storing separate pods is untested
        #   and almost definitely needs to be fixed in the future)
        if(len(acoustic_array)<=index):
            # Appending the new acoustic data object
            acoustic_array.append(new_acoustic)
        # If the array already has the proper number of acoustic pods in it based on pod_id
        else:
            # Storing the pod data in the appropriate location
            acoustic_array[index] = new_acoustic
        # Printing the new acoustic object to indicate to the user that we are indeed receiving acoustic messages
        # (This helps with debugging, you can comment it out or delete it if needed)
        print(acoustic_array[-1])
    # If the message was from the radar
    elif packet_type == "Radar":
        # Creating a new radar object to hold our new message
        new_message = Radar()
        # Checking to see if the circular array is full to determine where to store the new message
        if (looped == 0):
                # Calling a method that stores the info in packet into the correct object variables in message
                new_message.store_info((packet))
                # Throwing the new message into a new slot at the end
                radar_array.append(new_message)
                # Incrementing the recent variable to point to the latest data
                array_new += 1
                # Determining if the array was filled after this entry
                if(array_new>=logDepth):
                    # Changing the boolean if the array was filled
                    looped = 1
        # Storing data properly if the array is full
        else:
            # Incrementing the recent variable to point to the most recent data location
            array_new += 1
            # Looping to the beginning of the circular array after the last entry
            if (array_new > logDepth):
                array_new = 0
            # Determining the location of the oldest data
            array_old = array_new + 1
            # Checking to see if we have looped in the array
            if(array_old>logDepth):
                # Looping the location of the oldest data at the end of the circular array
                array_old = 0
            # Calling a method that stores the info in packet into the correct object variables in message
            new_message.store_info(packet)
            # Inserting the new message at the correct location
            radar_array[array_new] = new_message
        # Setting the location of drone_new (the most recent drone data) into the global variable
        drone_new = new_message.drone_new
    # If the packet was from the discovery drone (we never actually used this)
    elif packet_type == "DisctoC2":
        disctoc2_array.append(DiscoverytoC2(int(packet[64:128]), int(packet[128:192]), int(packet[192:256]), int(packet[256:320]), int(packet[320:384]), int(packet[384:448]), int(packet[448:512]), int(packet[512:576]), int(packet[576:640]), int(packet[640:704])))

    #Packet/Message Types
    #         #1-acoustic
    #         #2-radar
    #         #3-C2 to Drones
    #         #4-discovery
    #         #5-mitigate
    #         #6-images

## In order to implement Switch case statements into python
# def message_type(type)
    #switcher = {
        #e8a1: 'Acoustic'



# this starts the server and identifies what type of message an incoming message is
def start_server():
    """
    Creates and starts the UDP server to grap all of our data. It binds to a port and continually listens.
    :param shared_memory: the shared memory object across the server
    :return: radar data message
    """
    global status

    # Defines our IP's
    #radar_ip = '192.168.1.61'
    port_num = 55565
    # radar_message_length = 72
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(('', port_num))
    server_socket.settimeout(1)
    sleep(0.5)
    # print("UDP SERVER: RUNNING")
    try:
        raw_data = 0  # reset to 0 so you know whether you received something new
        raw_data = server_socket.recv(100000)  # capture packet...100000 is the buffer size, or maximum amount of data that can be received
    except:
        print("Did not parse data")
        status = -1
    if (raw_data != 0):  # if successful capture
        status = 1
        length = len(raw_data)  # define data length
        raw_data_output = str(raw_data)
        print(raw_data_output)
        #if raw_data[0] = 0xe8a1: #this is a simulated pod message
            #if raw_data[3] = 0:  #this is a simulated pod health message
                #form = '< + HHHHddddd'
                #data = struct.unpack(form, raw_data)
                #parse(data, 'Simulated Acoustic Health Message')
            #elif raw_data[3] = 1: #implying that it is a target message
                #form = '< + HHHHddddHd'
                #data = struct.unpack(form, raw_data)
                #parse(data, 'Simulated Acoustic Target Message')

            #else:
                #print('Server Message Ignore')
        #else: #a different type of message, functionality to be added later



        #if length = 48:   #if the length of the data is 48 bytes then this is a Simulated Health Message
        #form = '<' + 'IIIIddddd'


        #if length = 50: #if the length is 50 bytes, this is the Target Pod message
        #form = '<' +

        form = '<' + str(int(length / 8)) + 'd'  # data structure is little endian and in doubles
        data = struct.unpack(form, raw_data)
        header = int(data[0])
        #identifies what type of message it is
        if header == 1:
            packet_type = "Acoustic"
            parse(data, packet_type)
        elif header == 2:
            packet_type = "Radar"
            parse(data, packet_type)
        elif header == 4:
            packet_type = "DisctoC2"
            parse(data, packet_type)
        else:
            packet_type = "Unknown"

def send_c2todrones():
    #open a socket
    port_num = 11752
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(('', port_num))
    server_socket.settimeout(1)
    sleep(0.5)


    #we never used this, but instead you might just want to send MAVLINK messages directly to the drone instead of this to the on-board raspberry pi
    #if radar is populated, then we can populate C2toDrones
    if(len(radar_array) > 0):
        myradar = radar_array[-1] #get the most recent radar message

        # right now we take the most recent drone detected, because in theory the last drone detected should be the enemy drone
        # but, you will need to handle multiple enemy drones and assign them to one discovery drone.
        #you'll also need to distinguish between friendly drones and enemy drones. You can do this by comparing the location
        #that our drone is sending up to the blip on the radar. If it is the same, ignore that one it's not an enemy.
        if(radar_array[-1].numTgts > 0):
            flyalt_mmsl = myradar.drones[-1].range_m*math.sin(np.radians(myradar.drones[-1].elevation_deg)) #takes elevation and range to calculate altitude
            c2todrones_array.append(C2toDrones(3, discoveryIP, myradar.time_unix_sec, mode, myradar.drones[-1].flylat_deg, myradar.drones[-1].flylong_deg, flyalt_mmsl, 0, 0, 0, standoffdist_m))
            print(c2todrones_array[-1])

    if(len(c2todrones_array) > 0):
        myobject = c2todrones_array[-1] #send the last c2todrones object
        msg = np.zeros(11, dtype='float')
        msg = (float(myobject.header), float(myobject.src_ip), float(myobject.time_unix_sec), float(myobject.mode), float(myobject.flylat_deg),
               float(myobject.flylong_deg), float(myobject.flyalt_mmsl), float(myobject.nefvx_ms), float(myobject.nefvy_ms), float(myobject.nefvz_ms),
               float(myobject.standoffdist_m))

        packed_msg = struct.pack('<11d', *msg)
        server_socket.sendto(packed_msg, ("192.168.1.40", 11752)) #send the message

        with open(c2todrones_log,'ab') as f: #ab to append
                np.array(msg).tofile(f)
    else:
        print("There's no object in c2todrones_array")

main()

