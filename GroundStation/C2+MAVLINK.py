import tkinter as tk
from tkinter import *
from unittest import case

from PIL import ImageTk, Image
import threading
import datetime
import pytz
import socket
from time import sleep, time
import struct
import numpy as np  # Make sure NumPy is loaded before it is used in the callback
import math
import pymavlink
from GUItest import GUI
from time import sleep

from pymavlink import mavutil
import math
import numpy as np

# global variables
# listening port for Solo.  If using Solo 5, the last digit is set to 5, i.e. 14555
master = mavutil.mavlink_connection('udp:0.0.0.0:14555')
acoustic_array = []  # an array of the acoustic class objects
radar_array = []  # an array of radar class objects
c2todrones_array = []  # an array of C2toDrones class objects
disctoc2_array = []  # an array of DisctoC2 class objects
launch = 0
# Implementing a circular array to store the data
# Size (depth) of the circular array)
logDepth = 20
# Index of the most recent entry
array_new = -1
# Index of the oldest entry
array_old = 0
# Index of the most recent drone data pointer
drone_new = 0
# Boolean that tells if the array has been filled all the way or not
looped = 0
# mode [0 = standby, 1 = hover at home, 2 = launch to location, 3 = search, 4 = follow-me, 5 = RTB, etc
mode = 2
status = 1
# set distance between discovery drone and target in meters
standoffdist_m = 5.0
discoveryIP = 40  # the full ip address is 192.168.1.40

# server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# logfiles
c2todrones_log = 'c2todrones_' + str(int(datetime.datetime.now(tz=pytz.utc).timestamp())) + '.bin'


# hi
# class Pod_Data:
#     def __init__(self):
#         self.acoustics = []
#
#     def store_acoustics(self, acoustic_data):
#         index = acoustic_data.pod_id
#         self.acoustics[index] = acoustic_data

class Acoustic:
    def __init__(self, header, pod_id, time_unix_sec, podlat_deg, podlong_deg, podalt_mmsl, tgttype, anglearrival_deg,
                 batterylife):
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

    def update_grid(self, new_x, new_y):
        self.grid_x = new_x
        self.grid_y = new_y


class Radar:
    def __init__(self):
        self.header = 0
        self.radar_id = 0
        self.time_unix_sec = 0
        self.radarLat_deg = 0
        self.radarLong_deg = 0
        self.radarAlt_mMSL = 0
        self.numTgts = 0
        self.drones = []

    def store_info(self, locs):
        self.header = locs[0]
        self.radar_id = locs[1]
        self.time_unix_sec = locs[2]
        self.radarLat_deg = locs[3]
        self.radarLong_deg = locs[4]
        self.radarAlt_mMSL = locs[5]
        self.numTgts = locs[6]
        for j in range(int(self.numTgts)):
            drone_info = locs[7 + (3 * j):10 + (3 * j)]
            new_drone = Drone_Radar()
            new_drone.create_drone(drone_info, self.time_unix_sec)
            self.drones.append(new_drone)


class Drone_Radar():
    def __init__(self):
        self.time_unix_sec = 0
        self.range_m = 0
        self.azimuth_deg = 0
        self.elevation_deg = 0
        self.grid_x = 0
        self.grid_y = 0
        self.flylat_deg = 0
        self.flylong_deg = 0

    def create_drone(self, info, time):
        self.range_m = info[0]
        self.azimuth_deg = info[1]
        self.elevation_deg = info[2]
        self.time_unix_sec = time
        return self

    def store_grid_loc(self, x, y, width, height):
        self.grid_x = x
        self.grid_y = y
        self.find_lat_long(width, height)

    def find_lat_long(self, width, height):
        rel_y = self.grid_y / height
        rel_y *= (39.010327 - 39.007801)
        self.flylat_deg = 39.010327 - rel_y
        rel_x = self.grid_x / width
        rel_x *= (104.884606 - 104.878072)
        self.flylong_deg = -104.884606 + rel_x


class C2toDrones:
    def __init__(self, header, src_ip, time_unix_sec, mode, flylat_deg, flylong_deg, flyalt_mmsl, nefvx_ms, nefvy_ms,
                 nefvz_ms,
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
    def __init__(self, src_ip, time_unix_sec, disclat_deg, disclong_deg, discalt_mmsl, discvx_ms, discvy_ms, discvz_ms,
                 yaw_deg, timetostandoff_sec):
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


# we did not get to the mitigate drone, but here is what it would send you once you get to it class MitigatetoC2:
class MitigatetoC2:
    def __init__(self, header, src_ip, time_unix_sec, intLat_deg, intLong_deg, intAlt_deg, v_ms):
        self.header = header
        self.src_ip = src_ip
        self.time_unix_sec = time_unix_sec
        self.intLat_deg = intLat_deg
        self.intLong_deg = intLong_deg
        self.intAlt_deg = intAlt_deg
        self.v_ms = v_ms


def main():
    program = GUI.run_main(radar_array, array_old, array_new)
    mavutil.set_dialect("ardupilotmega")

    # verify can talk to target
    master.wait_heartbeat(blocking=True)
    print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_system))
    while True:
        start_server()
        program.update_log(radar_array, array_old, array_new, acoustic_array)
        if status == -1:
            program.indicate_no_connection()
        program.runGUI()
        program.window.update()
        # send_c2todrones()


# creates the class object and appends the respective array of those class objects
# ie, creates a radar object and adds it to the radar array
def parse(packet, packet_type):
    global array_new, array_old, looped
    global radar_array, acoustic_array
    if packet_type == "Acoustic":
        new_acoustic = Acoustic(packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6], packet[7],
                                packet[8])
        index = int(new_acoustic.pod_id)
        if (len(acoustic_array) <= index):
            acoustic_array.append(new_acoustic)
        else:
            acoustic_array[index] = new_acoustic
        print(acoustic_array[-1])
    elif packet_type == "Radar":
        new_message = Radar()
        if looped == 0:
            # Throwing the new message into a new slot at the end
            new_message.store_info((packet))
            radar_array.append(new_message)
            # Incrementing the recent variable to point to the latest data
            array_new += 1
            # Determining if the array was filled after this entry
            if array_new >= logDepth:
                # Changing the boolean if the array was filled
                looped = 1
        # Storing data properly if the array is full
        # test
        else:
            # Incrementing the recent variable to point to the most recent data location
            array_new += 1
            # Looping to the beginning of the circular array after the last entry
            if array_new > logDepth:
                array_new = 0
            # Determining the location of the oldest data
            array_old = array_new + 1
            # Looping the location of the oldest data at the end of the circular array
            if array_old > logDepth:
                array_old = 0
            new_message.store_info(packet)
            radar_array[array_new] = new_message
    elif packet_type == "DisctoC2":
        disctoc2_array.append(
            DiscoverytoC2(int(packet[64:128]), int(packet[128:192]), int(packet[192:256]), int(packet[256:320]),
                          int(packet[320:384]), int(packet[384:448]), int(packet[448:512]), int(packet[512:576]),
                          int(packet[576:640]), int(packet[640:704])))

    # Packet/Message Types
    #         #1-acoustic
    #         #2-radar
    #         #3-C2 to Drones
    #         #4-discovery
    #         #5-mitigate
    #         #6-images


# make this a function that only turns on the server
def start_server():
    """
    Creates and starts the UDP server to grap all of our data. It binds to a port and continually listens.
    :param shared_memory: the shared memory object across the server
    :return: radar data message
    """
    global status

    # Defines our IP's
    # radar_ip = '192.168.1.61'
    port_num = 55565
    # radar_message_length = 72
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(('', port_num))
    server_socket.settimeout(1)
    sleep(0.5)
    # print("UDP SERVER: RUNNING")
    try:
        raw_data = 0  # reset to 0 so you know whether you received something new
        raw_data = server_socket.recv(100000)
        # capture packet...100000 is the buffer size, or maximum amount of data that can be received
    except:
        print("Did not parse data")
        status = -1
    if raw_data != 0:  # if successful capture
        status = 1
        length = len(raw_data)  # define data length
        form = '<' + str(int(length / 8)) + 'd'  # data structure is little endian and in doubles
        data = struct.unpack(form, raw_data)
        header = int(data[0])
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
    port_num = 11752
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(('', port_num))
    server_socket.settimeout(1)
    sleep(0.5)

    # if radar is populated, then we can populate C2toDrones
    if len(radar_array) > 0:
        myradar = radar_array[-1]  # get the most recent radar message
        flyalt_mmsl = myradar.drones[-1].range_m * math.sin(
            np.radians(myradar.drones[-1].elevation_deg))  # takes elevation and range to calculate altitude
        c2todrones_array.append(C2toDrones(3, discoveryIP, myradar.time_unix_sec, mode, myradar.drones[-1].flylat_deg,
                                           myradar.drones[-1].flylong_deg, flyalt_mmsl, 0, 0, 0, standoffdist_m))
        print(c2todrones_array[-1])
        myobject = c2todrones_array[-1]  # send the last c2todrones object
        myradar = radar_array[-1]  # get the most recent radar message
        if launch == 0:
            launched = 1
            # launch aircraft

            msg3 = (master.target_system,
                    1,
                    4)  # set to guided mode
            print(msg3)
            master.mav.set_mode_send(*msg3)  # Send msg twice to ensure it is received
            master.mav.set_mode_send(*msg3)

            sleep(1)
            master.arducopter_arm()  # spin up props
            sleep(4)

            # take off (use location for launch)
            msg3 = (master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # (22)
                    0,  # confirmation
                    0,  # param 1 - min pitch
                    0,  # param 2 - empty
                    0,  # param 3 - empty
                    math.nan,  # param 4 - yaw (NaN is no change)
                    myobject.flylat_deg,  # param 5: latitude (deg)
                    myobject.flylong_deg,  # param 6: longitude (deg)
                    myobject.flyalt_mmsl * 3.28084)  # param 7: altitude
            print(msg3)
            master.mav.command_long_send(*msg3)
            master.mav.command_long_send(*msg3)

            # wait for correct height
            NotAtAlt = True
            while NotAtAlt:
                msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                if msg.relative_alt / 1e3 > myobject.flyalt_mmsl * 3.28084 * 0.9:
                    NotAtAlt = False

            print("At altitude")
        # right now we take the most recent drone detected, because in theory the last drone detected should be the enemy drone
        # but, you will need to handle multiple enemy drones and assign them to one discovery drone.
        # you'll also need to distinguish between friendly drones and enemy drones. You can do this by comparing the location
        # that our drone is sending up to the blip on the radar. If it is the same, ignore that one it's not an enemy.
        if radar_array[-1].numTgts > 0:
            # set ROI  (location of target.  This could be updated in the loop that repositions the aircraft)
            msg3 = (master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_ROI,  # (201)
                    #                     mavutil.mavlink.MAV_CMD_NAV_ROI, #(80)
                    0,  # confirmation
                    mavutil.mavlink.MAV_ROI_LOCATION,  # param 1 - ROI type (3)
                    0,  # param 2 - WP index
                    0,  # param 3 - ROI index
                    0,  # param 4 - empty
                    myobject.flylat_deg,  # param 5: latitude (deg)
                    myobject.flylong_deg,  # param 6: longitude (deg)
                    myobject.flyalt_mmsl * 3.28084)  # param 7: altitude
            print(msg3)
            master.mav.command_long_send(*msg3)
            master.mav.command_long_send(*msg3)

            sleep(2)
            # loop to fly waypoints (these would track the rogue UAV)
            msg3 = (master.target_system,
                    master.target_component,
                    6,  # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                    0b0000111111111000,  # copied from DroneKit
                    myobject.flylat_deg,  # latitude (deg)
                    myobject.flylong_deg,  # longitude (deg)
                    myobject.flyalt_mmsl * 3.28084,  # altitude
                    0, 0, 0,  # vx,vy,vz
                    0, 0, 0,  # afx,afy,afz
                    0, 0)  # yaw,yaw rate

            print(msg3)
            master.mav.position_target_global_int_send(*msg3)
            master.mav.position_target_global_int_send(*msg3)

            # how to return home
            # master.set_mode_rtl()

    #
    # if(len(c2todrones_array) > 0):
    #     myobject = c2todrones_array[-1] #send the last c2todrones object
    #     msg = np.zeros(11, dtype='float')
    #     msg = (float(myobject.header), float(myobject.src_ip), float(myobject.time_unix_sec), float(myobject.mode), float(myobject.flylat_deg),
    #            float(myobject.flylong_deg), float(myobject.flyalt_mmsl), float(myobject.nefvx_ms), float(myobject.nefvy_ms), float(myobject.nefvz_ms),
    #            float(myobject.standoffdist_m))
    #
    #     packed_msg = struct.pack('<11d', *msg)
    #     server_socket.sendto(packed_msg, ("192.168.1.40", 11752))
    #
    #     with open(c2todrones_log,'ab') as f: #ab to append
    #             np.array(msg).tofile(f)
    # else:
    #     print("There's no object in c2todrones_array")


main()
