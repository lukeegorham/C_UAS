#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov  3 09:32:57 2020

@author: pi
"""
import sys
import serial.tools.list_ports
from pymavlink import mavutil
import numpy as np
import socket
import struct
import threading
from time import sleep

udpServerSocketPort = 45454
udpRecvSocketPort = 50505
bufferSize = 1024


def wait_heartbeat():
    #    print("Waiting for heartbeat")
    mavConnection.wait_heartbeat()


def readMavlinkAndForwardUdp(linkdown):
    # set up message read and writes
    # define send variables

    global timestamp
    timestamp = 0
    lat_degE7 = 0
    lon_degE7 = 0
    global alt_mmMSL
    alt_mmMSL = 0
    velX_cmps = 0
    velY_cmps = 0
    velZ_cmps = 0
    heading_cdeg = 0
    baseTime = 0  # track utm time relative to autopilot start time
    # set up local globals
    global currentLat_Deg
    currentLat_Deg = 0
    global currentLon_Deg
    currentLon_Deg = 0
    global currentAlt_mMSL
    currentAlt_mMSL = 0
    global currentVelX_mps
    currentVelX_mps = 0
    global currentVelY_mps
    currentVelY_mps = 0
    global currentVelZ_mps
    currentVelZ_mps = 0
    global currentHeading_deg
    currentHeading_deg = 0
    global normalControl

    sendFormat = '<HHdlllhhhH'
    sendHeader = int("ac03", 16)
    while True:
        # send messages
        try:
            #            msg = mavConnection.recv_match(type=['SYSTEM_TIME','GLOBAL_POSITION_INT','HEARTBEAT','COMMAND_ACK'],blocking=True)
            msg = mavConnection.recv_match(type=['SYSTEM_TIME', 'GLOBAL_POSITION_INT', 'HEARTBEAT'], blocking=True)
            if not msg:
                continue

            #            print("Message Received")

            if msg.get_type() == 'SYSTEM_TIME':
                baseTime = msg.time_unix_usec / 1e6 - msg.time_boot_ms / 1e3

            if msg.get_type() == 'GLOBAL_POSITION_INT':
                timestamp = msg.time_boot_ms / 1e3 + baseTime
                lat_degE7 = msg.lat
                lon_degE7 = msg.lon
                alt_mmMSL = msg.alt
                velX_cmps = msg.vx
                velY_cmps = msg.vy
                velZ_cmps = msg.vz
                heading_cdeg = msg.hdg
                currentLat_Deg = lat_degE7 / 1e7
                currentLon_Deg = lon_degE7 / 1e7
                currentAlt_mMSL = alt_mmMSL / 1e3
                currentVelX_mps = velX_cmps / 1e2
                currentVelY_mps = velY_cmps / 1e2
                currentVelZ_mps = velZ_cmps / 1e2
                currentHeading_deg = heading_cdeg / 1e2

                msgLeng = 32
                msgOut = [sendHeader]
                msgOut.append(np.uint16(msgLeng))
                msgOut.append(timestamp)
                msgOut.append(np.int32(lat_degE7))
                msgOut.append(np.int32(lon_degE7))
                msgOut.append(np.int32(alt_mmMSL))
                msgOut.append(np.int16(velX_cmps))
                msgOut.append(np.int16(velY_cmps))
                msgOut.append(np.int16(velZ_cmps))
                msgOut.append(np.uint16(heading_cdeg))
                linkdown.sendto(struct.pack(sendFormat, *msgOut), ("192.168.1.255", udpRecvSocketPort))

            if msg.get_type() == 'HEARTBEAT':
                if ((msg.custom_mode == mavConnection.mode_mapping()['RTL']) or
                        (msg.custom_mode == mavConnection.mode_mapping()['LAND'])):
                    if normalControl:
                        print('HeartbeatMSG: In Safe RTL mode or land')
                        normalControl = False
            if msg.get_type() == 'COMMAND_ACK':
                print('msg.command:' + str(msg.command) + ', msg.result: ' + str(msg.result))

        except (KeyboardInterrupt):
            raise
        except:
            e = sys.exc_info()[0]
            print(e)


def readUdpAndSendMavlinkCommand(uplink):
    fly2Wp_Hdr = int("abf2", 16)
    flyRTL_Hdr = int("ace1", 16)
    msg_confirm = int("d478", 16)
    setRoi_Hdr = int("add0", 16)
    tiltCam_Hdr = int("aecf", 16)
    rcGimbal_Hdr = int("afbe", 16)

    # the onboard Raspberry Pi may refuse GS control for all commands except RTL

    while True:
        try:
            msgIn, ipSource = uplink.recvfrom(bufferSize)
            temp = msgIn[:2]  # separate header to test message
            msgId = struct.unpack('<H', temp)[0]

            # fly 2 waypoint command
            print(msgId)
            if (msgId == fly2Wp_Hdr and allowGsControl):
                mode_id = mavConnection.mode_mapping()[
                    'GUIDED']  # added by Max after 16 Mar test flight, because drone was not entering guided mode
                mavConnection.mav.command_long_send(mavConnection.target_system, mavConnection.target_component,
                                                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, mode_id, 0, 0, 0, 0, 0)
                msgFormat = '<HHdlllh'
                data = struct.unpack(msgFormat, msgIn)
                Fly2Wp(data[3] / 1e7, data[4] / 1e7, data[5] / 1e3, data[6] / 100)
                print("F2WP MSG: En-Route")
                # print(f"Lat: {data[3]/1e7}, Lon: {data[4]/1e7}")

            # RTL command
            if (msgId == flyRTL_Hdr):
                msgFormat = '<2H'
                data = struct.unpack(msgFormat, msgIn)
                if (data[1] == msg_confirm):  # verify command with second short
                    print("RTL MSG: Going Home")
                    RTL()

            # set ROI command
            if (msgId == setRoi_Hdr and allowGsControl):
                mode_id = mavConnection.mode_mapping()[
                    'GUIDED']  # added by Max after 16 Mar test flight, because drone was not entering guided mode
                mavConnection.mav.command_long_send(mavConnection.target_system, mavConnection.target_component,
                                                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, mode_id, 0, 0, 0, 0, 0)
                msgFormat = '<HHdlll'
                print("here")
                data = struct.unpack(msgFormat, msgIn)
                print("SetROI MSG: Facing Waypoint")
                latDeg = data[3] / 1e7
                lonDeg = data[4] / 1e7
                msgAlt = data[5] / 1e3
                Fly2Wp(latDeg, lonDeg, msgAlt, data[6] / 100)
                SetRoi(latDeg, lonDeg, msgAlt)

            # return gimbal to pilot command
            if (msgId == rcGimbal_Hdr and allowGsControl):
                msgFormat = '<2H'
                data = struct.unpack(msgFormat, msgIn)
                print("SetGimbalRC MSG: Sending...")
                if (data[1] == msg_confirm):  # verify command with second short
                    SetGimbalRc()

            # tilt camera command
            if (msgId == tiltCam_Hdr and allowGsControl):
                msgFormat = '<HHddd'
                data = struct.unpack(msgFormat, msgIn)
                SendTilt(data[3], data[4])
                print(f"TiltCam MSG: {data[3]}, {data[4]}")


        except (KeyboardInterrupt):
            raise
        except:
            e = sys.exc_info()[0]
            # print(e)


def RTL():
    # mode_id=6 # mode RTL
    mode_id = mavConnection.mode_mapping()['RTL']
    print("RTL mode sent")
    mavConnection.mav.command_long_send(mavConnection.target_system, mavConnection.target_component,
                                        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, mode_id, 0, 0, 0, 0, 0)


def Fly2Wp(latDeg, lonDeg, alt_mMSL, speed):
    if normalControl and mavConnection is not None:
        # fly 2 waypoint command
        # set guided mode

        #        mode_id=4 #mode GUIDED
        mode_id = mavConnection.mode_mapping()['GUIDED']
        print("GUIDED mode sent")
        mavConnection.mav.command_long_send(mavConnection.target_system, mavConnection.target_component,
                                            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, mode_id, 0, 0, 0, 0, 0)
        # send to waypoint
        ignoremask = int("fff8", 16)  # use only x,y,z
        # determine if current altitude is maintained
        if (alt_mMSL < 0):
            alt_mMSL = alt_mmMSL / 1e3
        print(f"Lat: {latDeg}, Lon: {lonDeg}")
        mavConnection.mav.set_position_target_global_int_send(
            0,  # system time in ms
            mavConnection.target_system,  # target system
            mavConnection.target_component,  # target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            ignoremask,  # ignore
            int(latDeg * 1e7),  # latitude_degE7
            int(lonDeg * 1e7),  # longitude deg_E7
            alt_mMSL,  # altitude m-MSL
            0, 0, 0,  # velocity
            0, 0, 0,  # accel x,y,z
            0, 0)  # yaw, yaw rate

        # set ground speed
        if (speed < 0):
            speed = -1  # no change
        mavConnection.mav.command_long_send(mavConnection.target_system, mavConnection.target_component,
                                            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                                            1,  # ground speed
                                            speed,  # speed in m/s
                                            -1,  # no change to throttle
                                            0,  # absolute speed
                                            0, 0, 0)


def SetGimbalRc():
    if normalControl:
        # reset tilt control
        mavConnection.mav.command_long_send(mavConnection.target_system, mavConnection.target_component,
                                            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL, 0,
                                            0,  # tilt in centiDeg
                                            0,  # roll
                                            0,  # pan
                                            0, 0, 0,
                                            mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
        # clear ROI command
        mavConnection.mav.command_long_send(mavConnection.target_system, mavConnection.target_component,
                                            mavutil.mavlink.MAV_CMD_DO_SET_ROI, 0,
                                            0,  # ROI mode
                                            0,  # WP index
                                            0,  # ROI index
                                            0,  # empty
                                            0,  # latitude
                                            0,  # longitude
                                            0)  # alt cm-MSL


def SetRoi(lat_deg, lon_deg, alt_mMSL):
    if normalControl:
        # set ROI command
        # note this command's altitude is relative to home position
        mavConnection.mav.command_long_send(mavConnection.target_system, mavConnection.target_component,
                                            mavutil.mavlink.MAV_CMD_DO_SET_ROI, 0,
                                            0,  # ROI mode
                                            0,  # WP index
                                            0,  # ROI index
                                            0,  # empty
                                            lat_deg,  # latitude
                                            lon_deg,  # longitude
                                            alt_mMSL - home_alt_mMSL)  # alt m-MSL
    print(lat_deg, lon_deg, alt_mMSL)


def RequestHomePosition():
    global home_alt_mMSL
    global home_lat_deg
    global home_lon_deg
    # request home position
    mavConnection.mav.command_long_send(mavConnection.target_system, mavConnection.target_component,
                                        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0,
                                        0, 0, 0, 0, 0, 0, 0)
    # wait to read home position
    msg = mavConnection.recv_match(type='HOME_POSITION',
                                   blocking=True, timeout=None)
    # store in global variables
    home_alt_mMSL = msg.altitude / 1e3
    home_lat_deg = msg.latitude / 1e7
    home_lon_deg = msg.longitude / 1e7


def SendTilt(tilt, yaw):
    # add threshold for F450 range
    if normalControl:
        # set tilt
        if (tilt > 45):
            tilt = 45
        if (tilt < -90):
            tilt = -90

        mavConnection.mav.command_long_send(mavConnection.target_system, mavConnection.target_component,
                                            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL, 0,
                                            tilt * 100,  # tilt in centiDeg
                                            0,  # roll
                                            0,  # pan
                                            0, 0, 0,
                                            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)
        # set vehicle yaw
        if (yaw != -1):  # don't change if -1
            yaw = yaw % 360  # ensure pan angle is in range
            # determine clockwise or counterclockwise yaw movement
            direction = np.sign(180 - (yaw - currentHeading_deg) % 360)
            if direction == 0:  # case on the 180 deg marks
                direction = 1
            mavConnection.mav.command_long_send(mavConnection.target_system, mavConnection.target_component,
                                                mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
                                                yaw,  # yaw angle [0-360]
                                                0,  # yaw change deg per second
                                                direction,  # direction of travel (-1:counterclockwise, 1:clockwise)
                                                0,  # absolute angle (0=north)
                                                0, 0, 0)


#        print('Tilt: '+str(tilt)+', Yaw: '+str(yaw)+', Curr Yaw: '+str(currentHeading_deg)+',direction:'+str(direction))

def closeMavlink():
    try:
        server_socket.close()
        client_socket.close()
    except:
        e = sys.exc_info()[0]


def InitMavlink(verbose):
    global home_alt_mMSL
    global home_lat_deg
    global home_lon_deg
    home_alt_mMSL = 0
    home_lat_deg = 0
    home_lon_deg = 0

    global normalControl  # used to control whether RPi or GS can control autopilot.  Gives control exclusively to pilot.
    normalControl = True

    global allowGsControl  # used so RPi can decide if GS messages get through to autopilot
    allowGsControl = True

    portNotConnected = True

    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        print(p)
        # autopilot can have multiple descriptions depending on pilot and connection
        if ('fmu' in p.description or 'CubeBlack' in p.description
                or 'STM' in p.description or 'CUBE' in p.description
                or 'ArduPilot' in p.description):
            if verbose:
                print(p)
            s = serial.Serial(p.device)
            portNotConnected = False

    if portNotConnected:
        if verbose:
            print('No Autopilot.  Check connection.')
    if verbose:
        print("Making mavLink connection")
    global mavConnection
    # s.close()  # needed to work on Windows python tests
    mavConnection = mavutil.mavlink_connection(s.port, baud=115200)

    if verbose:
        print("waiting for heartbeat")
    wait_heartbeat()  # establish connection with UAV

    # get home position for reference altitude
    if verbose:
        print("getting home position")
    # RequestHomePosition()
    if verbose:
        print('Home: lat: ' + str(home_lat_deg) + ', lon: ' + str(home_lon_deg) + ', alt: ' + str(home_alt_mMSL))

    if verbose:
        print("Sending stream request")  # these are the autopilot streams with data we use

    for i in range(0, 3):
        mavConnection.mav.request_data_stream_send(mavConnection.target_system,
                                                   mavConnection.target_component,
                                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 0, 1)
        mavConnection.mav.request_data_stream_send(mavConnection.target_system,
                                                   mavConnection.target_component,
                                                   mavutil.mavlink.MAV_DATA_STREAM_POSITION, 2, 1)
        mavConnection.mav.request_data_stream_send(mavConnection.target_system,
                                                   mavConnection.target_component,
                                                   mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1)
        mavConnection.mav.request_data_stream_send(mavConnection.target_system,
                                                   mavConnection.target_component,
                                                   mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 2, 1)

    try:
        if verbose:
            print("setting up server")
        global server_socket
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        if verbose:
            print("binding server")
        server_socket.bind(('', udpServerSocketPort))

        if verbose:
            print("setting up client")
        global client_socket
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        if verbose:
            print("starting server thread")
        threading.Thread(target=readUdpAndSendMavlinkCommand, args=(server_socket,)).start()

        if verbose:
            print("starting client thread")
        threading.Thread(target=readMavlinkAndForwardUdp, args=(client_socket,)).start()


    except:
        e = sys.exc_info()[0]
        if verbose:
            print(e)


