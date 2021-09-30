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

udpServerSocketPort=45454
udpRecvSocketPort=50505
bufferSize=1024
    
def wait_heartbeat():
#    print("Waiting for heartbeat")
    mavConnection.wait_heartbeat()
#    print("Heartbeat from Pixhawk (system %u component %u type %u)" % 
#          (mavConnection.target_system,mavConnection.target_component,mavConnection.mav_type))
    
def show_messages():
    while True:
        msg=mavConnection.recv_match(blocking=True)
        if not msg:
            return
        if msg.get_type() == "BAD DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()
        else:
            print(msg)

def readMavlinkAndForwardUdp(linkdown):
    # set up message read and writes
    # define send variables
    timestamp=0
    lat_degE7=0
    lon_degE7=0
    global alt_mmMSL
    alt_mmMSL=0
    velX_cmps=0
    velY_cmps=0
    velZ_cmps=0
    heading_cdeg=0
    baseTime=0  # track utm time relative to autopilot start time
    
    sendFormat='<HHdlllhhhH'
    sendHeader=int("ac03",16)  
    while True:
        # send messages
        try:
            msg = mavConnection.recv_match()
            if not msg:
                continue
            #print(msgPixIn.get_type())
            
            if msg.get_type() == 'SYSTEM_TIME':
            #msg=mavConnection.recv_match(type='SYSTEM_TIME',blocking=True)
                baseTime = msg.time_unix_usec / 1e6 - msg.time_boot_ms / 1e3
    
            
            #msg=mavConnection.recv_match(type='GLOBAL_POSITION_INT',blocking=True)
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                timestamp=msg.time_boot_ms/1e3+baseTime
                lat_degE7=msg.lat
                lon_degE7=msg.lon
                alt_mmMSL=msg.alt
                velX_cmps=msg.vx
                velY_cmps=msg.vy
                velZ_cmps=msg.vz
                heading_cdeg=msg.hdg
                
                msgLeng=32
                msgOut=[sendHeader]
                msgOut.append(np.uint16(msgLeng))
                msgOut.append(timestamp)
                msgOut.append(np.int32(lat_degE7))
                msgOut.append(np.int32(lon_degE7))
                msgOut.append(np.int32(alt_mmMSL))
                msgOut.append(np.int16(velX_cmps))
                msgOut.append(np.int16(velY_cmps))
                msgOut.append(np.int16(velZ_cmps))
                msgOut.append(np.uint16(heading_cdeg))
                linkdown.sendto(struct.pack(sendFormat,*msgOut),("192.168.1.255",udpRecvSocketPort))
                #print("global_position_int message (altitude %u timestamp %f)" % (alt_mmMSL,timestamp))
        except (KeyboardInterrupt):
            raise
        except:
            e=sys.exc_info()[0]
            print(e)
    
def readUdpAndSendMavlinkCommand(uplink):
    # test sending guided control messages
    fly2Wp_Hdr=int("abf2",16)
    flyRTL_Hdr=int("ace1",16)
    flyRTL_confirm=int("d478",16)
    setRoi_Hdr=int("add0",16)
    tiltCam_Hdr=int("aecf",16)

    while True:
        try:
#            print("waiting to receive msg")
            msgIn,ipSource=uplink.recvfrom(bufferSize)
#            print(msgIn)
            temp=msgIn[:2] # separate header to test message
            msgId=struct.unpack('<H',temp)[0]

            # fly 2 waypoint command
            if(msgId==fly2Wp_Hdr):
                msgFormat='<HHdlllh'
                data=struct.unpack(msgFormat,msgIn)
                # set guided mode
                Fly2Wp(data[3]/1e7,data[4]/1e7,data[5]/1e3,data[6]/100)
                
            #RTL command
            if(msgId==flyRTL_Hdr):
                msgFormat='<2H'
                data=struct.unpack(msgFormat,msgIn)
                if(data[1]==flyRTL_confirm):   #verify command with second short
                    RTL()

            # set ROI command
            if(msgId==setRoi_Hdr):
                msgFormat='<HHdlll'
                data=struct.unpack(msgFormat,msgIn)
                latDeg=data[3]/1e7
                lonDeg=data[4]/1e7
                msgAlt=data[5]/1e3
                SetRoi(latDeg,lonDeg,msgAlt)
                
            # tilt camera command
            if(msgId==tiltCam_Hdr):
                msgFormat='<HHdd'
                data=struct.unpack(msgFormat,msgIn)

                SendTilt(data[3])

                
        except (KeyboardInterrupt):
            raise
        except:
            e=sys.exc_info()[0]
            print(e)
    
def RTL():
    mode_id=6 # mode RTL
    #print(mavConnection.target_system,mavConnection.target_component,mode_id,mavutil.mavlink.MAV_CMD_DO_SET_MODE)
    mavConnection.mav.command_long_send(mavConnection.target_system,mavConnection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,0,1,mode_id,0,0,0,0,0)


def Fly2Wp(latDeg,lonDeg,alt_mMSL,speed):
    # fly 2 waypoint command
    # set guided mode

    mode_id=4 #mode GUIDED
    
    #print(m.target_system,m.target_component,mode_id,mavutil.mavlink.MAV_CMD_DO_SET_MODE)
    mavConnection.mav.command_long_send(mavConnection.target_system,mavConnection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,0,1,mode_id,0,0,0,0,0)
    # send to waypoint
    ignoremask=int("fff8",16) # use only x,y,z
    # determine if current altitude is maintained
    if(alt_mMSL<0):
        alt_mMSL=alt_mmMSL/1e3
    
    mavConnection.mav.set_position_target_global_int_send(
        0,  # system time in ms
        mavConnection.target_system,  # target system
        mavConnection.target_component,  # target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        ignoremask, # ignore
        int(latDeg*1e7), #latitude_degE7
        int(lonDeg*1e7), #longitude deg_E7
        alt_mMSL, #altitude m-MSL
        0, 0, 0, # velocity
        0, 0, 0, # accel x,y,z
        0, 0) # yaw, yaw rate

    # set ground speed
    if(speed<0):
        speed=-1 #no change
    mavConnection.mav.command_long_send(mavConnection.target_system,mavConnection.target_component,
                                        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,0,
                                        1, # ground speed
                                        speed, # speed in m/s
                                        -1, #no change to throttle
                                        0, # absolute speed
                                        0,0,0)


def SetRoi(lat_deg,lon_deg,alt_mMSL):
                # set ROI command
    mavConnection.mav.command_long_send(mavConnection.target_system,mavConnection.target_component,
                                        mavutil.mavlink.MAV_CMD_DO_SET_ROI,0,
                                        0, # ROI mode
                                        0, # WP index
                                        0, # ROI index
                                        0, # empty
                                        int(lat_deg*1e7), # latitude deg_E7
                                        int(lon_deg*1e7), # longitude deg_E7
                                        alt_mMSL) # alt m-MSL 
    #print("Set Roi (lat %f, lon %f, alt %f)" % (lat_deg,lon_deg,alt_mMSL))
                


def SendTilt(angle):
    if(angle>45):
        angle=45
    if(angle<-90):
        angle=-90
        
    mavConnection.mav.command_long_send(
        mavConnection.target_system,
        mavConnection.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        1,
        angle, #tilt
        0, #roll
        0, #pan
        0, 0, 0,
        mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)
    
    #print("tilt angle sent (angle %f)" % (angle))

def closeMavlink():
    server_socket.close()
    client_socket.close()    

def InitMavlink(verbose):
    portNotConnected=True
    
    ports=list(serial.tools.list_ports.comports())
    for p in ports:
#        if 'CubeBlack' in p.description:
#        if 'STM' in p.description:
        if 'fmu' in p.description or 'CubeBlack' in p.description:
 #       if 'CUBE' in p.description:
            if verbose:
                print(p)
            s = serial.Serial(p.device)
            portNotConnected=False
            
    if portNotConnected:
        if verbose:
            print('No Autopilot.  Check connection.')
    if verbose:
        print("Making mavLink connection")
    global mavConnection    
    mavConnection=mavutil.mavlink_connection(s.port,baud=115200)

    if verbose:
        print("waiting for heartbeat")
    wait_heartbeat() # establish connection with UAV
    if verbose:
        print("Sending stream request")  # these are the autopilot streams with data we use
    
    for i in range(0,3):
        mavConnection.mav.request_data_stream_send(mavConnection.target_system,
                                                   mavConnection.target_component,
                                                   mavutil.mavlink.MAV_DATA_STREAM_ALL,0,1)
        mavConnection.mav.request_data_stream_send(mavConnection.target_system,
                                                   mavConnection.target_component,
                                                   mavutil.mavlink.MAV_DATA_STREAM_POSITION,2,1)
        mavConnection.mav.request_data_stream_send(mavConnection.target_system,
                                                   mavConnection.target_component,
                                                   mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,2,1)
        mavConnection.mav.request_data_stream_send(mavConnection.target_system,
                                                   mavConnection.target_component,
                                                   mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,2,1)

    try:
        if verbose:
            print("setting up server")
        global server_socket
        server_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        
        if verbose:
            print("binding server")
        server_socket.bind(('',udpServerSocketPort))

        if verbose:
            print("setting up client")
        global client_socket
        client_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST,1)

        if verbose:
            print("starting server thread")
        threading.Thread(target=readUdpAndSendMavlinkCommand,args=(server_socket,)).start()

        if verbose:
            print("starting client thread")
        threading.Thread(target=readMavlinkAndForwardUdp,args=(client_socket,)).start()
    except:
        e=sys.exc_info()[0]
        if verbose:
            print(e)
    
#    return(mavConnection)
    
