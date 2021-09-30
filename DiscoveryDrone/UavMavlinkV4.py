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

udpServerSocketPort=45454
udpRecvSocketPort=50505
bufferSize=1024

global currentLat_Deg
currentLat_Deg=0
global currentLon_Deg
currentLon_Deg=0
global currentAlt_mMSL
currentAlt_mMSL=0

    
def wait_heartbeat():
#    print("Waiting for heartbeat")
    mavConnection.wait_heartbeat()
    
def readMavlinkAndForwardUdp(linkdown):
    # set up message read and writes
    # define send variables

    global timestamp
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
    # set up local globals
    global currentLat_Deg
    currentLat_Deg=0
    global currentLon_Deg
    currentLon_Deg=0
    global currentAlt_mMSL
    currentAlt_mMSL=0
    global currentVelX_mps
    currentVelX_mps=0
    global currentVelY_mps
    currentVelY_mps=0
    global currentVelZ_mps
    currentVelZ_mps=0
    global currentHeading_deg
    currentHeading_deg=0
    global normalControl
    
    sendFormat='<HHdlllhhhH'
    sendHeader=int("ac03",16)  
    while True:
        # send messages
        try:
            msg = mavConnection.recv_match()
            if not msg:
                continue
            
            if msg.get_type() == 'SYSTEM_TIME':
                baseTime = msg.time_unix_usec / 1e6 - msg.time_boot_ms / 1e3
    
            
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                timestamp=msg.time_boot_ms/1e3+baseTime
                lat_degE7=msg.lat
                lon_degE7=msg.lon
                alt_mmMSL=msg.alt
                velX_cmps=msg.vx
                velY_cmps=msg.vy
                velZ_cmps=msg.vz
                heading_cdeg=msg.hdg
                currentLat_Deg=lat_degE7/1e7
                currentLon_Deg=lon_degE7/1e7
                currentAlt_mMSL=alt_mmMSL/1e3
                currentVelX_mps=velX_cmps/1e2
                currentVelY_mps=velY_cmps/1e2
                currentVelZ_mps=velZ_cmps/1e2
                currentHeading_deg=heading_cdeg/1e2
                
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

            if msg.get_type() == 'HEARTBEAT':
                if ((msg.custom_mode==mavConnection.mode_mapping()['RTL']) or 
                    (msg.custom_mode==mavConnection.mode_mapping()['LAND'])):
                    print('Entered safe RTL mode or land')
                    normalControl=False
                
            sleep(.1)

        except (KeyboardInterrupt):
            raise
        except:
            e=sys.exc_info()[0]
            print(e)
    
def readUdpAndSendMavlinkCommand(uplink):

    fly2Wp_Hdr=int("abf2",16)
    flyRTL_Hdr=int("ace1",16)
    msg_confirm=int("d478",16)
    setRoi_Hdr=int("add0",16)
    tiltCam_Hdr=int("aecf",16)
    rcGimbal_Hdr=int("afbe",16)
    
    # the onboard Raspberry Pi may refuse GS control for all commands except RTL
    while True:
        try:
            msgIn,ipSource=uplink.recvfrom(bufferSize)
            temp=msgIn[:2] # separate header to test message
            msgId=struct.unpack('<H',temp)[0]
            # fly 2 waypoint command
            if(msgId==fly2Wp_Hdr and allowGsControl):
                msgFormat='<HHdlllh'
                data=struct.unpack(msgFormat,msgIn)
                Fly2Wp(data[3]/1e7,data[4]/1e7,data[5]/1e3,data[6]/100)
                print(data[3]/1e7)
                print(data[4]/1e7)
                
                
            #RTL command
            if(msgId==flyRTL_Hdr):
                msgFormat='<2H'
                data=struct.unpack(msgFormat,msgIn)
                if(data[1]==msg_confirm):   #verify command with second short
                    print("Return to Home")
                    RTL()

            # set ROI command
            if(msgId==setRoi_Hdr and allowGsControl):
                msgFormat='<HHdlll'
                data=struct.unpack(msgFormat,msgIn)
                latDeg=data[3]/1e7
                lonDeg=data[4]/1e7
                msgAlt=data[5]/1e3
                SetRoi(latDeg,lonDeg,msgAlt)
                
            #return gimbal to pilot command
            if(msgId==rcGimbal_Hdr and allowGsControl):
                msgFormat='<2H'
                data=struct.unpack(msgFormat,msgIn)
                if(data[1]==msg_confirm):   #verify command with second short
                    SetGimbalRc()

            # tilt camera command
            if(msgId==tiltCam_Hdr and allowGsControl):
                msgFormat='<HHdd'
                data=struct.unpack(msgFormat,msgIn)
                SendTilt(data[3])

                
        except (KeyboardInterrupt):
            raise
        except:
            e=sys.exc_info()[0]
            print(e)
    
def RTL():
    #mode_id=6 # mode RTL
    mode_id=mavConnection.mode_mapping()['RTL']
    mavConnection.mav.command_long_send(mavConnection.target_system,mavConnection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,0,1,mode_id,0,0,0,0,0)


def Fly2Wp(latDeg,lonDeg,alt_mMSL,speed):
    if normalControl:

        # fly 2 waypoint command
        # set guided mode
    
#        mode_id=4 #mode GUIDED
        mode_id=mavConnection.mode_mapping()['GUIDED']
        
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

def SetGimbalRc():
    if normalControl:
        # reset tilt control
        mavConnection.mav.command_long_send(mavConnection.target_system,mavConnection.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,0,
            0, #tilt in centiDeg
            0, #roll
            0, #pan
            0, 0, 0,
            mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
        # clear ROI command
        mavConnection.mav.command_long_send(mavConnection.target_system,mavConnection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_ROI,0,
            0, # ROI mode
            0, # WP index
            0, # ROI index
            0, # empty
            0, # latitude
            0, # longitude
            0) # alt cm-MSL 


def SetRoi(lat_deg,lon_deg,alt_mMSL):
    if normalControl:
        # set ROI command
        # note this command's altitude is relative to home position
        mavConnection.mav.command_long_send(mavConnection.target_system,mavConnection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_ROI,0,
            0, # ROI mode
            0, # WP index
            0, # ROI index
            0, # empty
            lat_deg, # latitude
            lon_deg, # longitude
            alt_mMSL-home_alt_mMSL) # alt m-MSL 
                
def RequestHomePosition():
    global home_alt_mMSL
    global home_lat_deg
    global home_lon_deg
    # request home position
    mavConnection.mav.command_long_send(mavConnection.target_system,mavConnection.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,0,
        0,0,0,0,0,0,0)
    # wait to read home position
    msg = mavConnection.recv_match(type='HOME_POSITION', 
        blocking=True,timeout=None)
    # store in global variables
    home_alt_mMSL=msg.altitude/1e3
    home_lat_deg=msg.latitude/1e7
    home_lon_deg=msg.longitude/1e7
    

def SendTilt(angle, yaw):
    # add threshold for F450 range
    if normalControl:
        if(angle>45):
            angle=45
        if(angle<-90):
            angle=-90
            
        mavConnection.mav.command_long_send(mavConnection.target_system,mavConnection.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,0,
            angle*100, #tilt in centiDeg
            0, #roll
            yaw*100, #pan
            0, 0, 0,
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)
    
def closeMavlink():
    try:
        server_socket.close()
        client_socket.close()    
    except:
        e=sys.exc_info()[0]

def InitMavlink(verbose):
    
    global normalControl  # used to control whether RPi or GS can control autopilot.  Gives control exclusively to pilot.
    normalControl=True
    
    global allowGsControl # used so RPi can decide if GS messages get through to autopilot
    allowGsControl=True
    
    portNotConnected=True
    
    ports=list(serial.tools.list_ports.comports())
    for p in ports:
        # autopilot can have multiple descriptions depending on pilot and connection
        if ('fmu' in p.description or 'CubeBlack' in p.description
            or 'STM' in p.description or 'CUBE' in p.description
            or 'ArduPilot' in p.description):
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
    s.close()  # needed to work on Windows python tests
    mavConnection=mavutil.mavlink_connection(s.port,baud=115200)

    if verbose:
        print("waiting for heartbeat")
    wait_heartbeat() # establish connection with UAV
    
    # get home position for reference altitude
    #if verbose:
    #    print("getting home position")
    #RequestHomePosition()
    #if verbose:
    #    print('Home: lat: '+str(home_lat_deg)+', lon: '+str(home_lon_deg)+', alt: '+str(home_alt_mMSL))
 
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

    #try:
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

        
    #except:
    e=sys.exc_info()[0]
    if verbose:
        print(e)

    
