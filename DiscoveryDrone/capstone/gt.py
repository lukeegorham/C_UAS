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

def main():
    InitMavlink(True)
    print("Hello World")
    # RTL()
    Fly2Wp(39.0,-114.0,2200,10)
    
def wait_heartbeat():
#    print("Waiting for heartbeat")
    mavConnection.wait_heartbeat()
    
    
def RTL():
    #mode_id=6 # mode RTL
    mode_id=mavConnection.mode_mapping()['RTL']
    print("RTL mode sent")
    mavConnection.mav.command_long_send(mavConnection.target_system,mavConnection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,0,1,mode_id,0,0,0,0,0)


def Fly2Wp(latDeg,lonDeg,alt_mMSL,speed):
    if normalControl and mavConnection is not None:
        # fly 2 waypoint command
        # set guided mode
    
#        mode_id=4 #mode GUIDED
        mode_id=mavConnection.mode_mapping()['GUIDED']
        print("GUIDED mode sent")
        mavConnection.mav.command_long_send(mavConnection.target_system,mavConnection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, mode_id, 0, 0, 0, 0, 0)
        # send to waypoint
        ignoremask=int("fff8",16) # use only x,y,z
        # determine if current altitude is maintained
        if(alt_mMSL<0):
            alt_mMSL=alt_mMSL/1e3
            # alt_mMSL=alt_mmMSL/1e3

        # print(f"Lat: {latDeg}, Lon: {lonDeg}")
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
    
        # # set ground speed
        # if(speed<0):
        #     speed=-1 #no change
        # mavConnection.mav.command_long_send(mavConnection.target_system,mavConnection.target_component,
        #                                     mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,0,
        #                                     1, # ground speed
        #                                     speed, # speed in m/s
        #                                     -1, #no change to throttle
        #                                     0, # absolute speed
        #                                     0,0,0)


def closeMavlink():
    try:
        server_socket.close()
        client_socket.close()    
    except:
        e=sys.exc_info()[0]

def InitMavlink(verbose):
    
    global home_alt_mMSL
    global home_lat_deg
    global home_lon_deg
    home_alt_mMSL=0
    home_lat_deg=0
    home_lon_deg=0

    global normalControl  # used to control whether RPi or GS can control autopilot.  Gives control exclusively to pilot.
    normalControl=True
    
    global allowGsControl # used so RPi can decide if GS messages get through to autopilot
    allowGsControl=True
    
    portNotConnected=True
    
    ports=list(serial.tools.list_ports.comports())
    for p in ports:
        print(p)
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
    # s.close()  # needed to work on Windows python tests
    mavConnection=mavutil.mavlink_connection(s.port,baud=115200)

    if verbose:
        print("waiting for heartbeat")
    wait_heartbeat() # establish connection with UAV
    
    # get home position for reference altitude
    if verbose:
        print("getting home position")
    #RequestHomePosition()
    if verbose:
        print('Home: lat: '+str(home_lat_deg)+', lon: '+str(home_lon_deg)+', alt: '+str(home_alt_mMSL))
 
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


if __name__ == "__main__":
    main()