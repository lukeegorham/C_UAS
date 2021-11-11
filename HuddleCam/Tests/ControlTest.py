#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 21 12:41:31 2020

reference:
    https://www.pyimagesearch.com/2019/04/15/live-video-streaming-over-network-with-opencv-and-imagezmq/

@author: pi
"""

#import cv2
##import numpy as np
import socket
import imutils
import sys
import numpy as np
import struct
import datetime
import pytz

import cv2, queue, threading, time
from threading import Thread, Event

# initialize the ImageSender object with the socket address of the
# server

# import the necessary packages
from datetime import datetime
import numpy as np
import socket
import argparse
import imutils
import cv2
import sys
import struct
import serial
import math
from serial import Serial

cam = Serial('/dev/ttyUSB0',9600) # worked on GPU
cam.close
HOST='';
PORT=46555;

ZoomValue = 0


class CtrlC2(Thread):
  def __init__(self, event):
    Thread.__init__(self)
    self.stopped = event

  def run(self):
    while (not self.stopped.is_set()):
      Ctrl()

def CameraAngleUtm (tx, ty, telv, cx, cy ,celv, cdirection):
    xdist = tx-cx
    ydist = ty-cy
    zdist = telv-celv
    hdeg = math.degrees(math.atan2(xdist, ydist))-cdirection
    print(hdeg)
    hdistance = math.sqrt(xdist**2 + ydist**2)
    vdeg = math.degrees(math.atan2(zdist, hdistance))
    totalDistance = math.sqrt(zdist**2 + hdistance**2)
    return [hdeg, vdeg, totalDistance]

def HuddleCamPos (cam, panAngDeg, tiltAngDeg, panSpeed, tiltSpeed):
    panAngPerClick = 0.075
    tiltAngPerClick = 0.079
    tiltRange = [-33.5, 84]
    #tiltAngDeg = math.fmod(tiltAngDeg+180, 360) - 180
    if tiltAngDeg > tiltRange[1]:
        tiltAngDeg = tiltRange[1]
    if tiltAngDeg < tiltRange[0]:
        tiltAngDeg = tiltRange[0]

    #print(tiltAngDeg)
    panRange = [-171, 184]
    panAngDeg = math.fmod(panAngDeg + 180, 360) - 180
    if (panAngDeg < panRange[0]) and (panAngDeg > (-360 + panRange[1])):
        panAngDeg = panRange[0]
    if panAngDeg <= (-360 + panRange[1]):
        panAngDeg = panAngDeg + 360
    panClick = round(panAngDeg / panAngPerClick)
    panHex = str(hex(int(panClick)))

    if panHex[0] == "-":
        panHex = int(panHex[3:],16)
        binaryPanHex = bin(int(panHex))
        numZerosNeeded = 14 - len(binaryPanHex)
        newPanBin = "0b"
        for i in range(numZerosNeeded):
            newPanBin += '1'
        for i in range(2,len(binaryPanHex)):
            if binaryPanHex[i] == "0":
                newPanBin+= '1'
            else:
                newPanBin += '0'
        newPanHex = hex(int(newPanBin,2) + 1)
        newPanHex = newPanHex[2:]
        while len(newPanHex) < 4:
            newPanHex = 'f' + newPanHex
    else:
        panHex = panHex[2:]
        while len(panHex) < 4:
            panHex = '0' + panHex
        newPanHex = panHex

    tiltClick = round(tiltAngDeg / tiltAngPerClick)
    tiltHex = str(hex(int(tiltClick)))
    if tiltHex[0] == "-":
        tiltHex = int(tiltHex[3:],16)
        binaryTiltHex = bin(int(tiltHex))
        numZerosNeeded = 14 - len(binaryTiltHex)
        newTiltBin = "0b"
        for i in range(numZerosNeeded):
            newTiltBin += '1'
        for i in range(2,len(binaryTiltHex)):
            if binaryTiltHex[i] == "0":
                newTiltBin+= '1'
            else:
                newTiltBin += '0'
        newTiltHex = hex(int(newTiltBin,2) + 1)
        newTiltHex = newTiltHex[2:]
        while len(newTiltHex) < 4:
            newTiltHex = 'f' + newTiltHex
        tiltHex = newTiltHex
    else:
        tiltHex = tiltHex[2:]
        while len(tiltHex) < 4:
            tiltHex = '0' + tiltHex

    cmd = [129, 1, 6, 2, panSpeed, tiltSpeed,
        int(newPanHex[0],16), int(newPanHex[1],16),
        int(newPanHex[2],16), int(newPanHex[3],16),
        int(tiltHex[0],16), int(tiltHex[1],16),
        int(tiltHex[2],16), int(tiltHex[3],16), 255]

    #cmd = [129,1,6,2,24,20,15,11,5,0,15,14,11,0,255]
    print("cmd is: " + str(cmd))
    cam.open
    cam.write(cmd)
    cam.close

def HuddleCamZoom(cam,zoomValue):
    baseCnt = 36
    levels = [1, 334/baseCnt]
    if (zoomValue > levels[1]): zoomValue = levels[0]
    pVal = [4.343211148793062e-08, -4.984554264045150e-05, 0.021857575650121, -4.553447194454285, 4.856833032085256e+02, -1.252202320377412e+04]
    zoomLev = round(np.polyval(pVal, zoomValue * baseCnt))
    print("zoomLev: ", zoomLev)
    cmdList = [129, 1, 4, 6, 3, 255]

    cmd = bytes(cmdList)

    cam.open
    cam.write(cmd)
    cam.close

    zoomHex = str(hex(int(zoomLev)))
    if (zoomHex[0] == "-"):
        zoomHex = zoomHex[3:]
    else:
        zoomHex = zoomHex[2:]  ## Might throw off zoom with negatives???

    while len(zoomHex) < 4:
        zoomHex = '0' + zoomHex
    cmd = [129, 1, 4, 71,int(zoomHex[0],16), int(zoomHex[1],16), int(zoomHex[2],16), int(zoomHex[3],16), 255]

    cam.open
    cam.write(cmd)
    cam.close

def control(tx, ty, telv, cx, cy ,celv, cdirection):
    [hdeg, vdeg, totalDistance] = CameraAngleUtm(tx, ty, telv, cx, cy ,celv, cdirection)
    print("CameraAngleUtm done")
    panAngDeg=-hdeg
    tiltAngDeg=vdeg
    panSpeed=24
    tiltSpeed=20
    HuddleCamPos(cam,panAngDeg,tiltAngDeg,panSpeed,tiltSpeed)
    print("HuddleCamPos done")

    if totalDistance > 150:
        zoomfactor = 9
    elif totalDistance > 130:
        zoomfactor = 8
    elif totalDistance > 110:
        zoomfactor = 7
    elif totalDistance > 90:
        zoomfactor = 6
    elif totalDistance > 70:
        zoomfactor = 5
    elif totalDistance > 50:
        zoomfactor = 4
    elif totalDistance > 40:
        zoomfactor = 3
    elif totalDistance > 20:
        zoomfactor = 2
    else:
        zoomfactor = 1

    HuddleCamZoom(cam,zoomfactor)
    print("HuddleCamZoom done - zoomfactor: " + str(zoomfactor))


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
    print("no bind")
    sys.exit()

def main():
  stopFlag = Event()
  Ctrl = CtrlC2(stopFlag)
  Ctrl.start()

 # while True:
  #stopFlag.set()


def Ctrl():
    global ZoomValue
        # receive RPi name and frame from the RPi and acknowledge
        # the receipt
    msgIn, (address,port) = receiver.recvfrom(65536)
    MsgHeadr=int("1e91",16) #unique message header
    metaFormat='<HHdiiiddddddddddd'
        # verify correct message
    temp=msgIn[0:2]
    recvHdr=struct.unpack('<H', temp)[0]
    #print(MsgHeadr,recvHdr)
    if(recvHdr==MsgHeadr):
            # get metadata size
        temp=msgIn[2:4]
        metaLen=struct.unpack('<H',temp)[0]
            # get metadata
        temp=msgIn[0:metaLen]
        metadata=struct.unpack(metaFormat,temp)

        timestamp=metadata[2]
        toggle=metadata[16]
        targx = metadata[3]
        targy = metadata[4]
        targz = metadata[5]
        if(targx == 0):
               print('Not receiving enemy location')
    #print(toggle)
    camx = int(39.008917 * 1e7)
    camy = int(-104.88299 * 1e7)
    camz = int(2163 * 10)
    if(toggle==0):
        control(targx, targy, targz, camx, camy, camz, 0)
    if(toggle==1):
        if (metadata[6] == 1):
                print("Up");

                cmd = [129,1,6,1,12,12,3,1,255]
                print("cmd is: " + str(cmd))
                cam.open
                cam.write(cmd)
                cam.close
        elif (metadata[8] == 1):
                print("Left");

                cmd = [129,1,6,1,12,12,1,3,255]
                print("cmd is: " + str(cmd))
                cam.open
                cam.write(cmd)
                cam.close
        elif (metadata[7] == 1):
                print("Down");

                cmd = [129,1,6,1,12,12,3,2,255]
                print("cmd is: " + str(cmd))
                cam.open
                cam.write(cmd)
                cam.close
        elif (metadata[9] == 1):
                print("Right");

                cmd = [129,1,6,1,12,12,2,3,255]
                print("cmd is: " + str(cmd))
                cam.open
                cam.write(cmd)
                cam.close
        elif (metadata[13] == 1):
                print("Stop");

                cmd = [129,1,6,1,12,12,3,3,255]
                print("cmd is: " + str(cmd))
                cam.open
                cam.write(cmd)
                cam.close
        elif (metadata[12] == 1):
                print("Out");
                ZoomValue = ZoomValue - 1;
                if (ZoomValue < 0):
                    ZoomValue = 0;
                cmd = [129,1,4,71,ZoomValue,ZoomValue,ZoomValue,ZoomValue,
ZoomValue,ZoomValue,ZoomValue,ZoomValue,255]
                print("cmd is: " + str(cmd))
                cam.open
                cam.write(cmd)
                cam.close
        elif (metadata[11] == 1):
                print("In");
                ZoomValue = ZoomValue + 1;
                if (ZoomValue > 15):
                    ZoomValue = 15;
                cmd = [129,1,4,71,ZoomValue,ZoomValue,ZoomValue,ZoomValue,
ZoomValue,ZoomValue,ZoomValue,ZoomValue,255]
                print("cmd is: " + str(cmd))
                cam.open
                cam.write(cmd)
                cam.close
        elif (metadata[10] == 1):
                print("Home");

                cmd = [129,1,6,4,255]
                print("cmd is: " + str(cmd))
                cam.open
                cam.write(cmd)
                cam.close

if __name__ == '__main__':
  main()