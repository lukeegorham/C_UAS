#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 21 12:41:31 2020

reference:
    https://www.pyimagesearch.com/2019/04/15/live-video-streaming-over-network-with-opencv-and-imagezmq/

@author: pi
"""

import socket
import imutils
import sys
import numpy as np
import struct
import datetime
import pytz
from datetime import datetime
import argparse
import struct
import serial
import math
from serial import Serial
import serial.tools.list_ports
import cv2, queue, threading, time
from threading import Thread, Event
from time import sleep
from subprocess import call

# initialize the ImageSender object with the socket address of the
# server
cam = Serial('/dev/ttyUSB0',9600) # worked on GPU
cam.close

cmd = [129, 1, 4, 53, 2, 255] #Set outdoor mode
cam.open
cam.write(cmd)
cam.close

cmd = [129, 1, 4, 57, 10, 255] #Set shutter priority automatic exposure
cam.open
cam.write(cmd)
cam.close

HOST='';
PORT=46555;

hostv='192.168.1.255'
portv=5566

ZoomValue = 0

global latitude_deg
latitude_deg=39.008942
global longitude_deg
longitude_deg=-104.879922
global elevation_m_MSL
elevation_m_MSL=2100.1

global imageProc
imageProc = 1
global imageSave
imageSave = 0

global direction
direction = 180

global followX
followX = 320
global followY
followY = 240

class SndImgC2(Thread):
  def __init__(self, event):
    Thread.__init__(self)
    self.stopped = event

  def run(self):
    while (not self.stopped.is_set()):
      SndImg()


class CtrlC2(Thread):
  def __init__(self, event):
    Thread.__init__(self)
    self.stopped = event

  def run(self):
    while (not self.stopped.is_set()):
      Ctrl()

try:
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sender.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
except socket.error:
	print('Failed to create socket')
	sys.exit()

rpiName=socket.gethostname()

class VideoCapture:

  def __init__(self, name):
    self.cap = cv2.VideoCapture(name)
    self.q = queue.Queue()
    t = Thread(target=self._reader)
    t.daemon = True
    t.start()

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
        except queue.Empty:
          pass
      self.q.put(frame)

  def read(self):
    return self.q.get()

def CameraAngleUtm (tx, ty, telv, cx, cy ,celv, cdirection):
    xdist = tx-cx
    ydist = ty-cy
    zdist = telv-celv
    hdeg = math.degrees(math.atan2(xdist, ydist))-cdirection
    #print(hdeg)
    hdistance = math.sqrt(xdist**2 + ydist**2)
    vdeg = math.degrees(math.atan2(zdist, hdistance))
    totalDistance = math.sqrt(zdist**2 + hdistance**2)
    return [hdeg, vdeg, totalDistance]

def HuddleCamPos (cam, panAngDeg, tiltAngDeg, panSpeed, tiltSpeed):
    panAngPerClick = 0.075
    tiltAngPerClick = 0.079
    #tiltAngPerClick = .6
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
    #print("cmd is: " + str(cmd))
    cam.open
    cam.write(cmd)
    cam.close

def HuddleCamZoom(cam,zoomValue):
    baseCnt = 36
    levels = [1, 334/baseCnt]
    if (zoomValue > levels[1]): zoomValue = levels[0]
    pVal = [4.343211148793062e-08, -4.984554264045150e-05, 0.021857575650121, -4.553447194454285, 4.856833032085256e+02, -1.252202320377412e+04]
    zoomLev = round(np.polyval(pVal, zoomValue * baseCnt))
    #print("zoomLev: ", zoomLev)
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
    #print("CameraAngleUtm done")
    panAngDeg=-hdeg + 25
    tiltAngDeg=vdeg + 20
    panSpeed=24
    tiltSpeed=20
    HuddleCamPos(cam,panAngDeg,tiltAngDeg,panSpeed,tiltSpeed)
    #print("HuddleCamPos done")

    if totalDistance > 150:
        zoomfactor = 2
    elif totalDistance > 130:
        zoomfactor = 2
    elif totalDistance > 110:
        zoomfactor = 2
    elif totalDistance > 90:
        zoomfactor = 2
    elif totalDistance > 70:
        zoomfactor = 2
    elif totalDistance > 50:
        zoomfactor = 2
    elif totalDistance > 40:
        zoomfactor = 1
    elif totalDistance > 20:
        zoomfactor = 1
    else:
        zoomfactor = 1

    HuddleCamZoom(cam,zoomfactor)
    #print("HuddleCamZoom done - zoomfactor: " + str(zoomfactor))


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

jpeg_quality = 70  # 0 to 100, higher is better quality, 95 is cv2 default
cap = VideoCapture(0)
noNewFile=True
msgFormatv='<HHddddddddHddddddHHH'
MsgHeadrv=int("a22e",16)
blueTargets = None
hDopMin = 3

def main():
  stopFlag = Event()
  SndImg = SndImgC2(stopFlag)
  SndImg.start()
  Ctrl = CtrlC2(stopFlag)
  Ctrl.start()
  getGpsData() # set system time and pod location
  print('lat: '+str(latitude_deg)+', lon: '+str(longitude_deg)+', alt: '+str(elevation_m_MSL))
  while True:
        try:
            print()
        except:
            e = sys.exc_info()[0]
            print("Time sync: <p>Error: %s</p> \n" % e)
        time.sleep(100)
def getGpsData():
    portNotConnected=True

    # look at all serial devices, then search for GPS receiver
    ports=list(serial.tools.list_ports.comports())
    for p in ports:
        # find if Global Sat GPS receiver is present
        if ('USB-Serial' in p.description):
            ser = serial.Serial(p.device, baudrate = 4800, timeout = 0.5)
            portNotConnected=False

    if portNotConnected:
        print('No GPS.  Check connection.')
        pass

    print("Receiving GPS data")
    sleep(30.0) # wait for GPS to setup
    global GpsLock
    global GpsTime
    global dateGps
    GpsLock=False
    GpsTime=False
    dateGps=""
    # keep trying GPS until time is received and HDop meets threshold
    while (GpsLock==False):
        try:
            data = ser.readline().decode()
            #print(data)
            parseGPS(data)
        except (KeyboardInterrupt):
            raise
        except:
            e = sys.exc_info()[0]
            print("Time sync: <p>Error: %s</p> \n" % e)
            pass

def parseGPS(data):
    global latitude_deg
    latitude_deg=0.0
    global longitude_deg
    longitude_deg=0.0
    global elevation_m_MSL
    elevation_m_MSL=0.0
    global dateUtc
    global dateGps

    global GpsLock
    global GpsTime

    if data[0:6] == "$GPRMC":
        sdata = data.split(",")
        if sdata[2] == 'V':  # this may occur if its been awhile since the GPS was used and the alamanac needs to be loaded
            print("no satellite data available")
            return
        print("---Parsing GPRMC---"),
        dateGps=("20"+sdata[9][4:6]+"-"+sdata[9][2:4]+"-"+sdata[9][0:2])
        GpsTime=True

    if (data[0:6] == "$GPGGA" and GpsTime==True):
        sdata = data.split(",")
        # verify hDOP within tolerance
        print("hDOP: %s" % sdata[8])
        if(float(sdata[8])<hDopMin):
            print("---Parsing GPGGA - min hDop met---"),
            # set system time to GPS (note that this is UTC time)
            if(dateGps != ""):  # test so system date is only set once
                dateUtc="sudo date --set '"+dateGps+" "+sdata[1][0:2]+":"+sdata[1][2:4]+":"+sdata[1][4:]+"'"
                call(dateUtc, shell=True)
                print(dateUtc)

            temp=int(float(sdata[2])/100.0)  #get degrees
            temp2=(float(sdata[2])-float(temp*100))/60.0  #get decimal degrees
            latitude_deg=float(temp)+temp2
            if(sdata[3]=='S'):  # south is negative
                latitude_deg=-latitude_deg

            temp=int(float(sdata[4])/100.0)  #get degrees
            temp2=(float(sdata[4])-float(temp*100))/60.0  #get decimal degrees
            longitude_deg=float(temp)+temp2
            if(sdata[5]=='W'): # west is negative
                longitude_deg=-longitude_deg

            elevation_m_MSL=float(sdata[9])
            GpsLock=True
    print("GPS lock: %s" % GpsLock)

def SndImg():
    #try:
      global followX
      global followY
      global imageProc
      frame = cap.read()
      if(imageSave == 1):
         cv2.imwrite('GroundCamera_' + str(int(datetime.now(tz=pytz.utc).timestamp())) + '_image_' + '.jpg', frame)
      (B, G, R) = cv2.split(frame)
      zeros = np.zeros(frame.shape[:2], dtype = "uint8")
      blueEmphasis = cv2.merge([B, zeros, zeros])
      blueGray = frame[:,:,0]
      blurred = cv2.GaussianBlur(blueGray, (7, 7), 0)
      (T, bThresh) = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY_INV)
      (cnts, hierarchy) = cv2.findContours(bThresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      if(len(cnts) > 0):
         biggestcnt = cnts[0]
         (bigx, bigy, bigw, bigh) = cv2.boundingRect(biggestcnt)
         big_area = bigw * bigh
         for (i, c) in enumerate(cnts):
             (newx, newy, neww, newh) = cv2.boundingRect(c)
             box_area = neww * newh
             target_area = cv2.contourArea(cnts[i])
             if target_area > big_area:
                 biggestcnt = cnts[i]

         M = cv2.moments(biggestcnt)
         blueTargets = frame.copy()

         cv2.drawContours(blueTargets, cnts, -1, (0, 255, 0), 2)
         (x,y),radius = cv2.minEnclosingCircle(biggestcnt)
         center = (int(x),int(y))
         radius = int(radius)
         cv2.circle(blueTargets,center,radius,(0,0,255),2)
         if(imageSave == 1):
             cv2.imwrite('ProcGroundCamera_' + str(int(datetime.now(tz=pytz.utc).timestamp())) + '_image_' + '.jpg', blueTargets)
         blueTargets=imutils.resize(blueTargets,width=320)
         #print(radius)
         if(radius < 240):
             followX = int(x)
             followY = int(y)


      if((len(cnts) > 0) & (imageProc == 1)):
             ret_code, jpg_buffer = cv2.imencode(
         ".jpg", blueTargets, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
      else:
             ret_code, jpg_buffer = cv2.imencode(
          ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])

      dataEncode=np.array(jpg_buffer)
      str_encode = dataEncode.tobytes()

      metaBasev=[MsgHeadrv]
      metaBasev.append(np.uint16(124)) # metadata size
      metaBasev.append(datetime.now(tz=pytz.utc).timestamp())
      metaBasev.append(int(latitude_deg*1e7))
      metaBasev.append(int(longitude_deg*1e7))
      metaBasev.append(int(elevation_m_MSL*10))
      metaBasev.append(90) #Azimuth (Deg)
      metaBasev.append(0) #Tilt (Deg)
      metaBasev.append(0) #Zoom (%)
      metaBasev.append(320) #Image Size (Pixels)
      metaBasev.append(0) #Blob Present (y or n)
      metaBasev.append(0) #Blob Size (Pixels)
      metaBasev.append(0) #Blob location on screen (x,y)
      metaBasev.append(0) #Object Classification
      metaBasev.append(0) #Object Class. certainty
      metaBasev.append(0) #Reserved Double
      metaBasev.append(0) #Reserved Double
      metaBasev.append(0) #Reserved UInt16
      metaBasev.append(0) #Reserved UInt16
      metaBasev.append(0) #Reserved UInt16
      metadatav=struct.pack(msgFormatv,*metaBasev)

      messagev=metadatav+str_encode
      #print(len(messagev))

      #print(jpg_buffer.shape)
#          print("")
      #try:
      sender.sendto(messagev,(hostv,portv))
      #except socket.error as msg:
      #    print(msg)
      #    sys.exit()

      #time.sleep(0.2) # allowed time for image processing
      #if chr(cv2.waitKey(1)&255) == 'q':
      #  break

    #except KeyboardInterrupt:
    #    cap.cap.release()
    #    cv2.destroyAllWindows()
    #    break

      sender.close

def Ctrl():
    global ZoomValue
    global followX
    global followY
    global latitude_deg
    global longitude_deg
    global elevation_m_MSL
    global imageProc
    global direction
        # receive RPi name and frame from the RPi and acknowledge
        # the receipt
    msgIn, (address,port) = receiver.recvfrom(65536)
    MsgHeadr=int("1e91",16) #unique message header
    metaFormat='<HHdiiiHHHHHHHHHHHHiiiHHHH'
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
        toggle = metadata[16]
        targx = metadata[3]
        targy = metadata[4]
        targz = metadata[5]
        print(metadata[5])
        print(metadata[3])
        print(metadata[4])
        follow = metadata[17]
        imageProc = metadata[23]
        imageSave = metadata[24]
        if(targx == 0):
                print('Not receiving enemy location')
                toggle = 0;

        if(metadata[21] == 1):
                camx = int(latitude_deg * 1e7)
                camy = int(longitude_deg * 1e7)
                camz = int(elevation_m_MSL * 10)
        else:
                camx = int(metadata[18])
                camy = int(metadata[19])
                camz = int(metadata[20])
        #direction = 180
        if(metadata[22] == 1): #North
                direction = 90
                print("North")
        elif(metadata[22] == 2): #East
                direction = 0
                print("East")
        elif(metadata[22] == 3): #South
                direction = 270
                print("South")
        elif(metadata[22] == 4): #West
                direction = 180
                print("West")

    #camx = int(39.008917 * 1e7)
    #camy = int(-104.88299 * 1e7)
    #camz = int(2163 * 10)
    #camx = int(39.00903 * 1e7)
    #camy = int(-104.8790683 * 1e7)
    #camz = int(2161.8 * 10)
    #follow = 1;
    if(toggle==0):
        if(follow == 1):
                control(targx, targy, targz, camx, camy, camz, direction)
        else:
                if(followX > 390):
                    print("Right");

                    cmd = [129,1,6,1,12,12,2,3,255]
                    #print("cmd is: " + str(cmd))
                    cam.open
                    cam.write(cmd)
                    cam.close
                elif(followX < 250):
                    print("Left");

                    cmd = [129,1,6,1,12,12,1,3,255]
                    #print("cmd is: " + str(cmd))
                    cam.open
                    cam.write(cmd)
                    cam.close
                elif(followY < 170):
                    print("Up");

                    cmd = [129,1,6,1,12,12,3,1,255]
                    #print("cmd is: " + str(cmd))
                    cam.open
                    cam.write(cmd)
                    cam.close
                elif(followY > 310):
                    print("Down");

                    cmd = [129,1,6,1,12,12,3,2,255]
                    #print("cmd is: " + str(cmd))
                    cam.open
                    cam.write(cmd)
                    cam.close
                else:
                    print("Stop");

                    cmd = [129,1,6,1,12,12,3,3,255]
                    #print("cmd is: " + str(cmd))
                    cam.open
                    cam.write(cmd)
                    cam.close
    if(toggle==1):
        if (metadata[6] == 1):
                #print("Up");

                cmd = [129,1,6,1,12,12,3,1,255]
                #print("cmd is: " + str(cmd))
                cam.open
                cam.write(cmd)
                cam.close
        elif (metadata[8] == 1):
                #print("Left");

                cmd = [129,1,6,1,12,12,1,3,255]
                #print("cmd is: " + str(cmd))
                cam.open
                cam.write(cmd)
                cam.close
        elif (metadata[7] == 1):
                #print("Down");

                cmd = [129,1,6,1,12,12,3,2,255]
                #print("cmd is: " + str(cmd))
                cam.open
                cam.write(cmd)
                cam.close
        elif (metadata[9] == 1):
                #print("Right");

                cmd = [129,1,6,1,12,12,2,3,255]
                #print("cmd is: " + str(cmd))
                cam.open
                cam.write(cmd)
                cam.close
        elif (metadata[13] == 1):
                #print("Stop");

                cmd = [129,1,6,1,12,12,3,3,255]
                #print("cmd is: " + str(cmd))
                cam.open
                cam.write(cmd)
                cam.close
        elif (metadata[12] == 1):
                #print("Out");
                ZoomValue = ZoomValue - 1;
                if (ZoomValue < 0):
                    ZoomValue = 0;
                cmd = [129,1,4,71,ZoomValue,ZoomValue,ZoomValue,ZoomValue,
ZoomValue,ZoomValue,ZoomValue,ZoomValue,255]
                #print("cmd is: " + str(cmd))
                cam.open
                cam.write(cmd)
                cam.close
        elif (metadata[11] == 1):
                #print("In");
                ZoomValue = ZoomValue + 1;
                if (ZoomValue > 15):
                    ZoomValue = 15;
                cmd = [129,1,4,71,ZoomValue,ZoomValue,ZoomValue,ZoomValue,
ZoomValue,ZoomValue,ZoomValue,ZoomValue,255]
                #print("cmd is: " + str(cmd))
                cam.open
                cam.write(cmd)
                cam.close
        elif (metadata[10] == 1):
                #print("Home");

                cmd = [129,1,6,4,255]
                #print("cmd is: " + str(cmd))
                cam.open
                cam.write(cmd)
                cam.close

if __name__ == '__main__':
  main()