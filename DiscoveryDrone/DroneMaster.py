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
from UavMavlinkV52 import *
from subprocess import call
import UavMavlinkV52 as mav




import cv2, queue, threading, time
from threading import Thread, Event
from time import sleep

# initialize the ImageSender object with the socket address of the
# server

global latitude_deg
latitude_deg=39.008942
global longitude_deg
longitude_deg=-104.879922
global elevation_m_MSL
elevation_m_MSL=2155.1

global imageProc
imageProc = 1
global imageSave
imageSave = 0

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


host='192.168.1.50'
port=44555

hostr='192.168.1.26';
portr=5666;


try:
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sender.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
except socket.error:
    print('Failed to create socket')
    sys.exit()

rpiName=socket.gethostname()

# bufferless VideoCapture
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

jpeg_quality = 70  # 0 to 100, higher is better quality, 95 is cv2 default
cap = VideoCapture(2)
print("Opening Camera Interface : 2")
noNewFile=True
msgFormat='<HHddddddddHddddhhh'
MsgHeadr=int("a59f",16) # randomly chosen for experiment

blueTargets = None

try:
    receiver=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
except socket.error:
    print('failed to create socket')
    sys.exit()

# bind to port
try:
    receiver.bind((hostr,portr))
except socket.error:
    print("no bind")
    sys.exit() #HAD TO REMOVE TO GET VIDEO FEED TO SEND - ISSUES LATER?


def main():
  verbose=True
  InitMavlink(verbose)
  stopFlag = Event()
  SndImgThread = SndImgC2(stopFlag)
  SndImgThread.start()
  CtrlThread = CtrlC2(stopFlag)
  CtrlThread.start()


def SndImg():
    #try:
      global imageProc
      global imageSave

      frame = cap.read()
      if(imageSave == 1):
         cv2.imwrite('DDrone_' + str(int(datetime.datetime.now(tz=pytz.utc).timestamp())) + '_image_' + '.jpg', frame)
      (B, G, R) = cv2.split(frame)
      zeros = np.zeros(frame.shape[:2], dtype = "uint8")
      blueEmphasis = cv2.merge([B, zeros, zeros])
      blueGray = frame[:,:,0]
      blurred = cv2.GaussianBlur(blueGray, (7, 7), 0)
      (T, bThresh) = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY_INV)
      (cnts, hierarchy) = cv2.findContours(bThresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
             cv2.imwrite('ProcDDrone_' + str(int(datetime.datetime.now(tz=pytz.utc).timestamp())) + '_image_' + '.jpg', blueTargets)
         blueTargets=imutils.resize(blueTargets,width=320)

      if((len(cnts) > 0) & (imageProc == 1)):
             ret_code, jpg_buffer = cv2.imencode(
         ".jpg", blueTargets, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
      else:
             ret_code, jpg_buffer = cv2.imencode(
          ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])

      dataEncode=np.array(jpg_buffer)
      str_encode = dataEncode.tobytes()

      metaBase=[MsgHeadr]
      metaBase.append(np.uint16(108)) # metadata size
      metaBase.append(datetime.datetime.now(tz=pytz.utc).timestamp())
      #metaBase.append(int(latitude_deg*1e7))
      #metaBase.append(int(longitude_deg*1e7))
      #metaBase.append(int(elevation_m_MSL*10))
      metaBase.append(int(mav.currentLat_Deg * 1e7))
      metaBase.append(int(mav.currentLon_Deg * 1e7))
      metaBase.append(int(mav.currentAlt_mMSL * 10))
      metaBase.append(90) #Azimuth (Deg)
      metaBase.append(0) #Tilt (Deg)
      metaBase.append(0) #Zoom (%)
      metaBase.append(320) #Image Size (Pixels)
      metaBase.append(0) #Blob Present (y or n)
      metaBase.append(0) #Blob Size (Pixels)
      metaBase.append(0) #Blob location on screen (x,y)
      metaBase.append(0) #Object Classification
      metaBase.append(0) #Object Class. certainty
      metaBase.append(0) #Velocity X
      metaBase.append(0) #Velocity Y
      metaBase.append(0) #Velocity Z
      metadata=struct.pack(msgFormat,*metaBase)
      #sleep(.05)
      message=metadata+str_encode
      #print(len(message))


      #print(jpg_buffer.shape)
#          print("")
      try:
          sender.sendto(message,(host,port))
      except socket.error as msg:
          print(msg)
          sys.exit()

      #time.sleep(0.2) # allowed time for image processing
      #if chr(cv2.waitKey(1)&255) == 'q':
      #  break

    #except KeyboardInterrupt:
    #    cap.cap.release()
    #    cv2.destroyAllWindows()
    #    break

      sender.close

      #cv2.destroyAllWindows()
      #cap.cap.release()
      #sys.exit()

def Ctrl():
        # receive RPi name and frame from the RPi and acknowledge
        # the receipt
    global imageProc
    global imageSave
    msgIn, (address,port) = receiver.recvfrom(65536)
    MsgHeadrR=int("a33e",16) #unique message header
    msgFormatR='<HHdHHHHddd'
        # verify correct message
    temp=msgIn[0:2]
    recvHdr=struct.unpack('<H', temp)[0]

    if(recvHdr==MsgHeadrR):
            # get metadata size
        temp=msgIn[2:4]
        msgLen=struct.unpack('<H',temp)[0]
            # get metadata
        temp=msgIn[0:msgLen]
        msgdata=struct.unpack(msgFormatR,temp)
        #print(msgdata[4])
        if(msgdata[4] == 1):
            SendTilt(-90, 0)
        if(msgdata[4] == 2):
            SendTilt(-10, 90)
        if(msgdata[4] == 3):
            SendTilt( 10, 180)
        if(msgdata[4] == 4):
            SendTilt( 90, 270)

    imageProc = msgdata[5]
    imageSave = msgdata[6]



if __name__ == '__main__':
  main()
