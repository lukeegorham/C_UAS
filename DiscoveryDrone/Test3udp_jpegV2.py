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
#import pyrealsense2 as rs
##import sys
##import pickle
#cap=cv2.VideoCapture(4)
#while True:
#    ret,frame=cap.read()
#    cv2.imshow('frame',frame)
#    if cv2.waitKey(1) & 0xFF == ord('q'):
#        break
#cap.release()
#cv2.destroyAllWindows()

import cv2, queue, threading, time

# initialize the ImageSender object with the socket address of the
# server
#host='192.168.1.75'
host='192.168.1.255'
port=5555
#sender = imagezmq.ImageSender(connect_to='tcp://192.168.1.75:5555')
# create dgram udp socket

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
    t = threading.Thread(target=self._reader)
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
cap = VideoCapture(4)
noNewFile=True

while True:
#  time.sleep(.5)   # simulate time between events
    try:
      frame = cap.read()
      frame=imutils.resize(frame,width=320)
      ret_code, jpg_buffer = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
      dataEncode=np.array(jpg_buffer)
      str_encode = dataEncode.tostring()
#      data=np.ndarray.tobytes(jpg_buffer)
#      data=np.ndarray.tobytes(temp)
      
#      data=json.dumps(jpg_buffer, cls=NumpyArrayEncoder).encode('utf8')
#      data=json.dumps(jpg_buffer, cls=NumpyArrayEncoder)
#      if(noNewFile):
#          print(frame.shape)
      print(jpg_buffer.shape)
#          print("")
      try:
          sender.sendto(str_encode,(host,port))
      except socket.error as msg:
          print(msg)
          sys.exit()
          
#      cv2.imshow("frame", frame)
      time.sleep(0.2) # allowed time for image processing
      if chr(cv2.waitKey(1)&255) == 'q':
        break

    except KeyboardInterrupt:
        cap.cap.release()
        cv2.destroyAllWindows()
        break
    
sender.close
cap.cap.release()
cv2.destroyAllWindows()
sys.exit()
