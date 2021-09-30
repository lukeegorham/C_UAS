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
import imagezmq
import imutils
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

sender = imagezmq.ImageSender(connect_to='tcp://192.168.1.75:5555')

rpiName=socket.gethostname()
jpeg_quality = 95  # 0 to 100, higher is better quality, 95 is cv2 default
    
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

cap = VideoCapture(4)
while True:
#  time.sleep(.5)   # simulate time between events
    try:
      frame = cap.read()
#      frame=imutils.resize(frame,width=480)
      ret_code, jpg_buffer = cv2.imencode(
        ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
      sender.send_jpg(rpiName, jpg_buffer)
#      sender.send_image(rpiName, frame)
      cv2.imshow("frame", frame)
      if chr(cv2.waitKey(1)&255) == 'q':
        break

    except KeyboardInterrupt:
        cap.cap.release()
        cv2.destroyAllWindows()
        break
    
sender.close()
cap.cap.release()
cv2.destroyAllWindows()
