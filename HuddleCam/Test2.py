#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 21 12:41:31 2020

@author: pi
"""

#import cv2
##import numpy as np
import socket
import zmq
import base64
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

context = zmq.Context()
footage_socket = context.socket(zmq.PUB)
footage_socket.connect('tcp://192.168.1.255:5555')

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
      encoded, buffer = cv2.imencode('.jpg', frame)
      jpg_as_text = base64.b64encode(buffer)
      footage_socket.send(jpg_as_text)      
      cv2.imshow("frame", frame)
      if chr(cv2.waitKey(1)&255) == 'q':
        break

    except KeyboardInterrupt:
        cap.cap.release()
        cv2.destroyAllWindows()
        break
    
cap.cap.release()
cv2.destroyAllWindows()
