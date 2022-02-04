#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 21 12:41:31 2020

@author: pi
"""

#import cv2
##import numpy as np
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
import numpy as np


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

cap = VideoCapture(2) #when there are errors stating "can't open camera by index" try changing the 1 to 0,2,3, or 4
while True:
#  time.sleep(.5)   # simulate time between events
    try:
        frame = cap.read()
        (B, G, R) = cv2.split(frame)
        zero = np.zeros(frame.shape[:2], dtype = "uint8")
        blueEmphasis = cv2.merge([B, zero, zero])
        blueGray = frame[:,:,0]
        blurred = cv2.GaussianBlur(blueGray, (7, 7), 0)
        (T, bThresh) = cv2.threshold(blurred, 75, 255, cv2.THRESH_BINARY_INV)
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

        blueTargets = frame.copy()
        cv2.drawContours(frame, cnts, -1, (0, 255, 0), 2)
        (x,y),radius = cv2.minEnclosingCircle(biggestcnt)
        center = (int(x),int(y))
        radius = int(radius)
        cv2.circle(blueTargets,center,radius,(0,0,255),2)

        cv2.imshow("Output", blueTargets)
        if chr(cv2.waitKey(1)&255) == 'q':
            break

    except KeyboardInterrupt:
        break
cap.cap.release()
cv2.destroyAllWindows()
