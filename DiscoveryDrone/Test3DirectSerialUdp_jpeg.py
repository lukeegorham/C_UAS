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
import serial
import serial.threaded
import struct
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


# create dgram udp socket
    # connect to serial port
     # connect to serial port

ser = serial.Serial(
    port='/dev/ttyAMA1',
    baudrate=230400,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
ser.rtscts = False
ser.xonxoff = False


if ser.isOpen():
    try:
        ser.flushInput() #flush input buffer, discarding all its contents
        ser.flushOutput()#flush output buffer, aborting current output 
                 #and discard all that is in buffer
    except serial.SerialException as e1:
        print("error communicating...: " + str(e1))

else:
    try:
        ser.open()
    except serial.SerialException as e:
        sys.stderr.write('Could not open serial port {}: {}\n'.format(ser.name, e))
        sys.exit(1)
        print("cannot open serial port ")
    

# bufferless VideoCapture
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
      str_encode = dataEncode.tobytes()
#      data=np.ndarray.tobytes(jpg_buffer)
#      data=np.ndarray.tobytes(temp)
      
#      data=json.dumps(jpg_buffer, cls=NumpyArrayEncoder).encode('utf8')
#      data=json.dumps(jpg_buffer, cls=NumpyArrayEncoder)
#      if(noNewFile):
      print(frame.shape)
      print(sys.getsizeof(str_encode))
      print(jpg_buffer.size)
#      sendSize=sys.getsizeof(str_encode)
      sendSize=jpg_buffer.size
      sizeBytes=sendSize.to_bytes(4,byteorder='little')
      print(sizeBytes)
      print(str_encode[sendSize-100:sendSize])
#          print("")
      try:
          ser.write(sizeBytes)
          ser.write(str_encode)
      except serial.SerialException as msg:
          print(msg)
          sys.exit()
          
#      cv2.imshow("frame", frame)
      time.sleep(0.2) # allowed time for image processing
      if chr(cv2.waitKey(1)&255) == 'q':
        break

    except KeyboardInterrupt:
        ser.close()
        cap.cap.release()
        cv2.destroyAllWindows()
        break
    
ser.close()
cap.cap.release()
cv2.destroyAllWindows()
sys.exit()
