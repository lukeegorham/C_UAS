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

class SerialToNet(serial.threaded.Protocol):
    """serial->socket"""

    def __init__(self):
        self.socket = None

    def __call__(self):
        return self

    def data_received(self, data):
        if self.socket is not None:
            self.socket.sendall(data)

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

# initialize the ImageSender object with the socket address of the
# server
host='192.168.1.75'
port=20002
#sender = imagezmq.ImageSender(connect_to='tcp://192.168.1.75:5555')

# create dgram udp socket
    # connect to serial port
     # connect to serial port
ser = serial.serial_for_url('/dev/ttyAMA1', do_not_open=True)
ser.baudrate = 115200
ser.bytesize = 8
ser.parity = 'N'
ser.stopbits = 1
ser.rtscts = True
ser.xonxoff = False

try:
    ser.open()
except serial.SerialException as e:
    sys.stderr.write('Could not open serial port {}: {}\n'.format(ser.name, e))
    sys.exit(1)

try:
	sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
except socket.error:
	print('Failed to create socket')
	sys.exit()

ser_to_net = SerialToNet()
serial_worker = serial.threaded.ReaderThread(ser, ser_to_net)
serial_worker.start()
    
rpiName=socket.gethostname()

# bufferless VideoCapture
jpeg_quality = 70  # 0 to 100, higher is better quality, 95 is cv2 default
cap = VideoCapture(4)
noNewFile=True

try:
    intentional_exit = False
    while True:
        sys.stderr.write("Opening connection to {}:{}...\n".format(host, port))
        client_socket = socket.socket()
        try:
            client_socket.connect((host, int(port)))
        except socket.error as msg:
            sys.stderr.write('WARNING: {}\n'.format(msg))
            time.sleep(5)  # intentional delay on reconnection as client
            continue
        sys.stderr.write('Connected\n')
        client_socket.setsockopt(socket.AF_INET,socket.SOCK_DGRAM)
            #~ client_socket.settimeout(5)
        try:
            ser_to_net.socket = client_socket
            # enter network <-> serial loop
            while True:
                try:
                    data = client_socket.recv(1024)
                    if not data:
                        break
                    ser.write(data)                 # get a bunch of bytes and send them
                except socket.error as msg:
                    sys.stderr.write('ERROR: {}\n'.format(msg))
                    # probably got disconnected
                    break
        except KeyboardInterrupt:
            intentional_exit = True
            raise
        except socket.error as msg:
            sys.stderr.write('ERROR: {}\n'.format(msg))
        finally:
            ser_to_net.socket = None
            sys.stderr.write('Disconnected\n')
            client_socket.close()
            time.sleep(5)  # intentional delay on reconnection as client
except socket.error:
    sys.stderr.write('\n--- exit ---\n')
    serial_worker.stop()    

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
          client_socket.sendto(str_encode,(host,port))
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
    
client_socket.close()
serial_worker.stop()
cap.cap.release()
cv2.destroyAllWindows()
sys.exit()
