import numpy as np
import argparse
import cv2
import serial
from serial import Serial
import socket
import threading
import base64
import zmq

# # # GLOBAL VARIABLES # # #
keep_processing = True
key = None
global c
c = None
global socket_gs1
global s
ZoomValue = 0
host_gs1 = '192.168.1.50'
port_gs1 = 46554
host_gcam = '192.168.1.26'
port_gcam = 46554


# # # DAEMON FUNCTION TO DISPLAY CAMERA # # #
def display_daemon():
    global socket_gs1
    # Get, Show, Then Send Video Frames
    while True:
        ret, frame = cap.read()
        cv2.imshow('Live GroundStation Camera Feed', frame)
        cv2.waitKey(1)
        frame = cv2.resize(frame, (640,480))
        encoded, buffer = cv2.imencode('.jpg', frame)
        jpg_as_text = base64.b64encode(buffer)
        socket_gs1.send(jpg_as_text)
            
            
def init_networking():
    global c
    global s
    global socket_gs1
    # Initialize Connection to Get Commands
    print(f'Connecting to Controls...')
    s = socket.socket()
    s.bind((host_gcam, port_gcam))
    s.listen(10)
    c, addr = s.accept()  # wait for connection to server
    # Initialize Connection to Send Video Feed
    print(f'Attaching Video Feed...')
    context = zmq.Context()
    socket_gs1 = context.socket(zmq.PUB)
    socket_gs1.connect('tcp://192.168.1.50:46554')


# # # INITIALIZE CAMERA # # #
print(f'Initializing Camera...')
cap  = cv2.VideoCapture(0)
cam = Serial('/dev/ttyUSB0',9600) # worked on GPU
cam.close()
cmd = [129,1,4,53,2,255] #outdoor mode = 2
cam.open()
cam.write(cmd)
cam.close()
cmd = [129,1,4,57,10,255] #shutter priority
cam.open()
cam.write(cmd)
cam.close()


# # # INITIALIZE LISTENING INTERFACE WITH GROUNDSTATION # # #
print(f'Initializing Network...')
init_networking()


# # # STARTING CONTROL LOOP # # #
print(f'Starting process...')
thread_1 = threading.Thread(target=display_daemon, args=(), daemon=True)
thread_1.start()
while (keep_processing):
    # Receive command from GroundStation1
    data = c.recv(32).decode()  # receive network packet into 32byte buffer
    key = str(data)[0] # assign first character in string to key
    c.send(b'Thanks!')
    # Open serial port, select message, send message, close serial port
    cam.open()
    if (key == 'q'):  # Exit case
            cam.close()
            cv2.destroyAllWindows()
            keep_processing = False
            s.close()
            socket_gs1.close()
            break
    elif (key == 'w'):
            cmd = [129,1,6,1,12,12,3,1,255]
            cam.write(cmd)
    elif (key == 'a'):
            cmd = [129,1,6,1,12,12,1,3,255]
            cam.write(cmd)
    elif (key == 's'):
            cmd = [129,1,6,1,12,12,3,2,255]
            cam.write(cmd)
    elif (key == 'd'):
            cmd = [129,1,6,1,12,12,2,3,255]
            cam.write(cmd)
    elif (key == 'f'):
            cmd = [129,1,6,1,12,12,3,3,255]
            cam.write(cmd)
    elif (key == 'o'):
            ZoomValue = ZoomValue - 1;
            if (ZoomValue < 0):
                ZoomValue = 0;
            cmd = [129,1,4,71,ZoomValue,ZoomValue,ZoomValue,ZoomValue,
ZoomValue,ZoomValue,ZoomValue,ZoomValue,255]
            cam.write(cmd)
    elif (key == 'i'):
            print("In");
            ZoomValue = ZoomValue + 1;
            if (ZoomValue > 15):
                ZoomValue = 15;
            cmd = [129,1,4,71,ZoomValue,ZoomValue,ZoomValue,ZoomValue,
ZoomValue,ZoomValue,ZoomValue,ZoomValue,255]
            cam.write(cmd)
    elif (key == 't'):
            cmd = [129,1,6,3,20,20,15,7,15,14,0,1,12,8,255]
            cam.write(cmd)
    elif (key == 'h'):
            cmd = [129,1,6,4,255]
            cam.write(cmd)
    cam.close()


print(f'Closing application...')
quit()
