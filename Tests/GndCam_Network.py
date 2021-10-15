import numpy as np
import argparse
import cv2
import serial
from serial import Serial
import socket
import threading
import base64
import zmq

# Global Variables
keep_processing = True
key = None
c = None
z_val = 0
host_gs1 = '192.168.1.50'
port_gs1 = 46554
host_cam = '192.168.1.26'
port_cam = 46554
global cam


# Daemon Thread to Display Camera
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
            

# Initialize Camera
print "Initializing Camera..."
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


# Initialize Networking
# Initialize Connection to Get Commands
print "Initializing Camera Controls..."
socket_cam = socket.socket()
socket_cmdbind((host_cam, port_cam))
socket_cmdlisten(10)
c, addr = socket_cmd.accept()  # blocks/waits for connection from GroundStation1
# Initialize Connection to Send Video Feed
print "Attaching Video Feed..."
context = zmq.Context()
socket_gs1 = context.socket(zmq.PUB)
socket_gs1.connect('tcp://' + host_gs1 + ':' + str(port_gs1))  # blocks/waits for connection to GroundStation1


# GroundStation Control Loop (manual controls)
def GS_Controls():
    global cam  # Camera socket
    print "Starting process..."
    thread_1 = threading.Thread(target=display_daemon, args=(), daemon=True)
    thread_1.start()
    while (keep_processing):
        # Receive command from GroundStation1
        data = cam_connection.recv(32).decode()  # receive network packet into 32byte buffer
        key = str(data)[0] # assign first character in string to key
        cam_connection.send(data)
        # Open serial port, select message, send message, close serial port
        cam.open()
        if (key == 'q'):  # Exit case
                cam.close()
                cv2.destroyAllWindows()
                keep_processing = False
                socket_cmd.close()
                socket_gs1.close()
                break
        elif (key == 'w'):  # Pan Up
                cmd = [129,1,6,1,12,12,3,1,255]
                cam.write(cmd)
        elif (key == 'a'):  # Pan Left
                cmd = [129,1,6,1,12,12,1,3,255]
                cam.write(cmd)
        elif (key == 's'):  # Pan Down
                cmd = [129,1,6,1,12,12,3,2,255]
                cam.write(cmd)
        elif (key == 'd'):  # Pan Right
                cmd = [129,1,6,1,12,12,2,3,255]
                cam.write(cmd)
        elif (key == 'f'):  # Stop
                cmd = [129,1,6,1,12,12,3,3,255]
                cam.write(cmd)
        elif (key == 'o'):  # Zoom Out
                z_val = z_val - 1;
                if (z_val < 0):
                    z_val = 0;
                cmd = [129,1,4,71,z_val,z_val,z_val,z_val,z_val,z_val,z_val,z_val,255]
                cam.write(cmd)
        elif (key == 'i'):  # Zoom In
                z_val = z_val + 1;
                if (z_val > 15):
                    z_val = 15;
                cmd = [129,1,4,71,z_val,z_val,z_val,z_val,z_val,z_val,z_val,z_val,255]
                cam.write(cmd)
        elif (key == 't'):  # Unsure
                cmd = [129,1,6,3,20,20,15,7,15,14,0,1,12,8,255]
                cam.write(cmd)
        elif (key == 'h'):  # Pan to Home (straight forward)
                cmd = [129,1,6,4,255]
                cam.write(cmd)
        elif (key == 'm'):  # Change Mode to Automatic Tracking
                cam.close()
                return
        cam.close()


    # Close Application Neatly After Breaking From Loop
    print "Closing application..."
    quit()


# Use Image Processing to Aim the Camera
def Auto_Controls():
    print "Initializing Auto Tracking..."
    print "Unable to find drone in image, exiting..."
    return


def main():
    while True:
        GS_Controls()    # Initialize with manual controls
        Auto_Controls()  # Then go to automatic controls when target acquired

if __name__ == '__main__':
    main()