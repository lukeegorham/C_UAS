import numpy as np
import argparse
import cv2
import serial
from serial import Serial
import socket
import threading
import base64
import zmq
import time

# Global Variables
connected = False
key = None
c = None
thread_1 = None
z_val = 0
host_gs1 = '192.168.1.50'
port_gs1 = 46554
host_cam = '192.168.1.26'
port_cam = 46554

# Socket Stuff
socket_cmd = socket.socket()
socket_cmd.bind((host_cam, port_cam))
socket_cmd.listen(2)
socket_cmd.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
socket_gs1 = None
addr = None
cam_connection = None

# Initialize Camera
print("Initializing Camera Serial Port Connection...")
cap = cv2.VideoCapture(0)
cam = Serial('/dev/ttyUSB0', 9600)  # worked on GPU
cam.close()
cmd = [129, 1, 4, 53, 2, 255]  # outdoor mode = 2
cam.open()
cam.write(cmd)
cam.close()
cmd = [129, 1, 4, 57, 10, 255]  # shutter priority
cam.open()
cam.write(cmd)
cam.close()
cam.open()
cmd = [129, 1, 6, 4, 255]  # Center camera
cam.write(cmd)
cam.close()


# Daemon Thread to Display Camera
def display_daemon():
    global socket_gs1
    global connected
    # Get, Show, Then Send Video Frames
    while not connected:
        time.sleep(0.1)
    while connected:
        ret, frame = cap.read()
        # cv2.imshow('Live GroundStation Camera Feed', frame)
        # cv2.waitKey(1)
        frame = cv2.resize(frame, (640, 480))
        encoded, buffer = cv2.imencode('.jpg', frame)
        jpg_as_text = base64.b64encode(buffer)
        try:
            socket_gs1.send(jpg_as_text)
        except zmq.error.ZMQError:
            break
    # cv2.destroyAllWindows()
    # cv2.waitKey(1)


# Initialize Networking
def GetConnection():
    global socket_cmd
    global socket_gs1
    global cam_connection
    global addr
    global connected
    global thread_1
    global host_gs1
    if connected:
        return
    # Initialize Connection to Get Commands
    print("Waiting For Connection From Camera Controls...")
    cam_connection, addr = socket_cmd.accept()  # blocks/waits for connection from GroundStation1
    print("Connected to", addr, "...")
    # Initialize Connection to Send Video Feed
    context = zmq.Context()
    socket_gs1 = context.socket(zmq.PUB)
    if addr:
        host_gs1 = str(addr[0])
    print("Sending Video Feed To", host_gs1, "...")
    socket_gs1.connect('tcp://' + host_gs1 + ':' + str(port_gs1))  # blocks/waits for connection to GroundStation1
    connected = True


# GroundStation Control Loop (manual controls)
def GS_Controls():
    global cam  # Camera socket
    global cmd
    global connected
    global cam_connection
    global key
    global socket_cmd
    global socket_gs1
    global z_val
    global cap
    global thread_1
    if not connected:
        return
    print("Starting Control Loop...")
    if thread_1 is None:  # Starts display Daemon
        thread_1 = threading.Thread(target=display_daemon, args=(), daemon=True)
        thread_1.start()
        print("Started...")
    while True:
        # Receive command from GroundStation1
        data = cam_connection.recv(32).decode()  # receive network packet into 32byte buffer
        key = data[0]  # assign first character in string to key
        cam_connection.send(bytes(key, 'utf8'))
        print("Command", key)
        # Open serial port, select message, send message, close serial port
        cam.open()
        if (key == 'q'):  # Exit case
            print("Disconnected From Controls...")
            cam.close()
            connected = False
            socket_gs1.close()
            cv2.destroyAllWindows()  # Yes, technically calling this twice, it helps to close without errors
            cv2.waitKey(1)
            break
        elif (key == 'w'):  # Pan Up
            cmd = [129, 1, 6, 1, 12, 12, 3, 1, 255]
            cam.write(cmd)
        elif (key == 'a'):  # Pan Left
            cmd = [129, 1, 6, 1, 12, 12, 1, 3, 255]
            cam.write(cmd)
        elif (key == 's'):  # Pan Down
            cmd = [129, 1, 6, 1, 12, 12, 3, 2, 255]
            cam.write(cmd)
        elif (key == 'd'):  # Pan Right
            cmd = [129, 1, 6, 1, 12, 12, 2, 3, 255]
            cam.write(cmd)
        elif (key == 'f'):  # Stop
            cmd = [129, 1, 6, 1, 12, 12, 3, 3, 255]
            cam.write(cmd)
        elif (key == 'o'):  # Zoom Out
            z_val = z_val - 1
            if (z_val < 0):
                z_val = 0
            cmd = [129, 1, 4, 71, z_val, z_val, z_val, z_val, z_val, z_val, z_val, z_val, 255]
            cam.write(cmd)
        elif (key == 'i'):  # Zoom In
            z_val = z_val + 1
            if (z_val > 15):
                z_val = 15
            cmd = [129, 1, 4, 71, z_val, z_val, z_val, z_val, z_val, z_val, z_val, z_val, 255]
            cam.write(cmd)
        elif (key == 't'):  # Unsure
            cmd = [129, 1, 6, 3, 20, 20, 15, 7, 15, 14, 0, 1, 12, 8, 255]
            cam.write(cmd)
        elif (key == 'h'):  # Pan to Home (straight forward)
            cmd = [129, 1, 6, 4, 255]
            cam.write(cmd)
        elif (key == 'm'):  # Change Mode to Automatic Tracking
            cam.close()
            return
        cam.close()

    # Close Application Neatly After Breaking From Loop
    # print("Connection Terminated...")


# Use Image Processing to Aim the Camera
def Auto_Controls():
    print("Initializing Auto Tracking...")
    print("Unable to find drone in image, exiting...")
    return


def main():
    global cap
    global socket_gs1
    global socket_cmd
    global thread_1
    try:
        while True:  # Initialize with manual controls
            GetConnection()
            GS_Controls()  # Use manual controls until q signal
            if connected:
                Auto_Controls()  # Then go to automatic controls
            else:
                thread_1.join()
                thread_1 = None
    except KeyboardInterrupt:
        print("Exiting Program...")
    try:
        socket_gs1.close()
    except:
        print("Video Socket Already Closed!")
    try:
        socket_cmd.close()
    except:
        print("Command Socket Already Closed!")
    print("Good Shutdown Conditions... GOODBYE!")
    quit()


if __name__ == '__main__':
    main()

