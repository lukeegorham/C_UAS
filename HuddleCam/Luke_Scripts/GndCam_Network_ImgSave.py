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
take_pic = 0  # Control mutex to take picture
a = 0         # Picture number
connected = False
key = None
c = None
thread_1 = None
z_val = 0
port_gs1 = 46554
host_cam = '192.168.1.22'
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
cam = Serial('/dev/ttyUSB0',9600) # worked on GPU
cam.close()
cmd = [129,1,4,53,2,255] #outdoor mode = 2
cam.open()
cam.write(cmd)
cam.close()
cmd = [129,1,4,57,0,255]  #shutter priority
cam.open()
cam.write(cmd)
cam.close()
cam.open()
cmd = [129,1,6,4,255]  # Center camera
cam.write(cmd)
cam.close()


# Daemon Thread to Display Camera
def display_daemon():
    global socket_gs1
    global connected
    global take_pic
    global a
    
    #CHANGE FOLDER NAME FOR EACH TEST
    RESULT_VID_PATH = "./BlobResults/" ##########
    a = 0;
    
    # Get, Show, Then Send Video Frames
    while not connected:
        time.sleep(0.1)
    while connected:
        ret, image = cap.read()
        
        # when we reach the end of the video (file) exit cleanly
        if not ret:
            keep_processing = False;
            
        (B, G, R) = cv2.split(image)
        zero = np.zeros(image.shape[:2], dtype = "uint8")
        blueEmphasis = cv2.merge([B, zero, zero])
        blueGray = image[:,:,0]
        blurred = cv2.GaussianBlur(blueGray, (7, 7), 0)
        (T, bThresh) = cv2.threshold(blurred, 75, 255, cv2.THRESH_BINARY_INV)
        (__, cnts, hierarchy) = cv2.findContours(bThresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
        # print(M)
        blueTargets = image.copy()
        blueTargetsCont = blueTargets.copy()
        cv2.drawContours(blueTargetsCont, cnts, -1, (0, 255, 0), 2)
        (x,y),radius = cv2.minEnclosingCircle(biggestcnt)
        center = (int(x),int(y))
        radius = int(radius)
        cv2.circle(blueTargets,center,radius,(0,0,255),2)

        # out.write(blueTargets)

        # cv2.imshow("Output", blueTargets)
    
        key = cv2.waitKey(1) & 0xFF;
        
        # char = input("Want to screenshot? : ")

        if(take_pic == 1):
            cv2.imwrite(RESULT_VID_PATH + 'Raw' + str(a) + '.png', image)
            cv2.imwrite(RESULT_VID_PATH + 'Gray' + str(a) + '.png', blueGray)
            cv2.imwrite(RESULT_VID_PATH + 'Blur' + str(a) + '.png', blurred)
            cv2.imwrite(RESULT_VID_PATH + 'Contours' + str(a) + '.png', blueTargetsCont)
            cv2.imwrite(RESULT_VID_PATH + 'Processed' + str(a) + '.png', blueTargets)
            print("Picture taken!")
            a = a + 1
            take_pic = 0
        
        
        blueTargets = cv2.resize(blueTargets, (640,480))
        encoded, buffer = cv2.imencode('.jpg', blueTargets)
        jpg_as_text = base64.b64encode(buffer)
        
        # Input Save Code         socket_gs1.send(jpg_as_text)
        
        
        
    cv2.destroyAllWindows()
    cv2.waitKey(1)
        

# Initialize Networking
def GetConnection():
    global socket_cmd
    global socket_gs1
    global cam_connection
    global addr
    global connected
    global thread_1
    if connected:
        return
    # Initialize Connection to Get Commands
    print("Waiting For Connection From Camera Controls...")
    cam_connection, (addr, port) = socket_cmd.accept()  # blocks/waits for connection from GroundStation1
    print("Connected to", addr, "...")
    # Initialize Connection to Send Video Feed
    print("Sending Video Feed to", addr , "...")
    context = zmq.Context()
    socket_gs1 = context.socket(zmq.PUB)
    socket_gs1.connect('tcp://' + addr + ':' + str(port_gs1))  # blocks/waits for connection to GroundStation1
    connected = True


# GroundStation Control Loop (manual controls)
def GS_Controls():
    global cam  # Camera socket
    global connected
    global cam_connection
    global cam
    global socket_cmd
    global socket_gs1
    global z_val
    global cap
    global thread_1
    global take_pic
    global a
    if not connected:
        return
    print("Starting Control Loop...")
    if thread_1 == None:   # Starts display Daemon
        thread_1 = threading.Thread(target=display_daemon, args=(), daemon=True)
        thread_1.start()
    while True:
        # Receive command from GroundStation1
        data = cam_connection.recv(32).decode()  # receive network packet into 32byte buffer
        key = data[0] # assign first character in string to key
        cam_connection.send(bytes(key, 'utf8'))
        print("Command", key)
        # Open serial port, select message, send message, close serial port
        try:
             cam.open()
        except:
             print("Camera already opened, continuing...")
        if (key == 'q'):  # Exit case
                print("Disconnected From Controls...")
                cam.close()
                cv2.destroyAllWindows()  # Yes, technically calling this twice, it helps to close without errors
                cv2.waitKey(1)
                connected = False
                break
        elif (key == 'w'):  # Pan Up
                cmd = [129,1,6,1,12,12,3,1,255]
                cam.write(cmd)
                print("Wrote Up:", key)
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
                print("Wrote Stop:", key)
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
        elif (key == 'p'):  # Change Mode to Automatic Tracking
                take_pic = 1
                print(f"Taking picture: {a}!") 
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
    global socket_md
    global thread_1
    try:
        while True:    # Initialize with manual controls
            GetConnection()
            GS_Controls()    # Use manual controls until q signal
            if connected:
                Auto_Controls()  # Then go to automatic controls
            else:
                thread_1.join()
                thread_1 = None
    except KeyboardInterrupt:
        print("Exiting Program...")
    try:
        socket_gs1.close()
        socket_cmd.close()
    except:
        print("Nothing to close!")
    quit()
    

if __name__ == '__main__':
    main()
