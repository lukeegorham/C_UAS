# import the necessary packages
from datetime import datetime
import numpy as np
import socket
import argparse
import imutils
import cv2
import sys
import struct
# initialize the ImageHub object


HOST='';
PORT=4555;
jpeg_quality=80

#create socket
try:
    receiver=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
except socket.error:
    print('failed to create socket')
    sys.exit()

# bind to port
try:
    receiver.bind((HOST,PORT))
except socket.error:
    print("no bind")
    sys.exit()

while True:
    try:
	    # receive RPi name and frame from the RPi and acknowledge
	    # the receipt
        msgIn, (address,port) = receiver.recvfrom(65536)
        MsgHeadr=int("a59f",16) #unique message header
        metaFormat='<HHddddddddHddddhhh'
        # verify correct message
        temp=msgIn[0:2]
        recvHdr=struct.unpack('<H', temp)[0]
        print(MsgHeadr,recvHdr)
        if(recvHdr==MsgHeadr):
            # get metadata size
            temp=msgIn[2:4]
            metaLen=struct.unpack('<H',temp)[0]
            # get metadata
            temp=msgIn[0:metaLen]
            metadata=struct.unpack(metaFormat,temp)
            jpg_buffer=msgIn[metaLen:] #image is rest of message
            
            timestamp=metadata[2]
            azimuth=metadata[6]
            print(azimuth)
            frame1=np.asarray(bytearray(jpg_buffer),dtype="uint8")
            
            frame1=cv2.imdecode(frame1, cv2.IMREAD_COLOR)
            frame=imutils.resize(frame1,640)
            cv2.imshow("frame",frame)

        key = cv2.waitKey(1) & 0xFF
	    # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
        elif key == ord('a'):
            keypress = 'a'
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        break
# do a bit of cleanup
cv2.destroyAllWindows()

