### Transcribing blob detection code from my laptop,but with input as video and not set of pics###
import numpy as np
import argparse
import cv2
import serial
from serial import Serial

RESULT_VID_PATH = "./BlobResults/"

cap  = cv2.VideoCapture(0)
cam = Serial('/dev/ttyUSB0',9600) #Identifying port camera is connected to
cam.close

#Next two commands necessary to instantiate camera
cmd = [129,1,4,53,2,255] #outdoor mode = 2
cam.open
cam.write(cmd)
cam.close

cmd = [129,1,4,57,10,255] #shutter priority
cam.open
cam.write(cmd)
cam.close

#Getting specs on video
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

#Writing output video
out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'XVID'), fps, (width, height))

keep_processing = True

ZoomValue = 0;

while (keep_processing == True):
    # load the input image and construct an input blob for the image
    # by resizing to a fixed 300x300 pixels and then normalizing it
    # (note: normalization is done via the authors of the MobileNet SSD
    # implementation)
    #   image = cv2.imread(args["image"])
    if (cap.isOpened):
        ret, image = cap.read();
            
        # when we reach the end of the video (file) exit cleanly
        if not ret:
            keep_processing = False;

        out.write(image)

        #cv2.imshow("Output", image)
    
        key = cv2.waitKey(1) & 0xFF;
    
    #The following allow user to input keyboard commands to control camera
    if (key == ord('x')):
            keep_processing = False;
    elif (key == ord('w')):
            print("Up");
            
            cmd = [129,1,6,1,12,12,3,1,255]
            print("cmd is: " + str(cmd))
            cam.open
            cam.write(cmd)
            cam.close
    elif (key == ord('a')):
            print("Left");
            
            cmd = [129,1,6,1,12,12,1,3,255]
            print("cmd is: " + str(cmd))
            cam.open
            cam.write(cmd)
            cam.close
    elif (key == ord('s')):
            print("Down");
            
            cmd = [129,1,6,1,12,12,3,2,255]
            print("cmd is: " + str(cmd))
            cam.open
            cam.write(cmd)
            cam.close
    elif (key == ord('d')):
            print("Right");
            
            cmd = [129,1,6,1,12,12,2,3,255]
            print("cmd is: " + str(cmd))
            cam.open
            cam.write(cmd)
            cam.close
    elif (key == ord('f')):
            print("Stop");
            
            cmd = [129,1,6,1,12,12,3,3,255]
            print("cmd is: " + str(cmd))
            cam.open
            cam.write(cmd)
            cam.close
    elif (key == ord('o')):
            print("Out");
            ZoomValue = ZoomValue - 1;
            if (ZoomValue < 0):
                ZoomValue = 0;
            cmd = [129,1,4,71,ZoomValue,ZoomValue,ZoomValue,ZoomValue,
ZoomValue,ZoomValue,ZoomValue,ZoomValue,255]
            print("cmd is: " + str(cmd))
            cam.open
            cam.write(cmd)
            cam.close
    elif (key == ord('i')):
            print("In");
            ZoomValue = ZoomValue + 1;
            if (ZoomValue > 15):
                ZoomValue = 15;
            cmd = [129,1,4,71,ZoomValue,ZoomValue,ZoomValue,ZoomValue,
ZoomValue,ZoomValue,ZoomValue,ZoomValue,255]
            print("cmd is: " + str(cmd))
            cam.open
            cam.write(cmd)
            cam.close
    elif (key == ord('t')):
            print("Test");
            cmd = [129,1,6,3,20,20,15,7,15,14,0,1,12,8,255]
            print("cmd is: " + str(cmd))
            cam.open
            cam.write(cmd)
            cam.close
    elif (key == ord('h')):
            print("Home");
            
            cmd = [129,1,6,4,255]
            print("cmd is: " + str(cmd))
            cam.open
            cam.write(cmd)
            cam.close

cap.release()
out.release()

cv2.destroyAllWindows()