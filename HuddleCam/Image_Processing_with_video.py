### Transcribing blob detection code from my laptop,but with input as video and not set of pics###
import numpy as np
import argparse
import cv2
import serial
from serial import Serial

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

        # cv2.imshow("file", frame)
        # cv2.waitKey(0)
            
        # when we reach the end of the video (file) exit cleanly
        if not ret:
            keep_processing = False;
            
        (B, G, R) = cv2.split(image)
        zero = np.zeros(image.shape[:2], dtype = "uint8")
        blueEmphasis = cv2.merge([B, zero, zero])
        blueGray = image[:,:,0]
        blurred = cv2.GaussianBlur(blueGray, (7, 7), 0)
        (T, bThresh) = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY_INV)
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
        cv2.drawContours(image, cnts, -1, (0, 255, 0), 2)
        (x,y),radius = cv2.minEnclosingCircle(biggestcnt)
        center = (int(x),int(y))
        radius = int(radius)
        cv2.circle(blueTargets,center,radius,(0,0,255),2)

        # out.write(blueTargets)

        cv2.imshow("Output", blueTargets)
    
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
