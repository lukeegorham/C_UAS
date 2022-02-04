import serial
import math
import numpy as np
import cv2
import sys
import time
from serial import Serial

#COMNUM = 18
#cam = serial.Serial('COM25', 9600, timeout=0)
#cam.port = COMNUM -1
#cam.close

# Initialize Camera
#print("Initializing Camera Serial Port Connection...")
#cap  = cv2.VideoCapture(0)
#cam = Serial('/dev/ttyUSB0',9600) # worked on GPU
#cam.close

#cmd = [129,1,4,53,2,255] #outdoor mode = 2
#cam.open
#cam.write(cmd)
#cam.close

# cmd = [129,1,4,57,10,255] #shutter priority
# cam.open
# cam.write(cmd)
# cam.close

#cmd = [129,1,4,57,0,255]  #auto priority
#cam.open
#cam.write(cmd)
#cam.close

#x = 0
#y = 0


def CameraAngleUtm (tx, ty, telv, cx, cy ,celv, cdirection):
	xdist = tx-cx
	ydist = ty-cy
	zdist = telv-celv
	hdeg = math.degrees(math.atan2(xdist, ydist))-cdirection
	hdistance = math.sqrt(xdist**2 + ydist**2)
	vdeg = math.degrees(math.atan2(zdist, hdistance))
	totalDistance = math.sqrt(zdist**2 + hdistance**2)
	zoomfactor = totalDistance/8
	return [hdeg, vdeg, zoomfactor]

def HuddleCamPos (cam, panAngDeg, tiltAngDeg, panSpeed, tiltSpeed):
	panAngPerClick = 0.075
	tiltAngPerClick = 0.079
	tiltRange = [-33.5, 84]
	tiltAngDeg = math.fmod(tiltAngDeg+180, 360) - 180
	if tiltAngDeg > tiltRange[1]:
		tiltAngDeg = tiltRange[1]
	if tiltAngDeg < tiltRange[0]:
		tiltAngDeg = tiltRange[0]
	
	#print(tiltAngDeg)
	panRange = [-171, 184]
	panAngDeg = math.fmod(panAngDeg + 180, 360) - 180
	if (panAngDeg < panRange[0]) and (panAngDeg > (-360 + panRange[1])):
		panAngDeg = panRange[0]
	if panAngDeg <= (-360 + panRange[1]):
		panAngDeg = panAngDeg + 360
	panClick = round(panAngDeg / panAngPerClick)
	panHex = str(hex(int(panClick)))
	
	if panHex[0] == "-":
		panHex = int(panHex[3:],16)
		binaryPanHex = bin(int(panHex))
		numZerosNeeded = 14 - len(binaryPanHex)
		newPanBin = "0b"
		for i in range(numZerosNeeded):
			newPanBin += '1'
		for i in range(2,len(binaryPanHex)):
			if binaryPanHex[i] == "0":
				newPanBin+= '1'
			else:
				newPanBin += '0'
		newPanHex = hex(int(newPanBin,2) + 1)
		newPanHex = newPanHex[2:]
		while len(newPanHex) < 4:
			newPanHex = 'f' + newPanHex
	else:
		panHex = panHex[2:]
		while len(panHex) < 4:
			panHex = '0' + panHex
		newPanHex = panHex
		
	tiltClick = round(tiltAngDeg / tiltAngPerClick)
	tiltHex = str(hex(int(tiltClick)))
	if tiltHex[0] == "-":
		tiltHex = int(tiltHex[3:],16)
		binaryTiltHex = bin(int(tiltHex))
		numZerosNeeded = 14 - len(binaryTiltHex)
		newTiltBin = "0b"
		for i in range(numZerosNeeded):
			newTiltBin += '1'
		for i in range(2,len(binaryTiltHex)):
			if binaryTiltHex[i] == "0":
				newTiltBin+= '1'
			else:
				newTiltBin += '0'
		newTiltHex = hex(int(newTiltBin,2) + 1)
		newTiltHex = newTiltHex[2:]
		while len(newTiltHex) < 4:
			newTiltHex = 'f' + newTiltHex
		tiltHex = newTiltHex
	else:
		tiltHex = tiltHex[2:]
		while len(tiltHex) < 4:
			tiltHex = '0' + tiltHex
			
	cmd = [129, 1, 6, 2, panSpeed, tiltSpeed,
		int(newPanHex[0],16), int(newPanHex[1],16),
		int(newPanHex[2],16), int(newPanHex[3],16),
		int(tiltHex[0],16), int(tiltHex[1],16),
		int(tiltHex[2],16), int(tiltHex[3],16), 255]
	
	#cmd = [129,1,6,2,24,20,15,11,5,0,15,14,11,0,255]
	print("cmd is: " + str(cmd))
	cam.open
	cam.write(cmd)

	cam.close

def HuddleCamZoom(cam,zoomValue):
	baseCnt = 36
	levels = [1, 334/baseCnt]
	if (zoomValue > levels[1]):
		zoomValue = levels[0]
	pVal = [4.343211148793062e-08, -4.984554264045150e-05, 0.021857575650121, -4.553447194454285, 4.856833032085256e+02, -1.252202320377412e+04]
	zoomLev = round(np.polyval(pVal, zoomValue * baseCnt))
	cmdList = [129, 1, 4, 6, 3, 255]
	
	cmd = bytes(cmdList)
	
	cam.open
	cam.write(cmd)
	cam.close
	
	zoomHex = str(hex(int(zoomLev)))
	if (zoomHex[0] == "-"):
		zoomHex = zoomHex[3:]
	else:
		zoomHex = zoomHex[2:]  ## Might throw off zoom with negatives???
	
	while len(zoomHex) < 4:
		zoomHex = '0' + zoomHex
	cmd = [129, 1, 4, 71,int(zoomHex[0],16), int(zoomHex[1],16), 
						int(zoomHex[2],16), int(zoomHex[3],16), 255]
	
	cam.open
	cam.write(cmd)
	cam.close
	
def control(tx, ty, telv, cx, cy ,celv, cdirection):
	[hdeg, vdeg, zoomfactor] = CameraAngleUtm(tx, ty, telv, cx, cy ,celv, cdirection)
	print("CameraAngleUtm done")
	panAngDeg=hdeg
	tiltAngDeg=vdeg
	panSpeed=24
	tiltSpeed=20
	HuddleCamPos(cam,panAngDeg,tiltAngDeg,panSpeed,tiltSpeed)
	print("HuddleCamPos done")
	zoomPow = zoomfactor
	HuddleCamZoom(cam,zoomPow)
	print("HuddleCamZoom done")
	
#################

def blobDetect(camera):
    
    # Initialize Camera
    print("Initializing Camera Serial Port Connection...")
    cap  = cv2.VideoCapture(0)
    cam = Serial('/dev/ttyUSB0',9600) # worked on GPU
    cam.close

    cmd = [129,1,4,53,2,255] #outdoor mode = 2
    cam.open
    cam.write(cmd)
    cam.close


    cmd = [129,1,4,57,0,255]  #auto priority
    cam.open
    cam.write(cmd)
    cam.close

    cam.open
    cmd = [129,1,6,4,255]  # Center camera
    cam.write(cmd)
    cam.close
    
    time.sleep(3) #wait 3 sec
    
    ZoomValue = 0;
    
    print("Out");
    ZoomValue = ZoomValue - 1;
    if (ZoomValue < 0):
        ZoomValue = 0;
    cmd = [129,1,4,71,ZoomValue,ZoomValue,ZoomValue,ZoomValue,ZoomValue,ZoomValue,ZoomValue,ZoomValue,255]
    cam.open
    cam.write(cmd)
    cam.close
    
    keep_processing = True
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
            x = int(x)
            y = int(y)
            radius = int(radius)
            cv2.circle(blueTargets,center,radius,(0,0,255),2)
            if x > 345:
                print("Right");

                cmd = [129,1,6,1,12,12,2,3,255]
                cam.open
                cam.write(cmd)
                cam.close
                
                time.sleep(1)
                
                print("Stop");
                cmd = [129,1,6,1,12,12,3,3,255]
                cam.open
                cam.write(cmd)
                cam.close
                
            elif x < 295:
                print("Left");
                cmd = [129,1,6,1,12,12,1,3,255]
                cam.open
                cam.write(cmd)
                cam.close
                time.sleep(1)
                print("Stop");
                cmd = [129,1,6,1,12,12,3,3,255]
                cam.open
                cam.write(cmd)
                cam.close
                
            elif 295< x < 345:
                print("Stop");
                cmd = [129,1,6,1,12,12,3,3,255]
                cam.open
                cam.write(cmd)
                cam.close

            cv2.imshow("Output", blueTargets)
            
            cam.open
            cmd = [129,1,6,4,255]  # Center camera
            cam.write(cmd)
            cam.close

            key = cv2.waitKey(1) & 0xFF;

def main():
            
	print("Enter main()")
	cameraToUse = 0
	#control(x, y, 0, 0, 0, 0, 0)
	blobDetect(cameraToUse)
	key = cv2.waitKey(1) & 0xFF;
	print("Exit main()")


if __name__ == '__main__':
    main()


