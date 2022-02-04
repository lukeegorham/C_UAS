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

cap  = cv2.VideoCapture(0)
cam = Serial('/dev/ttyUSB0',9600) # worked on GPU
cam.close

cmd = [129,1,4,53,2,255] #outdoor mode = 2
cam.open
cam.write(cmd)
cam.close

cmd = [129,1,4,57,10,255] #shutter priority
cam.open
cam.write(cmd)
cam.close

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
	
def humanDetect(camera):
	cap  = cv2.VideoCapture(camera)
	
	# initialize the list of class labels MobileNet SSD was trained to
	# detect, then generate a set of bounding box colors for each class
	CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
		"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
		"dog", "horse", "motorbike", "person", "pottedplant", "sheep",
		"sofa", "train", "tvmonitor"]
	
	COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))
	
	# load our serialized model from disk
	print("[INFO] loading model...")
	#	net = cv2.dnn.readNetFromCaffe(args["prototxt"], args["model"])
	net = cv2.dnn.readNetFromCaffe("MobileNetSSD_deploy.prototxt.txt", "MobileNetSSD_deploy.caffemodel")
	
	keep_processing = True
	personDetected = False
	
	while (keep_processing == True and personDetected == False):
		# load the input image and construct an input blob for the image
		# by resizing to a fixed 300x300 pixels and then normalizing it
		# (note: normalization is done via the authors of the MobileNet SSD
		# implementation)
		#	image = cv2.imread(args["image"])
		if (cap.isOpened):
					ret, image = cap.read();
				
					# when we reach the end of the video (file) exit cleanly
					if (ret == 0):
						keep_processing = False;
	
		(h, w) = image.shape[:2]
		blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)
		
		# pass the blob through the network and obtain the detections and
		# predictions
		# 	print("[INFO] computing object detections...")
		net.setInput(blob)
		detections = net.forward()
		
		# loop over the detections
		for i in np.arange(0, detections.shape[2]):
			# extract the confidence (i.e., probability) associated with the
			# prediction
			confidence = detections[0, 0, i, 2]
		
			# filter out weak detections by ensuring the `confidence` is
			# greater than the minimum confidence
			if confidence > (0.85):  #args["confidence"]
				# extract the index of the class label from the `detections`,
				# then compute the (x, y)-coordinates of the bounding box for
				# the object
				idx = int(detections[0, 0, i, 1])
				box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
				(startX, startY, endX, endY) = box.astype("int")
		
				# display the prediction
				label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
				#	print("[INFO] {}".format(label))
				if (CLASSES[idx] == "person"):
					personDetected = True
				cv2.rectangle(image, (startX, startY), (endX, endY),
					COLORS[idx], 2)
				y = startY - 15 if startY - 15 > 15 else startY + 15
				cv2.putText(image, label, (startX, y),
					cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
		
		# show the output image
		cv2.imshow("Output", image)
		
		key = cv2.waitKey(1) & 0xFF
		if (key == ord('x')):
				keep_processing = False
	
	return personDetected
		
#################


def main():
	print("Enter main()")
	cameraToUse = 0
	x = 1000
	y = 0
	control(x, y, 0, 0, 0, 0, 0)
	
	keep_looping = True
	while(keep_looping == True):
		if(humanDetect(cameraToUse) == True):
			if (x < 180):
				x += 50
			else:
				x = 0
			control(x, y, 0, 0, 0, 0, 0)
			time.sleep(1)
		else:
			x = x

		key = cv2.waitKey(1) & 0xFF;
		if (key == ord('x')):
			keep_looping = False;
		
	print("Exit main()")


if __name__ == '__main__':
    main()
