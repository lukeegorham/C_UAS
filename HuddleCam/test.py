
import cv2
import numpy as np
import glob
from timeit import default_timer as timer
# import imutils

start = timer()
files = glob.glob('Data/DronePics/Drone_*.png')
RESULT_IMG_PATH = "./BlobResults/"
x = 0
a = 0

for filename in files:
    frame = cv2.imread(filename)
    a = a + 1
    # cv2.imshow("file", frame)
    # cv2.waitKey(0)
    (B, G, R) = cv2.split(frame)
    zero = np.zeros(frame.shape[:2], dtype = "uint8")
    blueEmphasis = cv2.merge([B, zero, zero])
    blueGray = frame[:,:,0]
    blurred = cv2.GaussianBlur(blueGray, (7, 7), 0)
    # (T, bThresh) = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY_INV)
    (T, bThresh) = cv2.threshold(blurred, 75, 255, cv2.THRESH_BINARY_INV)
    (cnts, hierarchy) = cv2.findContours(bThresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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

    M = cv2.moments(biggestcnt)
    # print(M)
    blueTargets = frame.copy()
    cv2.drawContours(frame, cnts, -1, (0, 255, 0), 2)
    (x,y),radius = cv2.minEnclosingCircle(biggestcnt)
    center = (int(x),int(y))
    radius = int(radius)
    cv2.circle(blueTargets,center,radius,(0,0,255),2)
    cv2.imshow("blueTargets", blueTargets)
    cv2.waitKey(0)
    cv2.imwrite(RESULT_IMG_PATH + 'Blob' + str(a) + '.png', blueTargets)
print("My program took", timer() - start, "to run")


