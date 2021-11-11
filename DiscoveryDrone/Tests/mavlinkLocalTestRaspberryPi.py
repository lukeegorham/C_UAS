#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 29 08:31:38 2021

@author: pi
"""

# example code to be included in Raspberry Pi script for controlling autopilot 
# and also setting up comms for ground station

import sys
#from UavMavlink import *
from UavMavlinkV2 import *  # make sure version V2
from time import sleep
import numpy as np

def Main():
    verbose=True  # set to true to see Mavlink setup messages
    try:
        print("setting up mavLink server threads")
        InitMavlink(verbose)  # setup mavLink connection and UDP servers
        # parameters for testing
        testLat=38.99725
        testLon=-104.604280
        testAlt=np.zeros((3,1))
        testAlt[0,0]=2226
        testAlt[1,0]=2216
        testAlt[2,0]=2206
        flightSpeed=5
        tilt=25
        toggle=-1
        cntr=0
        while True:
            # example to demonstrate camera tilt command        
            SendTilt(tilt)
            tilt*=toggle  # used to alternate tilt angles for test
            # example to set camera tilt control back to RC pilot
#            SetGimbalRc()
            # example to set ROI for camera
#            SetRoi(testLat,testLon,testAlt[cntr%3,0])
#            print("Current location: lat: %f, Lon: %f, Alt: %f",(currentLat_Deg,currentLon_Deg,currentAlt_mMSL))
            sleep(5)
#            cntr+=1 # used to toggle through ROI altitudes
            # example to demonstrate fly to waypoint
#            Fly2Wp(testLat,testLon,testAlt[0,0],flightSpeed)
#            sleep(5)
            # example to demonstrate return to launch
#            RTL()            
    except (KeyboardInterrupt):
        raise
    except:
        e=sys.exc_info()[0]
        print(e)
    closeMavlink()
        


if __name__ == '__main__':
    Main()
