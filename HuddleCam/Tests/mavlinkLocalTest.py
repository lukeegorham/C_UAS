#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 29 08:31:38 2021

@author: pi
"""

import sys
from UavMavlink import *
from time import sleep

def Main():
    verbose=True
    try:
        print("setting up mavLink server threads")
        InitMavlink(verbose)  # setup mavLink connection and UDP servers
        # parameters for testing
        testLat=39.1234
        testLon=-104.5678
        testAlt=2222.2
        while True:
            # your code here.  Sleep is just a space keeper        
#            SendTilt(-25)
#            SetRoi(testLat,testLon,testAlt)
            sleep(5)
#            Fly2Wp(testLat,testLon,testAlt,5)
#            sleep(5)
#            RTL()            
    except (KeyboardInterrupt):
        raise
    except:
        e=sys.exc_info()[0]
        print(e)
    closeMavlink()
        

    


if __name__ == '__main__':
    Main()
