"""

"""
# Import Python System Libraries

import time
import sys
import serial.tools.list_ports  # used to identify 
import serial
from time import sleep
from subprocess import call

# GPS settings
hDopMin=6
            
def main():

    # get system time and location from GPS
    getGpsData() # set system time and pod location
    print('lat: '+str(latitude_deg)+', lon: '+str(longitude_deg)+', alt: '+str(elevation_m_MSL))

    # your code goes here
    while True:
        try:
            print()
        except:
            e = sys.exc_info()[0]
            print("Time sync: <p>Error: %s</p> \n" % e)
        time.sleep(100)
        

################### GPS Functionality
def getGpsData():
    portNotConnected=True
    
    # look at all serial devices, then search for GPS receiver
    ports=list(serial.tools.list_ports.comports())
    for p in ports:
        # find if Global Sat GPS receiver is present
        if ('USB-Serial' in p.description):
            ser = serial.Serial(p.device, baudrate = 4800, timeout = 0.5)
            portNotConnected=False
            
    if portNotConnected:
        print('No GPS.  Check connection.')
        pass
        
    print("Receiving GPS data")
    sleep(30.0) # wait for GPS to setup
    global GpsLock
    global GpsTime
    global dateGps
    GpsLock=False
    GpsTime=False
    dateGps=""
    # keep trying GPS until time is received and HDop meets threshold
    while (GpsLock==False):
        try:
            data = ser.readline().decode()
            #print(data)
            parseGPS(data)
        except (KeyboardInterrupt):
            raise        
        except:
            e = sys.exc_info()[0]
            print("Time sync: <p>Error: %s</p> \n" % e)
            pass

def parseGPS(data):
    global latitude_deg
    latitude_deg=0.0
    global longitude_deg
    longitude_deg=0.0
    global elevation_m_MSL
    elevation_m_MSL=0.0
    global dateUtc
    global dateGps

    global GpsLock
    global GpsTime    
    
    if data[0:6] == "$GPRMC":
        sdata = data.split(",")
        if sdata[2] == 'V':  # this may occur if its been awhile since the GPS was used and the alamanac needs to be loaded
            print("no satellite data available")
            return
        print("---Parsing GPRMC---"),
        dateGps=("20"+sdata[9][4:6]+"-"+sdata[9][2:4]+"-"+sdata[9][0:2])
        GpsTime=True

    if (data[0:6] == "$GPGGA" and GpsTime==True):
        sdata = data.split(",")
        # verify hDOP within tolerance
        print("hDOP: %s" % sdata[8])
        if(float(sdata[8])<hDopMin):
            print("---Parsing GPGGA - min hDop met---"),
            # set system time to GPS (note that this is UTC time)
            if(dateGps != ""):  # test so system date is only set once
                dateUtc="sudo date --set '"+dateGps+" "+sdata[1][0:2]+":"+sdata[1][2:4]+":"+sdata[1][4:]+"'"
                call(dateUtc, shell=True)
                print(dateUtc)
                
            temp=int(float(sdata[2])/100.0)  #get degrees
            temp2=(float(sdata[2])-float(temp*100))/60.0  #get decimal degrees
            latitude_deg=float(temp)+temp2
            if(sdata[3]=='S'):  # south is negative
                latitude_deg=-latitude_deg

            temp=int(float(sdata[4])/100.0)  #get degrees
            temp2=(float(sdata[4])-float(temp*100))/60.0  #get decimal degrees
            longitude_deg=float(temp)+temp2
            if(sdata[5]=='W'): # west is negative
                longitude_deg=-longitude_deg
                
            elevation_m_MSL=float(sdata[9])
            GpsLock=True
    print("GPS lock: %s" % GpsLock)
    
if __name__ == '__main__':
    main()
