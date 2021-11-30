# Installing Software

### Description:
This file includes initial software installation instructions for Ground Station 1 laptop, Ground Station 2 laptop, ground camera Raspberry PI, drone Raspberry PI, and the acoustic hub Raspberry PI units. 

## Ground Station Laptop 1:
### Necessary Files:
- C2MasterFile.py
- GUI_Master.py
- LLtoUTM.py
- ALL PNG FILES (in GroundStation Directory)

### Instructions To Install and Launch C2/GUI System:
1) **OPTIONAL** - Download and install PyCharm 
2) Download and install [Python 3.7.9]()
3) Set up PyCharm to use Python3.7.9 environment
4) IF ON LINUX:
   1) [Download](../GroundStation/Install/Install_Libraries.sh) `Install_Libraries.sh`
   2) `chmod +x Install_Libraries.sh`
   3) `./Install_Libraries.sh`
5) IF ON WINDOWS:
   1) [Download](../GroundStation/Install/Install_Libraries.bat) `Install_Libraries.bat`
   2) Double-click on Install_Libraries.bat to run install script
        1) NOTE: Must run this script as an administrator 
6) **IF INSTALLED** - In PyCharm, add configuration to run with Python3.7.9 interpreter
7) **WINDOWS 10:** Configure GS1 to have the following network settings:
   1) Go to Windows 10 Settings -> Network & Internet -> Ethernet
   2) Click Properties, then Edit (under IP Settings)
   3) Set `Manual` Settings:
      1) IPV4 `ON`
      2) `IP Addr:  192.168.1.50`
      3) `Subnet Length: 24`
      4) `Gateway:  192.168.1.1`
      5) `Pref DNS: 192.168.1.1`
   4) Click Save
8) **LINUX:**
    1) Configure ethernet settings as seen in step 7 
9) Firewall Settings (*can entirely disable if desired - no internet connectivity required*)
   1) Allow These Ports (both IN and OUT, both TCP and UDP):
      1) `XXXX [UDP]` - get discovery drone video feed
      2) `XXXX [TCP]` - get discovery drone info
      3) `XXXX [TCP]` - send discovery drone commands
      4) `XXXX [UDP]` - get ground camera video feed
      5) `XXXX [TCP]` - send ground camera commands
10) Run  - `python3 C2_MasterFile.py`
    1) Can be run from either pycharm or through CLI python3
11) Click checkbox on GUI to allow Acoustic Messages to get positions to diplay on the screen

## Ground Station Laptop 2:
### Necessary Files:
Copy all of `AcousticPodRadarSim` to laptop. Note that there is a version for simulating output of Acousic Pods and Radar and another which takes "true" values in the field and sends to GS1 to display on the GUI. Run the proper version depending on if you are in the lab or in the field.

### Launching Acoustic/Radar Simulator:
1) Firewall Settings (*can entirely disable if desired - no internet connectivity required*)
   1) Allow These Ports (both IN and OUT):
      1) `XXXXX [TCP]` - get target position(s) from acoustics
      2) `XXXXX [TCP]` - get status messages from acoustics
      3) `XXXXX [TCP]` - get target position(s) from radar
      4) `XXXXX [TCP]` - get status messages from radar
2) Run `AcousticPodRadarSim/MainProgram/bin/x64/Debug/SoloSimRadar.exe`

-- Back to [master_documentation](../Documentation/Master_Documentation.md) --