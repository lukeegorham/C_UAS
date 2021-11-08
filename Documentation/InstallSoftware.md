# Installing Software
* Author: CUAS Capstone Team USAFA C/O 2022
* Date: 2021-2022 School Year

### Description:
This file describes in-depth how to configure software on each subsystem's hardware.


---
## Groundstation Laptop 1:
### Necessary Files:
```?
***ALL PNG FILES***
C2MasterFile.py
GUI_Master.py
LLtoUTM.py
```

### Instructions To Install and Launch C2/GUI System:
1) Download and install PyCharm 
2) Download and install Python 3.7.9 (google it)
3) Set up PyCharm to use Python3.7.9 environment
4) IF ON LINUX:
   1) Download `Install_Script.sh`
   2) `chmod +x Install_Script.sh`
   3) `./Install_Script.sh`
5) IF ON WINDOWS:
   1) Download `Install_Script.bat`
   2) Double-click on Install_Script.bat to run install script
6) In PyCharm, add configuration to run with Python3.7.9 interpreter
7) Configure GS1 to have the following network settings:
   1) Go to Windows 10 Settings -> Network & Internet -> Ethernet
   2) Click Properties, then Edit (under IP Settings)
   3) Set `Manual` Settings:
      1) IPV4 `ON`
      2) `IP Addr:  192.168.1.50`
      3) `Subnet Length: 24`
      4) `Gateway:  192.168.1.1`
      5) `Pref DNS: 192.168.1.1`
   4) Click Save
8) Disable Firewall
   1) Or Allow These Ports (both IN and OUT):
      1) `XXXX [UDP]` - get discovery drone video feed
      2) `XXXX [TCP]` - get discovery drone info
      3) `XXXX [TCP]` - send discovery drone commands
      4) `XXXX [UDP]` - get ground camera video feed
      5) `XXXX [TCP]` - send ground camera commands
9) Run  - `python3 C2_MasterFile.py`
   1) Can be run from either pycharm or through native python
10) Click checkbox on GUI to allow Acoustic Messages to get positions to diplay on the screen

---
## Groundstation Laptop 2:
### Necessary Files:
Copy all of `AcousticPodRadarSim` to laptop. Note that there is a version for simulating output of Acousic Pods and Radar and another which takes "true" values in the field and sends to GS1 to display on the GUI. Run the proper version depending on if you are in the lab or in the field.

### Launching Acoustic/Radar Simulator:
1) Disable Firewall
   1) Or Allow These Ports (both IN and OUT):
      1) `XXXXX [TCP]` - get target position(s) from acoustics
      2) `XXXXX [TCP]` - get status messages from acoustics
      3) `XXXXX [TCP]` - get target position(s) from radar
      4) `XXXXX [TCP]` - get status messages from radar
2) Run `AcousticPodRadarSim/MainProgram/bin/x64/Debug/SoloSimRadar.exe`
[Link](AcousticPodRadarSim/MainProgram/bin/x64/Debug/SoloSimRadar.exe)
