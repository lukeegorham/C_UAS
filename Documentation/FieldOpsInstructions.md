# Field Operating Instructions:
* Author: CUAS Capstone Team USAFA C/O 2022
* Date: 2021-2022 School Year

---
## Prerequisites:
* Installed all programs/scripts on hardware, reference the [installation guide](InstallSoftware.md)
* Procured sufficient means of powering each system in the field, see [power](PowerSystem.md)
* Configured static IP addresses for each subsystem in the [network guide](Network.md)
* Allowed TCP/UDP ports for each subsystem IAW [proper network configuration](InstallNetConfig.md)

---
## Field Setup:
1) Place and power on Acoustics Hub
   1) Power is provided using a 12V LiPo battery
   2) Connect ethernet cord to port on hub and to switch
2) Place and power on Acoustic Pods
   1) Each require a 12V LiPo battery
3) Set up camera on tripod and face towards enemy
   1) Three cords connect
      1) Power: 12V center-positive bullet connector (connect to harness)
      2) Serial port: 8-pin to USB TypeA - plug into `IN` port on back (connect to PI)
      3) Imagery cable: USB3.0 TypeB to USB3.0 TypeA (connect to PI)
   2) Use the level on the camera's base to level the unit
4) Connect power to Ethernet Switch
5) Turn on both Groundstation Laptops
   1) Password to both computers is `dfec`
6) Connect ethernet cables:
   1) Check Acoustics Pod connected to Switch
   2) Connect GS1 to Switch (Yellow)
   3) Connect GS2 to Switch (Long Orange)
   4) Connect PI to Switch  (Blue)
   5) Connect POE Box {LAN Port} to Switch (Short Orange)
   6) Connect POE Box {POE Port} to Bullet (Blue)
7) Connect power harness to PI, Camera, and POE
   1) Remember to flip the switch on the camera to `ON`
      1) NOTE: Camera will go through a boot-up initialization sequence
      2) NOTE: It has helped in the past to cover the camera during boot-up
   2) Ensure fan is spinning on PI
   3) Ensure lights are on for both the POE and the Bullet Antenna

---
## Launch:
1) Initialize Acoustic Sensors:
   1) SSH into Acoustics HUB
      1) `IP Addr:  192.168.1.31`
      2) `Username: pi`
      3) `password: dfec`
      4) `python3 ____________________ -______`
2) Initialize Ground Camera Feed Server
   1) SSH into Camera Controller
      1) `IP Addr:  192.168.1.26`
      2) `Username: pi`
      3) `Password: dfec`
      4) `python3 GndCam_Networking.py`
3) Launch Acoustic/Radar Simulator
   1) Run on GS2 computer
   2) Shortcut is on desktop
   3) Ensure this is not the Simulator or Test version
4) Launch C2/GUI System
   1) Run on GS1 computer
   2) Shortcut is on desktop
5) Click to enable acoustic data on GUI and Acoustic/Radar Simulator