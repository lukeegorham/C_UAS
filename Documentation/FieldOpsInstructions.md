# Field Operating Instructions:

### Description:
This file includes prerequisites and procedures for real-world execution of our system.


## Prerequisites:
* Installed all programs/scripts on hardware, reference the [software install guide](InstallSoftware.md)
* Procured sufficient means of powering each system in the field, see [power guide](PowerSystem.md)

## Necessary Hardware:
- Ground Station Laptops **(x2)**
- Ethernet Switch
- Ground Camera Unit:
    - Raspberry PI Controller
    - Camera with gimbal mounted on tripod
    - USB3.0 TypeA to USB3.0 TypeB
    - USB2.0 TypeA to 8-Pin Serial Connector
    - Power cord (12V center-positive bullet connector)
- Acoustics System:
    - Hub Unit
    - Stand
    - Antenna
    - Pods: each with own antenna **(x3+)**
- MicroHard pDDL2450 unit
- Batteries/wiring harnesses to power each system
    - 12V Lead-Acid batteries with adapters to female T-Plug Connectors **(x2)**
    - 11.1V LiPo batteries **(x1 for Acoustics Hub, 1ea for Acoustic Pods)**
    - Power adapter (outputs 12V and 5V from 12V input) with parallel connectors
- Ethernet cables: 
    - 20-foot **(x2)**
    - 6-foot **(x3)**
    - 1-foot **(x1)**
    - 4-inches **(x1)**
- Magnetic Compass (used to set up acoustic pods)

## Setup:
1) Place and power on Acoustics Hub
   1) Power is provided using a 12V LiPo battery
   2) Connect ethernet cable to port on hub and to switch
2) Place and power on Acoustic Pods
   1) Each require a 12V LiPo battery
3) Set up camera on tripod and face towards enemy
   1) Three cable connect
      1) Power: 12V center-positive bullet connector (connect to harness)
      2) Serial port: 8-pin to USB TypeA - plug into `IN` port on back (connect to PI)
      3) Imagery cable: USB3.0 TypeB to USB3.0 TypeA (connect to PI)
   2) Use the level on the camera's base to level the unit
4) Connect power to Ethernet Switch
5) Turn on both GroundStation Laptops
   1) Password to both computers is `dfec`
6) Connect ethernet cables:
   1) Check Acoustics Pod connected to Switch (20' ethernet cable)
   2) Connect Radar Unit to Switch (20' ethernet cable)
   3) Connect GS1 to Switch (6' ethernet cable)
   4) Connect GS2 to Switch (6' ethernet cable)
   5) Connect PI to Switch  (6' ethernet cable)
   6) Connect MicroHard1 to Switch (1' ethernet cable)
7) Connect power harness to PI, Camera, and POE
   1) Remember to flip the switch on the camera to `ON`
      1) NOTE: Camera will go through a boot-up initialization sequence
      2) NOTE: It has helped in the past to cover the camera during boot-up
   2) Ensure fan is spinning on PI
   3) Ensure lights are on for both the POE and the Bullet Antenna


## Launch:
1) Initialize Acoustic System:
   1) SSH into Acoustics HUB from GroundStation 1 Laptop using PuTTY
      1) `IP Addr:  192.168.1.31`
      2) `Username: pi`
      3) `password: dfec`
      4) `python3 ____________________ -______`
2) Initialize Ground Camera System
   1) SSH into Camera Controller from GroundStation 1 Laptop using PuTTY
      1) `IP Addr:  192.168.1.26`
      2) `Username: pi`
      3) `Password: dfec`
      4) `python3 GndCam_Networking.py`
3) Initialize Discovery Drone
    1) SSH into Drone from GroundStation 1 Laptop using PuTTY
        1) `IP Addr: 192.168.1.25`
        2) `Username: pi`
        3) `Password: dfec`
        4) `cd WorkingImages/`
        5) `python3 Drone_Master.py`
4) Launch Acoustic/Radar Simulator
   1) Run on GS2 computer
   2) Shortcut is on desktop
   3) Ensure this is not the Simulator version
5) Launch C2/GUI System
   1) Run on GS1 computer
   2) Shortcut is on desktop
6) Click to enable acoustic data in GUI and Acoustic/Radar Simulator

-- Back to [master_documentation](../Documentation/Master_Documentation.md) --