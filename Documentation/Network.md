# Network Documentation

### Description:
The C2 system consists of two grounstation laptops, one Raspberry PI to control the ground camera, a central Raspberry PI hub (for the acoustic sensors), and one ethernet-to-wifi antenna system (comprised of two MicroHard radios) to control the Discovery Drone.

All computers are organized with static IP addresses and an unmanaged ("dumb") switch is used to connect each system as a hub-and-spoke local network.

Two 12V Lead-Acid batteries are used to power all the groundstation equipment, using the provided wires and adapters. One battery powers the Camera, Raspberry PI (which controls the camera), and the ethernet-wifi antenna. The other battery powers the ethernet switch. The two groundstation laptops run off of their included batteries. 

If network configuration (especially the link to the discovery drone) is lost or needs to be reconfigured from scratch, reference [this](InstallNetConfig.md) file.

## Components:
#### Groundstation Laptop 1: *(Dell ToughBook running Windows 10)*
```?
IP Address: 192.168.1.50/24     (set DNS & Gateway to 192.168.1.1)
Ports:
    56555 - ??? List
    55565 - [TCP] Receive messages from Simulated Radar
    46554 - [46555] [TCP] Send Camera commands to Camera Controller
    46554 - [5566?] [UDP] Receive Camera feed grom Camera Controller
    44555 - [65536] [TCP] Receive Discovery Drone information
    65536 - [45454] [5666] [TCP] Send commands to Discovery Drone
    XXXXX - [TCP] Receive target information from GS2
    XXXXX - [TCP] Receive acoustic messages from Acoustic Hub
```

#### Groundstation Laptop 2: *(Dell ToughBook running Windows 10)*
```?
IP Address: 192.168.1.51/24     (set DNS & Gateway to 192.168.1.1)
Ports:
    56555 - List
```

#### Camera Controller: *(Raspberry PI running Raspian)*
```?
IP Address: 192.168.1.26/24     (set DNS & Gateway to auto, disable IPv6)
Ports:
    46554 - [TCP] Listen for commands from GS1
    46554 - [UDP] Send video feed to GS1
```

#### Acoustics Hub: *(Raspberry PI running Raspian)*
```?
IP Address: 192.168.1.31/24     (set DNS & Gateway to auto, disable IPv6)
Ports:
    XXXXX - [TCP] Sends target location to GS2
                Messages sent every XXX seconds for targets
                System health messages sent every XXX seconds
```

#### Radar Unit: *(L-Star)*
```?
IP Address: 192.168.1.XX/24
Ports:
    XXXXX - Sends locations of targets to GS2
```

#### Ethernet-Drone Master Radio: *(MicroHard pDDL2450 Connected to main switch)*
```?
LAN IP Address: 192.168.1.5/24     (set DNS & Gateway to 192.168.1.1)
RF Data: *MASTER*
     Frequency - Ch22 / 2423MHz
     Bandwidth - 8MHz
     Power - 30dBm
     Auth - AES128  Password: capstone

Login:
     Username: admin
     Password: capstone
```

#### Ethernet-Drone Slave Radio: *(MicroHard pDDL2450 Connected to Discovery Drone Raspberry PI)*
```?
LAN IP Address: 192.168.1.6/24     (can set DNS & Gateway to 192.168.1.1)
RF Data: *SLAVE*
     Frequency - Ch22 / 2423MHz
     Bandwidth - 8MHz
     Power - 30dBm
     Auth - AES128  Password: capstone

Login:
     Username: admin
     Password: capstone
```

#### Discovery Drone:
```?
IP Address: 192.168.1.25/24
Ports:
   44555 - [UDP] send status/pos/video to GS1
    5666 - Unused... [TCP] receive messages from GS1?
   65536 - [TCP] receive commands from GS1
```

## Sending Messages to Discovery Drone:
The GS1 laptop can directly "see" the Discovery Drone's Raspberry PI unit through the MicroHard radios. Essentially, these radios are able to emulate a wired ethernet connection between the Raspberry PI on the drone to the switch. Therefore, any hardware can send IP packets to the drone via its IP address `192.168.1.25`.

## Debugging:
Debugging is very difficult with networking. But there are some key things to diagnosing your network.

1) Ensure that each component has the proper static IP address
2) Ensure that the firewall is off or ports are allowed for what you want to do
3) Control+F your files to ensure that you are binding and sending to the right addresses
   1) Python sometimes doesn't like to bind to blank addresses `''` so it's better to specify your own address so it doesn't try and bind the loopback address
4) Ping the address you are trying to connect to (IOT ping Windows devices, you have to allow it in the firewall)

## Misc Notes:
* The IP addresses each have `/24` appended to indicate `Netmask: 255.255.255.0`
* The acoustic pods are linked via 900MHz telemetry
* If launching a script via ssh -> call command, you cannot open a cv2 window on the server from a login shell. Instead, comment out those lines (in the server code) and ensure that you are sending the video feed to view on a cv2 window on the client.

-- Back to [master_documentation](../Documentation/Master_Documentation.md) --