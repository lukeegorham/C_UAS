# C2 Network
* Author: CUAS Capstone Team USAFA C/O 2022
* Date: 2021-2022 School Year

---
### DESCRIPTION:
> The C2 system consists of two grounstation laptops, one Raspberry PI to control the ground camera, a central Raspberry PI hub (for the acoustic sensors), and one ethernet-to-wifi antenna to control the Discovery Drone.

>All computers are organized with static IP addresses and an unmanaged ("dumb") switch is used to connect each system as a hub-and-spoke local network.

>Two 12V Lead-Acid batteries are used to power all the groundstation equipment, using the provided wires and adapters. One battery powers the Camera, Raspberry PI (which controls the camera), and the bullet wifi antenna POE unit. The other battery powers the ethernet switch. The two groundstation laptops run off of their included batteries. 

---
### COMPONENTS:
#### Groundstation Laptop 1: *(Dell ToughBook)*
```?
IP Address: 192.168.1.50/24     (can set DNS, Gateway to 192.168.1.1)
Ports:
    56555 - List
```

#### Groundstation Laptop 2: *(Dell ToughBook)*
```?
IP Address: 192.168.1.51/24     (can set DNS, Gateway to 192.168.1.1)
Ports:
    56555 - List
```

#### Camera Controller: *(Raspberry PI)*
```?
IP Address: 192.168.1.26/24
Ports:
    56555 - [TCP] Listen for commands from GS1
    XXXXX - [UDP] Send video feed to GS1
```

#### Acoustics Hub: *(Raspberry PI)*
```?
IP Address: 192.168.1.31/24     (can set DNS, Gateway to 192.168.1.1)
Ports:
    XXXXX - [TCP] Sends target location to GS2
                Messages sent every XXX seconds for targets
                System health messages sent every XXX seconds
```

#### Radar Unit: *(L-Star)*
```?
IP Address: 192.168.1.XX/24     (can set DNS, Gateway to 192.168.1.1)
Ports:
    XXXXX - Sends locations of targets to GS2
```

#### Ethernet-Wifi Antenna: *(Ubiquiti Bullet M2)*
```?
IP Address: 192.168.1.XX/24     (can set DNS, Gateway to 192.168.1.1)
Ports:
    XXXXX - Sends locations of targets to GS2
```

---
### SENDING MESSAGES TO DISCOVERY DRONE:
> General paragraph explaining methods.

---
### DEBUGGING:
Debugging is very difficult with networking. But there are some key things to diagnosing your network.

1) Ensure that each component has the proper static IP address
2) Ensure that the firewall is off or ports are allowed for what you want to do
3) Control+F your files to ensure that you are binding and sending to the right addresses
   1) Python sometimes doesn't like to bind to blank addresses `''` so it's better to specify your own address so it doesn't try and bind the loopback address
4) Ping the address you are trying to connect to (IOT ping Windows devices, you have to allow it in the firewall)

---
### Misc Notes:
* The IP addresses each have `/24` appended to indicate `Netmask: 255.255.255.0`
* The acoustic pods are linked via 900MHz telemetry
* POE - Power Over Ethernet, used to power the ethernet-wifi antenna
* If launching a script via ssh -> call command, you cannot open a cv2 window on the server from a login shell. Instead, comment out those lines (in the server code) and ensure that you are sending the video feed to view on a cv2 window on the client.