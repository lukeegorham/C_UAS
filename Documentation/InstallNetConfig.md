# Starting Network From Scratch

### Description:
This file outlines the requirements and procedure for initial setup for the C2 network. DO NOT execute this procedure when simply powering on the system. If powering on the system and using the "default" configuration, use [this](FieldOpsInstructions.md) guide. 

### Hardware Requirements:
- Ground Station Laptops [*windows 10 preferred, linux okay*] **(x2)**
- Raspberry PI [*Raspian OS*] **(x3)**
- Unmanaged Network Switch [*requires 5+ ethernet ports*]
- MicroHard pDDL2450 radios **(x2)**
- Batteries/wiring harnesses to [power](PowerSystem.md) each system
    - Note that the Raspberry PI units require 5V, all other systems require 12V
- Ethernet cords: 
    - 20-foot **(x2)**
    - 6-foot **(x3)**
    - 1-foot **(x1)**
    - 4-inches **(x1)**
- Preconfigured Acoustic Pods (x3+)

## Procedure: (*perform in-lab with wifi connection*)
- Power on MicroHard radios and a GroundStation laptop
- Configure MicroHard Radio #1:
    - Reset each by holding down the button for 10 seconds (then wait ~60sec for reset to take effect)
    - Ensure DHCP ethernet addressing on the GroundStation Laptop 1
    - Connect GS Laptop to MicroHard1 via ethernet cable
    - Open browser on laptop and go to `192.168.168.1`
    - Login using `admin` as both the username and password
    - Follow prompts to reset the admin password (our team used `capstone`)
    - Go to Settings Tab -> LAN and click EDIT configuration
    - Set IP address to `192.168.1.5`
    - Disable off DHCP server
    - Click Submit and then wait ~60sec for changes to take effect
    - Configure laptop as static IPv4 addressing to `192.168.1.10`
    - In browser on laptop, go to `192.168.1.5`
    - Login using **Username:** `admin` and your new password
    - Go to Wireless Tab -> RF
    - Change configuration to `Master`
    - Set parameters as desired (must be duplicated for MicroHard2), see our example for configuration in [this](Network.md) document
    - Click Submit
- Configure MicroHard Radio #2:
    - Ensure DHCP ethernet addressing on the GroundStation Laptop 1
    - Connect GS Laptop to MicroHard1 via ethernet cable
    - Open browser on laptop and go to `192.168.168.1`
    - Login using `admin` as both the username and password
    - Follow prompts to reset the admin password (our team used `capstone`)
    - Go to Settings Tab -> LAN and click EDIT configuration
    - Set IP address to `192.168.1.6`
    - Disable off DHCP server
    - Click Submit and then wait ~60sec for changes to take effect
    - Configure laptop as static IPv4 addressing to `192.168.1.10`
    - In browser on laptop, go to `192.168.1.6`
    - Login using **Username:** `admin` and your new password
    - Go to Wireless Tab -> RF
    - Change configuration to `Slave`
    - Set parameters as desired (must be duplicate of MicroHard1, except "Slave")
    - Click Submit   
- Configure GroundStation Laptops IAW the [install guide](InstallSoftware.md)
    - Double-check IP addressing if laptops configured prior to configuring MicroHard Radios
- Configure Raspberry PI units IAW the [install guide](InstallSoftware.md)
    - Ground Camera
    - Discovery Drone
    - Acoustic Hub
- Connect Radar Unit to switch using a 20-foot ethernet cable
    - Perform any additional setup procedures
- Connect Acoustics Hub to switch using a 20-foot ethernet cable
- Connect each system to switch using 3-feet cables:
    - GroundStation Laptop 1
    - GroundStation Laptop 2
    - Ground Camera Raspberry PI
- Connect MicroHard1 to the switch using a 1-foot cable
- Connect the MicroHard2 to the drone using the 4-inch cable
 
-- Back to [master_documentation](../Documentation/Master_Documentation.md) --
