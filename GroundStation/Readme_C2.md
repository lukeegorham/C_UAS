# C2 System Readme

### Description:
The C2 system is the integral functionality of this project. It involves the network connection in each subsystem which allows the system as a whole to function. Additionally, it interfaces with the [Sensor Fusion](../Documentation/SensorFusion.md) and [Image Processing](../Documentation/ImageProcessing.md) modules to maximize system efficiency.

Management of the C2 System requires meticulous documentation as there are many details which must be tracked and properly accounted for. Changing any one piece of code or functionality might require editing several code files, and documentation should be continuously updated every time the code changes. Our team did not deem it necessary to maintain a changelog for these files, but frequent commits and verbose commit messages are good methods for tracking which changes might have caused an issue should it arise.

The "owner" of the C2 System development should work in close conjunction with the owner of the Graphical User Interface (GUI) functionality. The GUI requires myriad variables and data structures for its own functionality, but are maintained and updated by the C2 System. Use of the "Find and Replace" feature of PyCharm is recommended to prevent unintentional errors from changing names or architecture of data structures.


## Hardware Components:
* Ethernet Switch
* Ground Station Laptop 1   (GS1)
* Ground Station Laptop 2   (GS2)
* Ground Camera RaspPI      (CAM)
* Microhard Radio 1         (MR1)
* Microhard Radio 2         (MR2)
* Discovery Drone RaspPI    (DPI)


## Software Components:
### GS1 Software:
* C2_MasterFile.py
* GUI_Master.py
* capstone2018sensor_fusion_2.py
* LLtoUTM.py
* Pictures - see [GUI Readme](Readme_GUI.md)

### GS2 Software:
* Acoustic/Radar [Simulator](../AcousticPodRadarSim/SIM_README.md)

### [Network Configuration](../Documentation/InstallNetConfig.md):
* See [network](../Documentation/Network.md) for more details

### Table Of Connection Management Files
| Send-Recv | GS1 | GS2 | CAM | DPI |
| GS1 | N | N | [Y](../Tests/GS_Camera.py)-[Y](../Tests/GndCam_Network.py) | [Y](UavMavlinkV52.py)-[Y](C2_MasterFile.py) |
| GS2 | [Y](../AcousticPodRadarSim/SIM_README.md)-N | N | SSH | SSH |
| CAM | [Y](../Tests/GS_Camera.py)-[Y](../Tests/GS_Camera.py) | N | N | N |
| DPI | [Y](../DiscoveryDrone/DroneMaster.py)-[Y](../GroundStation/UavMavlinkV52.py) | N | N | N |


## Subsystem Functionality:
### Ground Station 1:
This computer runs the master script `C2_MasterFile.py`. This script invokes multiple threads and other scripts/programs to achieve full functionality. Essentially, this subsystem creates and maintains links to the other systems to receive drone data and display on the integrated GUI.

### Ground Station 2:
This computer runs the Acoustic/Radar Simulator, and is used to log into the Raspberry PI units and manage the status of the other subunits. The Simulator takes acoustic data and telemetry data (from the enemy "target" drone, for the sake of simulating a radar unit only), and sends GS1 an array of targets, which GS1 will display on the GUI.

### Camera:
The Raspberry PI unit maintains a serial interface with the camera-gimbal unit (to point and zoom the camera). It takes the video feed and sends images to GS1 via the ethernet switch, and receives aiming commands from GS1.

### Discovery Drone:
The Raspberry PI unit onboard the Discovery Drone connects to our network bia the Microhard radio link. It connects to MR2 via a short ethernet cord, and the microhard radio system simulates an ethernet cord, allowing computers plugged into the switch to "see" the drone's PI unit as if it were plugged directly into the switch.
 
The PI unit sends a data structure to GS1 every ___ seconds, which includes its position/velocity/heading as well as a video image (after [image processing](../Documentation/ImageProcessing.md)). GS1 then decodes the data structure, displays the video feed, and updates the discovery drone's position for the GUI to display.

## Power:
Reference the [power](../Documentation/PowerSystem.md) document for more detailed instructions. Two 12V lead-acid batteries are used to power the C2 system on the ground, in addition to the self-powered laptops.

The Ethernet Switch has its own lead-acid battery for power in the field (tongue connectors on the lead battery -> T-connector -> 4-pin connector -> center-positive bullet). In the lab, connect the manufacturer's bullet plug to the power port.

The GS1 and GS2 laptops have two batteries each, batteries are hot-swappable and >5hrs of runtime is possible under ideal conditions.

The Ground camera system is powered by two 4-pin connectors (from the Battery2 system). The 5V +/- pins on one connector power the Raspberry PI unit, and the 12V +/- pins on the second connector power the camera-gimbal unit. In the lab, use the same connectors, but power with a 12V center-positive bullet power supply with an additional connector: bullet[female] to T-connector[female]. See [power](../Documentation/PowerSystem.md) for more details. 

The MR1 unit is powered through a 4-pin connector and uses only the 12V +/- pins. Power in the lab and in the field in the same manner as for the camera system.

The Discovery Drone Raspberry PI and MR2 units are powered by the drone's own connectors and converter from the battery. Disconnect the propellers and the motor board (yellow plug after the Y-connetor) if powering on in the lab. The PI unit will power the autopilot system.

-- Back to [master_documentation](../Documentation/Master_Documentation.md) --