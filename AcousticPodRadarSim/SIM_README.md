# Acoustic Simulator

### Description:
This program is a part of the GFE package for our capstone. It was writen by Dr. Gruber for the 2020-2021 team and simulates the messages sent by the radar unit so that we can practice reading these messages with our own program. Currently, this program is configured to run on the GroundStation 2 Laptop and sends messages to the GroundStation 1 Laptop to be displayed on the GUI.

It is critical to note that the simulator forwards the packets from the acoustic hub to the GroundStation 1 Laptop.

## Instructions for Using with Sensor Data 
To run the program:
- Go to `AcousticPodRadarSimAsterixV9 -> MainProgram -> bin -> x64 -> Debug`
- Run `SoloSimRadar.exe`

## Instructions for Simulating Sensor Data
To run the program:
- Go to `AcousticPodRadarSimAsterixV9 -> MainProgramSimulator -> bin -> x64 -> Debug`
- Run `SoloSimRadar.exe`

## Important Notes:
- Code files edited in `MainProgram` and `MainProgramSimulator` must be maintained identical
- Use Visual Studio to edit and recompile these files after editing code or configuration files
- The `App.config` files are xml files used at compile-time to change the function of the compiled binaries
    - Changing these files will require re-building the program for changes to take effect
    

-- Back to [master_documentation](../Documentation/Master_Documentation.md) --

