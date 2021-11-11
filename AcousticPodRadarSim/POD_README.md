# Acoustic Hub and Pods

### Description:
These pods and hub are part of the GFE package for our capstone. They were designed by Dr. Gruber for the 2020-2021 team and uses microphones and trigonometry to determine the position of an incoming drone. Additionally, these pods are programmed to distinguish between a larger quadcopter and a smaller "racing" drone. 

The hardware within each of the pods is a Raspberry PI unit connected to two microphones. It uses 900MHz telemetry to communicate with the main hub. Each pod is powered by a 11.1V LiPo battery (T-Plug connectors) and a 12V-5V DC converter. 

Inside the hub is a power converter and Raspberry PI unit similar to the pods. However, it uses a higher gain antenna than the pods and connects to the switch via ethernet. It is powered the same way as the pods, but rather than being on the ground, it is mounted on a stand (which is made of a pelican case and a foldable rod).

## Instructions for Setup 
Set up in the field IAW the [field setup guide](../Documentation/FieldOpsInstructions.md)

## Important Notes:
- The 2021-2022 team has never had an issue with the pods, and there should be no reason to change the code
- The 2021-2022 team did change the code on the hub to send the network packets to the GS2 laptop, but no other chances are likely
    

-- Back to [master_documentation](../Documentation/Master_Documentation.md) --

