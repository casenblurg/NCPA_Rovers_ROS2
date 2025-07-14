# LT2 Rover Documentation - Table of Contents

This guide provides detailed documentation for the LT2 rover platform used at NCPA, covering everything from hardware setup to software configuration and troubleshooting.

---

## [1. Control Cable](https://github.com/casenblurg/NCPA_Rovers_ROS2/blob/main/LT2/Control_Cable/The%20Control%20Cable.md)
**Overview:**  
RS232 to TTL cable. Plugged into COM1 on the UpBoard2 and TELEM1 on the PixHawk. Includes enclosure CAD file, 3D print file, description, and pinout.

![CadImage](https://github.com/casenblurg/NCPA_Rovers_ROS2/blob/main/LT2/Figures/RS232_Case_Picture.JPG)

---

## [2. Local Area Network](https://github.com/casenblurg/NCPA_Rovers_ROS2/blob/main/LT2/LocalAreaNetwork/setup.md)
**Overview:**  
The connection between the ground station (Ubiquiti radio dish) and the rovers antennas.

### Local Machine Connection

| Step                            | Setting               |
|---------------------------------|------------------------|
| IP Address                      | 192.168.2.10           |
| Subnet Mask                     | 255.255.255.0          |
| Gateway                         | 192.168.2.1            |
| Ubiquiti Dish Connection        | POE injector (POE port)|
| Local Machine Connection        | POE injector (LAN port)|


### Rover Setup

| Step            | Setting             |
|-----------------|---------------------|
| IP Address      | 192.168.2.rover#    |
| Subnet Mask     | 255.255.255.0       |
| Gateway         | 192.168.2.1         |

---

## [3. Troubleshooting](https://github.com/casenblurg/NCPA_Rovers_ROS2/blob/main/LT2/Troubleshooting/README.md)
**Overview:**  
A reference for diagnosing and fixing reocurring hardware and software issues encountered during LT2 operation. 

---

## [4. UpBoard Mount](https://github.com/casenblurg/NCPA_Rovers_ROS2/tree/main/LT2/UpBoardMount)
**Overview:**  
Mechanical design and mounting instructions for securing the UpBoard computer onto the LT2 chassis. Includes CAD files, print files, and reference image.

![Upboard mount Image](https://github.com/casenblurg/NCPA_Rovers_ROS2/blob/main/LT2/UpBoardMount/HoleDrillGuideReference.png)

---

## [5. Basic Development Environment](https://github.com/casenblurg/NCPA_Rovers_ROS2/blob/main/LT2/Basic%20Development%20Environment.md)
**Overview:**  
Sets up a standard ROS 2 development environment tailored for the LT2 rover. Includes installation steps for dependencies and recommended development practices.

---

## [6. uXRCE-DDS](https://github.com/casenblurg/NCPA_Rovers_ROS2/blob/main/LT2/uXRCE-DDS.md)
**Overview:**  
Details the setup and usage of Micro XRCE-DDS for lightweight communication between the rover's microcontroller and ROS 2 nodes, enabling message passing.

---

