# Documentation Overview     


**Introudction**

The point of this documentation is to aid in **setup**, **debugging**, and **creating** software for the Rovers at the National Center for Physical Acoustics ([NCPA](https://olemiss.edu/ncpa/)) in the **Battlefield Acoustics Group**. This documentation covers the software for the two types of rovers at the **NCPA**. 
1. LT2 (Smaller Black Rover)
2. [TR1200](https://docs.trossenrobotics.com/tr1200_docs/) (Larger Blue Rover)

There is only one **TR1200**, while there are multiple **LT2** rovers. Any of this documentation reffering to the **LT2** is specifically reffering to work that has been done on the **LT2-3**.

---

**LT2-3**


This rover's onboard computer is an [Up Sqaured Pro](https://up-board.org/up-squared-pro/) running **Ubuntu 22.04 LTS  (Jammy Jellyfish)** and **ROS2 Humble**. 

The onboard computer communicates to a **Pixhawk CUAV x7** over **USB** interfaced by [XRCE](https://docs.ncnynl.com/en/px4/en/middleware/uxrce_dds.html) which will be explained in further detail in the **LT2** section. Details [here](https://github.com/casenblurg/NCPA_Rovers_ROS2/blob/main/LT2/uXRCE-DDS.md).

---

**TR1200**

*This documentation is under the assumption that Ros2 is already installed on the system*

This rover's onboard computer is an [Up Sqaured Pro](https://up-board.org/up-squared-pro/) running **Ubuntu 22.04 LTS  (Jammy Jellyfish)** and **ROS2 Humble**. The communication between the rover and onboard computer is over **CAN bus** utilizing a **ROS** package provided by [Trossen Robotics](https://www.trossenrobotics.com/) [(Interbotix)](https://github.com/Interbotix/tr1200_ros). Before controlling the rover remotely, there is some setup that needs to be done which is be explained further in the **TR1200** directory.
