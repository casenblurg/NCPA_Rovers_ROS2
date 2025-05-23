# Documentation Overview

**Introudction**

The point of this documentation is to aid in **setup**, **debugging**, and **creating** software for the Rovers at the National Center for Physical Acoustics ([NCPA](https://olemiss.edu/ncpa/)) in the **Battlefield Acoustics Group**. This documentation covers the software for the two types of rovers at the **NCPA**. 
1. LT2 (Smaller Black Rover)
2. [TR1200](https://docs.trossenrobotics.com/tr1200_docs/) (Larger Blue Rover)

There is only one **TR1200**, while there are multiple **LT2** rovers. Any of this documentation reffering to the **LT2** is specifically reffering to work that has been done on the **LT2-3**.

---

**LT2-3**

*This documentation is under the assumption that Ros2 and Mavros are already installed on the system*

This rover's onboard computer is an [Up Sqaured Pro](https://up-board.org/up-squared-pro/) running **Ubuntu 20.04.1 LTS  (Focal Fossa)**. This specific version was retained to preserve compatibility with existing software developed by a collaborating colleagues. Upgrading the OS could risk introducing dependency conflicts or breaking functionality. If you are looking to upgrade the OS, it is recommended to either consult **Nhat Van/Noah Knutson**, or boot from an external hard drive/make a copy of the current OS. 

The onboard computer communicates to a **Pixhawk CUAV x7** over **USB** interfaced by [Mavros](https://github.com/mavlink/mavros) which will be explained in further detail in the **LT2** section. **Mavros** is a **ROS** package that utilizes the [MavLink](https://github.com/mavlink/mavlink) protocol, this is the standard messaging protocol used by **PX4**.

---

**TR1200**

*This documentation is under the assumption that Ros2 is already installed on the system*

This rover's onboard computer is an [Up Sqaured Pro](https://up-board.org/up-squared-pro/) running **Ubuntu 22.04 LTS  (Jammy Jellyfish)**, and **ROS 2 Humble**. The communication between the rover and onboard computer is over **CAN bus** utilizing a **ROS** package provided by [Trossen Robotics](https://www.trossenrobotics.com/) [(Interbotix)](https://github.com/Interbotix/tr1200_ros). Before controlling the rover remotely, there is some setup that needs to be done which will be explained further in the **TR1200** directory.
