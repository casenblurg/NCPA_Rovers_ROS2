
# Terminology
This document uses the following terminology. While I will try to be as clear as possible, look here for an exact definition of an unfamiliar term.


| Term            | Definition                                                                                                                                                                                       |
| --------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| uXRCE-DDS       | (aka: *XRCE-DDS*, *DDS*) Middle-ware between the PixHawk and UpBoard$^2$.                                                                                                                        |
| PixHawk         | PixHawk encompasses a wide range of open source flight controllers. The LT2 rovers use [*CUAV x7+*](https://docs.px4.io/main/en/flight_controller/cuav_x7.html#cuav-x7-flight-controller) model. |
| Control Cable   | RS232 to TTL cable. Plugged into COM1 on the UpBoard$^2$ and TELEM1 on the PixHawk. Note, this connection is not currently on the Wiring Diagram.                                                |
| **Debug Cable** | USB A to C cable. The USB A side is plugged into UpBoard$^2$ and the USB C side is plugged into the PixHawk. Mainly used for Debugging, QGC, and flashing firmware onto the PixHawk.             |
| QGC             |  [QGroundControl](https://qgroundcontrol.com/). Used for changing PixHawk settings and accessing the NuttX console.                                                                              |

---
# uXRCE-DDS
Extremely Resource Constrained Environment(s) DDS (or XRCE-DDS), is software that acts as *middle-ware* between the PixHawk (known as the **client**, running on the CUAV x7+) and ROS2 (known as the **agent**, running on the UpBoard$^2$). It operates by publishing uORB topics as ROS2 topics.

For more information, see the PixHawk documentation [here](https://docs.px4.io/main/en/middleware/uxrce_dds.html)

---
## Configuration
The rs232 to TTL cable should be plugged into the TELEM1 port on the PixHawk. This is important to note again as the two different TELEM ports on the pixhawk have two different baud rates by default.


| Port   | Baud Rate |
| ------ | --------- |
| TELEM1 | `57600`   |
| TELEM2 | `115600`  |
<small>I need to double check TELEM2's default baud rate. -Joey</small>


