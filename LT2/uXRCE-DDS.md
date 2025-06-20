
# Terminology
This document uses the following terminology. While I will try to be as clear as possible, look here for an exact definition of an unfamiliar term.


| Term          | Definition                                                                                                                                                                                       |
| ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| uXRCE-DDS     | (aka: *XRCE-DDS*, *DDS*) Middle-ware between the PixHawk and UpBoard<sup>2</sup>.                                                                                                                        |
| PixHawk       | PixHawk encompasses a wide range of open source flight controllers. The LT2 rovers use [*CUAV x7+*](https://docs.px4.io/main/en/flight_controller/cuav_x7.html#cuav-x7-flight-controller) model. |
| Control Cable | RS232 to TTL cable. Plugged into COM1 on the UpBoard<sup>2</sup> and TELEM1 on the PixHawk. Note, this connection is not currently on the Wiring Diagram.                                                    |
| Debug Cable   | USB A to C cable. The USB A side is plugged into UpBoard<sup>2</sup> and the USB C side is plugged into the PixHawk. Mainly used for Debugging, QGC, and flashing firmware onto the PixHawk.                 |
| QGC           | [QGroundControl](https://qgroundcontrol.com/). Used for changing PixHawk settings and accessing the NuttX console.                                                                               |

---
# uXRCE-DDS
Extremely Resource Constrained Environment(s) DDS (or XRCE-DDS), is software that acts as *middle-ware* between the PixHawk (known as the **client**, running on the CUAV x7+) and ROS2 (known as the **agent**, running on the UpBoard<sup>2</sup>). It operates by publishing uORB topics as ROS2 topics.

![visualization](https://docs.px4.io/main/assets/architecture_xrce-dds_ros2.DXSOuyOh.svg)
<sub>Image Credit: PixHawk</sub>

For more information, see the PixHawk documentation [here](https://docs.px4.io/main/en/middleware/uxrce_dds.html)

---
## Configuring the PixHawk
Unfortunately, categories are not sorted in any meaningful way in QGC. You can search for parameters individually so those will also be noted when applicable.
### Baud Rates

Category: `Serial`

The rs232 to TTL cable should be plugged into the TELEM1 port on the PixHawk. This is important to note again as the two different TELEM ports on the pixhawk have two different baud rates by default.

Serial ports must be "in-use" in order to be displayed in the `Serial` category. (See [Enabling the uXRCE-DDS Client](#enabling-the-uxrce-dds-client))


| Port   | PARAM           | Baud Rate |
| ------ | --------------- | --------- |
| TELEM1 | *SER_TEL1_BAUD* | `57600`   |
| TELEM2 | *SER_TEL2_BAUD* | `921600`  |

Needed baud rate on TELEM1: `TBD`

### Enabling the uXRCE-DDS Client

Category: `UXRCE-DDS Client` (inside "Standard" folder)

By default, the `UXRCE_DDS_CFG` parameter is set to `Disabled`, this should be set to `TELEM1`.

### Configuring the Client

#### Domain ID

Category: `UXRCE-DDS Client` (inside "System" folder)
> Note: This category won't show up unless [`UXRCE_DDS_CFG` has been set](#enabling-the-uxrce-dds-client).

The `UXRCE_DDS_DOM_ID` variable is set to `0` by default. You'll need to change this to whatever you set `ROS_DOMAIN_ID` on a given rover. 

Current domain IDs per rover:

| Rover | Domain ID |
| ----- | --------- |
| Leo   | `10`      |

For more information about domain IDs, see [the ROS2 docs](https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html).

#### Timestamp synchronization

Category: `UXRCE-DDS Client` (inside "System" folder)
> Note: This category won't show up unless [`UXRCE_DDS_CFG` has been set](#enabling-the-uxrce-dds-client)

 **THIS IS CURRENTLY A HACK, it has LIMITED KNOWN effects**

The parameter `UXRCE_DDS_SYNCT` has been changed from `Default` to `Disabled` in order to get `/fmu/out/*` topics to show.

`UXRCE_DDS_SYNCT` handles timestamp synchronization, this may cause issues in the future.

## Setting Up the Bridge
When [configured correctly](#configuring-the-pixhawk)