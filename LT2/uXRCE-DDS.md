
# Terminology
This document uses the following terminology. While I will try to be as clear as possible, look here for an exact definition of an unfamiliar term.


| Term              | Definition                                                                                                                                                                                       |
| ----------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| uXRCE-DDS         | (aka: *XRCE-DDS*, *DDS*) Middle-ware between the PixHawk and UpBoard<sup>2</sup>.                                                                                                                |
| PixHawk           | PixHawk encompasses a wide range of open source flight controllers. The LT2 rovers use [*CUAV x7+*](https://docs.px4.io/main/en/flight_controller/cuav_x7.html#cuav-x7-flight-controller) model. |
| [Control Cable]() | RS232 to TTL cable. Plugged into COM1 on the UpBoard<sup>2</sup> and TELEM1 on the PixHawk. Note, this connection is not currently on the Wiring Diagram.                                        |
| Debug Cable       | USB A to C cable. The USB A side is plugged into UpBoard<sup>2</sup> and the USB C side is plugged into the PixHawk. Mainly used for Debugging, QGC, and flashing firmware onto the PixHawk.     |
| QGC               | [QGroundControl](https://qgroundcontrol.com/). Used for changing PixHawk settings and accessing the NuttX console.                                                                               |

---
# uXRCE-DDS
Extremely Resource Constrained Environment(s) DDS (or XRCE-DDS), is software that acts as *middle-ware* between the PixHawk (known as the **client**, running on the CUAV x7+) and ROS2 (known as the **agent**, running on the UpBoard<sup>2</sup>). It operates by publishing uORB topics as ROS2 topics.

![visualization](https://docs.px4.io/main/assets/architecture_xrce-dds_ros2.DXSOuyOh.svg)
<sub>Image Credit: PixHawk</sub>

For more information, see the PixHawk documentation [here](https://docs.px4.io/main/en/middleware/uxrce_dds.html)

---
# Configuring the PixHawk
Unfortunately, categories are not sorted in any meaningful way in QGC. You can search for parameters individually so those will also be noted when applicable.
> Note that this document only covers changes regarding the `uxrce_dds_client`. For information regarding basic PixHawk setup, such as Air Frame, Actuator, and GPS setup, see (TODO: MAKE DOC!)
## Baud Rates

Category: `Serial`

The rs232 to TTL cable should be plugged into the TELEM1 port on the PixHawk. This is important to note again as the two different TELEM ports on the pixhawk have two different baud rates by default.

Serial ports must be "in-use" in order to be displayed in the `Serial` category. (See [Enabling the uXRCE-DDS Client](#enabling-the-uxrce-dds-client))


| Port   | PARAM           | Baud Rate |
| ------ | --------------- | --------- |
| TELEM1 | *SER_TEL1_BAUD* | `57600`   |
| TELEM2 | *SER_TEL2_BAUD* | `921600`  |

Needed baud rate on TELEM1: `TBD`

## Enabling the uXRCE-DDS Client

Category: `UXRCE-DDS Client` (inside "Standard" folder)

By default, the `UXRCE_DDS_CFG` parameter is set to `Disabled`, this should be set to `TELEM1`.

## Configuring the Client

### Domain ID

Category: `UXRCE-DDS Client` (inside "System" folder)
> Note: This category won't show up unless [`UXRCE_DDS_CFG` has been set](#enabling-the-uxrce-dds-client).

The `UXRCE_DDS_DOM_ID` variable is set to `0` by default. You'll need to change this to whatever you set `ROS_DOMAIN_ID` on a given rover. 

Current domain IDs per rover:

| Rover | Domain ID |
| ----- | --------- |
| Leo   | `10`      |

For more information about domain IDs, see [the ROS2 docs](https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html).

### Timestamp synchronization

Category: `UXRCE-DDS Client` (inside "System" folder)
> Note: This category won't show up unless [`UXRCE_DDS_CFG` has been set](#enabling-the-uxrce-dds-client)

 **THIS IS CURRENTLY A HACK, it has LIMITED KNOWN effects**

The parameter `UXRCE_DDS_SYNCT` has been changed from `Default` to `Disabled` in order to get `/fmu/out/*` topics to show.

`UXRCE_DDS_SYNCT` handles timestamp synchronization, this may cause issues in the future.

---
# Setting Up the Bridge

## Understanding the Client
When [configured correctly](#configuring-the-pixhawk), the `uxrce_dds_client` is launched at boot in this format:
```nsh
nsh> uxrce_dds_client start -t serial -d /path/to/TELEMx -b SER_TELx_BAUD
```

Where `x` is the telemetry port that you have set up.

While it is possible to launch and configure `uxrce_dds_client` to use the [debug cable](#terminology), this is not advised since you will locked out of QGC. This means that if you need to change a PixHawk parameter, you'll need to re-flash the firmware and set up all the parameters again.

You can confirm that `uxrce_dds_client` is online but running this command in the NuttX shell:
```nsh
nsh> uxrce_dds_client status
```


## The Agent

### Compiling the Agent
As the UpBoard<sup>2</sup>s run Ubuntu 20.04, you'll need to compile the agent, `MicroXRCEAgent`, from source. Additionally, since the compiler also needs cmake versions `[3.20.0, 3.30.0)`, I recommend using version 3.25.3.

> Note: It may be easier to clone Leo, Lilly, or another working rover's entire drive onto the rover you are working on with `dd`. (Since I have yet to discuss this idea with the rest of the team, I'll wait to document the compilation process as it is rather lengthy. -Joey)

Once `uxrce_dds_client` is online, you can launch the agent. In the UpBoard<sup>2</sup>, run:
```bash
$ sudo MicroXRCEAgent serial -D /dev/tty(TBD) -b (TBD)
```

If launched correctly, you should see:
```bash
[1750434557.048426] info     | TermiosAgentLinux.cpp | init                     | running...             | fd: 3  
[1750434557.048980] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4  
[1750434558.104060] info     | Root.cpp           | create_client            | create                 | client_key: 0x00000001, session_id: 0x81  
[1750434558.104207] info     | SessionManager.hpp | establish_session        | session established    | client_key: 0x00000001, address: 1  
[1750434558.437796] info     | ProxyClient.cpp    | create_participant       | participant created    | client_key: 0x00000001, participant_id: 0x001(1)  
[1750434558.659029] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x00000001, topic_id: 0x800(2), participant_id: 0x001(1)
```

Note, this will eventually stop after a second or two, this is normal behavior.

If there is an error, you'll normally be stuck at this screen:
```bash
[1750434067.819842] info     | TermiosAgentLinux.cpp | init                     | running...             | fd: 3  
[1750434067.820371] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4
```

Check you baud rate (both the PixHawk and command), and which Telemetry port was selected if this happens. 

