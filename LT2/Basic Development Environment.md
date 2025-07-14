> Note: This documentation already assumes you have a basic understanding of key concepts such as: [*C++*](https://www.w3schools.com/cpp/), [*OOP*](https://en.wikipedia.org/wiki/Object-oriented_programming), [*ROS2*](https://docs.ros.org/en/humble/index.html), as well as other pieces of documentation in this repository. This should be one of the last documents you read.
> 
> Additionally, while Python can be used for all source code, it is notably slower than C++. Thus C++ will be used in this documentation. You should already be aware of the slight [package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#what-makes-up-a-ros-2-package) and [programming](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html) differences between Python and C++

## Theory

### Assigning Tasks to Nodes
ROS2 allows us to separate different processes into separate nodes. To take full advantage of this, we need to group related tasks that the rover must do into nodes. 

Take caution not to separate out tasks *too much* as this will add unnecessary complexity. For example: Arming and Disarming probably don't need separate nodes at the beginning of a project, just as moving left, right, forward, and backwards could also be grouped into one node.

Of course, none of this is a hard and fast rule. For example, if there are a series of steps -- separate from those needed to arm the rover -- it may be a good idea create another node to handle disarming.

### ROS2 Topics
The XRCE-DDS bridge creates `/fmu/in/*` and `/fmu/out/*` topics. As the names hint at, **in topics** handle data going *into the PixHawk*, and **out topics** handle going *out from the PixHawk*. 

To communicate with `/fmu/*` topics, you must use [`px4_msgs`](https://github.com/PX4/px4_msgs). These messages standardize the variables that are expected by the PixHawk. 

Assuming you have setup your ROS2 workspace properly and the XRCE Agent is running (explained below), you can see all available `/fmu/` topics by running:
```bash
$ ros2 topic list
```

You should see an output like this:
```
/fmu/in/actuator_motors  
/fmu/in/actuator_servos  
/fmu/in/arming_check_reply  
/fmu/in/aux_global_position  
/fmu/in/config_control_setpoints  
/fmu/in/config_overrides_request  
/fmu/in/differential_drive_setpoint  
/fmu/in/goto_setpoint  
/fmu/in/manual_control_input  
/fmu/in/message_format_request  
/fmu/in/mode_completed  
/fmu/in/obstacle_distance  
/fmu/in/offboard_control_mode  
/fmu/in/onboard_computer_status  
/fmu/in/register_ext_component_request  
/fmu/in/sensor_optical_flow  
/fmu/in/telemetry_status  
/fmu/in/trajectory_setpoint  
/fmu/in/unregister_ext_component  
/fmu/in/vehicle_attitude_setpoint  
/fmu/in/vehicle_command  
/fmu/in/vehicle_command_mode_executor  
/fmu/in/vehicle_mocap_odometry  
/fmu/in/vehicle_rates_setpoint  
/fmu/in/vehicle_thrust_setpoint  
/fmu/in/vehicle_torque_setpoint  
/fmu/in/vehicle_trajectory_bezier  
/fmu/in/vehicle_trajectory_waypoint  
/fmu/in/vehicle_visual_odometry  
/fmu/out/battery_status  
/fmu/out/estimator_status_flags  
/fmu/out/failsafe_flags  
/fmu/out/manual_control_setpoint  
/fmu/out/position_setpoint_triplet  
/fmu/out/sensor_combined  
/fmu/out/timesync_status  
/fmu/out/vehicle_attitude  
/fmu/out/vehicle_control_mode  
/fmu/out/vehicle_gps_position  
/fmu/out/vehicle_local_position  
/fmu/out/vehicle_odometry  
/fmu/out/vehicle_status  
/parameter_events  
/rosout
```

If `/fmu/out/*` topics do not appear, you may have to wait a few seconds for the Agent to finish subscribing to the `/fmu/out/*` topics. If they still do not show up, please re-read [uXRCE-DDS].

In order to see which `px4_msgs` corresponds to which topic, run the command:
```bash
$ ros2 topic info <topic>
```

For example, doing this on `/fmu/in/offboard_control_mode` outputs:
```bash
$ ros2 topic info /fmu/in/offboard_control_mode
Type: px4_msgs/msg/OffboardControlMode  
Publisher count: 0  
Subscription count: 1
```

Googling "OffboardControlMode px4 message" brings us to the message's source code:
```
# Off-board control mode

uint64 timestamp		# time since system start (microseconds)

bool position
bool velocity
bool acceleration
bool attitude
bool body_rate
bool thrust_and_torque
bool direct_actuator
```

We can use these variables in code:
```C++
// needed to be able to access the message
#include <px4_msgs/msg/offboard_control_mode.hpp>
...

OffboardControlMode msg{};
msg.position = true;
msg.velocity = false;
msg.timestamp = custom_get_time();

...
```

Notice that not every variable needs to be instantiated and they do not need to be in the order of the source code.

> Note: Messages can routinely be *100+ lines long!* However, you'll generally find that most of these lines are **constant definitions and instantiations.** For example, here is a constant from the `VehicleCommand.msg`:
> ```
> uint16 VEHICLE_CMD_DO_SET_MODE = 176 # Set system mode.
> ```
> In order to use this constant do:
> ```c++
>VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
>```

