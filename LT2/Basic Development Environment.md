> Note: This documentation already assumes you have a basic understanding of key concepts such as: [*C++*](https://www.w3schools.com/cpp/), [*OOP*](https://en.wikipedia.org/wiki/Object-oriented_programming), [*ROS2*](https://docs.ros.org/en/humble/index.html), as well as other pieces of documentation in this repository. This should be one of the last documents you read.
> 
> Additionally, while Python can be used for all source code, it is notably slower than C++. Thus C++ will be used in this documentation. You should already be aware of the slight [package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#what-makes-up-a-ros-2-package) and [programming](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html) differences between Python and C++.

## Theory

### Assigning Tasks to Nodes
ROS2 allows us to separate different processes into separate nodes. To take full advantage of this, we need to group related tasks that the rover must do into **nodes**. 

Take caution not to separate out tasks *too much* as this will add unnecessary complexity. For example: Arming and Disarming probably don't need separate nodes at the beginning of a project, just as moving left, right, forward, and backwards could also be grouped into one node.

Of course, none of this is a hard and fast rule. For example, if there are a series of steps -- separate from those needed to arm the rover -- it may be a good idea create another node to handle disarming.

### ROS2 Topics
The XRCE-DDS bridge creates `/fmu/in/*` and `/fmu/out/*` topics. As the names hint at, **in topics** handle data going *into the PixHawk*, and **out topics** handle data going *out from the PixHawk*. 

To communicate with `/fmu/*` topics, you must use [`px4_msgs`](https://github.com/PX4/px4_msgs). These messages standardize the variables that are expected by the PixHawk. 

Assuming you have setup your ROS2 workspace properly and the XRCE Agent is running (explained below), you can see all available `/fmu/` topics by running:
```bash
ros2 topic list
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

If `/fmu/out/*` topics do not appear, you may have to wait a few seconds for the Agent to finish subscribing to the `/fmu/out/*` topics. If they still do not show up, please re-read [uXRCE-DDS](https://github.com/casenblurg/NCPA_Rovers_ROS2/blob/main/LT2/uXRCE-DDS.md).

In order to see which `px4_msgs` corresponds to which topic, run the command:
```bash
ros2 topic info <topic>
```

For example, doing this on `/fmu/in/offboard_control_mode` outputs:
```bash
$ ros2 topic info /fmu/in/offboard_control_mode
Type: px4_msgs/msg/OffboardControlMode  
Publisher count: 0  
Subscription count: 1
```

Googling "[OffboardControlMode px4 message](https://docs.px4.io/main/en/flight_modes/offboard.html)" brings us to the message's source code:
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

With these basics now covered, let's go over a code breakdown.

## Code

### Setting up a development environment

> Note: If you already have a ROS2 workspace ready, skip to [Creating the Package](https://github.com/casenblurg/NCPA_Rovers_ROS2/edit/main/LT2/Basic%20Development%20Environment.md#creating-the-package)

To create and build the workspace:

1. Open a new terminal.
2. Create and navigate into a new workspace directory using:

   ```sh
   mkdir -p ~/ws_sensor_combined/src/
   cd ~/ws_sensor_combined/src/
   ```

3. Clone the example repository and [px4_msgs](https://github.com/PX4/px4_msgs) to the `/src` directory (the `main` branch is cloned by default, which corresponds to the version of PX4 we are running):

   ```sh
   git clone https://github.com/PX4/px4_msgs.git
   git clone https://github.com/PX4/px4_ros_com.git
   ```

4. Source the ROS 2 development environment into the current terminal and compile the workspace using `colcon`:


   ```sh
   cd ..
   source /opt/ros/humble/setup.bash
   colcon build
   ```

This builds all the folders under `/src` using the sourced toolchain.

[Source](https://docs.px4.io/main/en/ros2/user_guide.html#building-the-workspace).

## Creating the Package

1. Navigate to `src/`
2. Use `ros2 pkg create` to create a package using `ament_cmake` and set its' dependencies as: `px4_msgs` and `rclcpp`
### The Arm Node
Navigate to the new package's `src/` directory and create a new file called `offboard.cpp`.

Before we write any code, let's list the things we need this node to do.

Node Features:
- Switching Flight Modes
- Arm / Disarm
- Arm Retry

Simple right? Wrong. Each feature requires multiple intermediary steps. Here is a more thorough breakdown.

Node Features (verbose):
- Switching Flight Modes
	- Publish to `/fmu/in/vehicle_command`
	- Create message
	- Only publish message before arm attempt
- Arm
	- At least 10 `offboard_control_mode` and `vehicle_attitude_setpoint` messages need to be published before arm attempt
	- Create arm message
	- Publish to `/fmu/in/vehicle_command`
- Disarm
	- Create disarm message
	- Publish to `/fmu/in/vehicle_command`
	- Switch flight mode 
- Arm Retry
	- Create QoS profile
	- Subscribe to `/fmu/out/vehicle_control_mode`using QoS profile
	- Read `flag_armed` value
	- Retry if value isn't `true`.

Let's begin.

#### Actually Programming (fr fr this time)

First let's write out our includes.

```c++
#include <px4_msgs/msg/offboard_control_mode.hpp>  
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>  
#include <px4_msgs/msg/vehicle_control_mode.hpp>  
#include <px4_msgs/msg/vehicle_command.hpp>  
#include <rclcpp/rclcpp.hpp>  
#include <stdint.h>  
  
#include <chrono>  
#include <iostream>
```

These tell us we'll be using `OffboardControlMode.msg`, `VehicleAttitudeSetpoint.msg`, `VehicleControlMode.msg`, and `VehicleCommand.msg`. The other four includes are necessary for most ROS2 nodes.

Next, let's set up our namespace.
```c++
using namespace std::chrono;  
using namespace std::chrono_literals;  
using namespace px4_msgs::msg;
```

Now, create the `Offboard` class. This class will inherit from `rclcpp::Node`.
```c++
class Offboard : public rclcpp::Node
{
	...
}
```

Let's create some private variables.

We'll need three publishers, one for `OffboardControlMode.msg`, `VehicleAttitudeSetpoint.msg`, and `VehicleCommand.msg`.

```c++
private:
	// publishers
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher;
	rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_publisher;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher;
```

Now we need to define the method signatures for the publishers' methods.
```c++
...
	// methods
	void publish_attitude_setpoint();
	void publish_offboard_control_mode();
	void publish_vehicle_command(uint16_t command, float enabled = 0.0, float mode_id = 0.0);
```

With the method signatures defined, let's write each method.

First, `publish_attitude_setpoint()`. We need this message to pass a fail safe, but the only data that matters is a valid timestamp.
```c++
void Offboard::publish_attitude_setpoint()
{
	VehicleAttitudeSetpoint msg{};
	
	msg.yaw_sp_move_rate = 0.0;
	msg.q_d = {0.0, 0.0, 0.0, 0.0};
	msg.thrust_body = {0.0, 0.0, 0.0};
	
	msg.timestamp = this->getclock()->now().nanoseconds() / 1000;
	
	attitude_setpoint_publisher->publish(msg);
}
```

Most publishing methods follow this formula.
1. Create message
2. Assign data
3. Assign timestamp
4. Publish using shared pointer

The next method requires a bit more explanation. When we want to switch to off-board mode, we need to tell the PixHawk which types of data will be sent to it. These data type are organized in a hierarchy of precedent.

```
position
velocity
acceleration
attitude
body_rate
thrust_and_torque
direct_actuator
```

From [the PixHawk Docs](https://docs.px4.io/main/en/flight_modes/offboard.html#ros-2-messages):
> *"The fields are ordered in terms of priority such that `position` takes precedence over `velocity` and later fields, `velocity` takes precedence over `acceleration`, and so on. The first field that has a non-zero value (from top to bottom) defines what valid estimate is required in order to use offboard mode, and the setpoint message(s) that can be used. For example, if the `acceleration` field is the first non-zero value, then PX4 requires a valid `velocity estimate`, and the setpoint must be specified using the `TrajectorySetpoint` message."*

Knowing that, let's organize the booleans in our message in order of their hierarchy.
```c++
void Offboard::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = true;
	
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	
	offboard_control_mode_publisher->publish(msg);
}
```

The last publisher method is very simple as we're mostly just passing the pass-by-variables in the method signature to the message.

```c++
void Offboard::publish_vehicle_command(uint16_t command, float enabled, float mode_id)
{
	VehicleCommand msg{};
	
	msg.param1 = enabled;
	msg.param2 = mode_id;
	msg.command = command;
	msg.target_system = 1;
	msg.target_commponent = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	
	vehicle_command_publisher->publish(msg);
}
```

Now let's create a public constructor. 

The main job of the constructor is to 