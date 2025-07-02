# **OLD MAVROS STUFF**



# LT2 PX4 Connection Overview 

This guide explains how to set up and connect the onboard computer of an **LT2 rover** to **PX4** using **MAVROS**. It also includes useful commands for operation. Please note that this guide assumes **ROS 2** and **MAVROS** are already installed and configured. 

**Ensure Rover is ON**

Start by accessing the onboard computer, whether that is through **SSH** (See: [**NCPA_Rovers_ROS2/SSH_Into_Rovers/README.md**](https://github.com/casenblurg/NCPA_Rovers_ROS2/blob/main/SSH_Into_Rovers/README.md)) or other means. Before launching **MAVROS**, you need to create and configure a `.yaml` parameter file. This file should either be placed in the same directory where you will run the **MAVROS** node, or you must specify its full path in the command.

A minimal working example of the `.yaml` file (mavros_param.yaml) is shown below:
```yaml
# mavros_param.yaml
mavros:
  ros__parameters: {}

mavros_router:
  ros__parameters: {}

mavros_node:
  ros__parameters:
    fcu_url: serial:///dev/serial/by-id/usb-CUAV_PX4_CUAV_X7Pro_0-if00:57600 
```

The `fcu_url` parameter in this file tells **MAVROS** how to connect to the **PX4**, in this case we are using a serial connection via **USB**. Therefore, the path points to the **PX4's** **USB** connection on the the system.

When the `.yaml` file is configured, run the command:

```bash
ros2 run mavros mavros_node --ros-args --params-file ./mavros_param.yaml
```
When you run this command you should **ROS2** related outputs to the terminal.

---
# Useful Commands

After running **MAVROS**, access another terminal in the onboard computer and use these commands as needed:

---
`ros2 topic echo /mavros/state`: 

`ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"`

 `ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'MANUAL'}"`
 










