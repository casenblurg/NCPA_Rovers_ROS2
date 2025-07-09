# Troubleshooting



| Problem                                                                                                                                 | Solution                                                                             |
| --------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| **ROS2 Humble install step**: sudo apt install ros-dev-tools; returns **E: Unable to correct problems, you have held broken packages.** | sudo apt install python3-colcon-common-extensions python3-vcstool python3-rosdep<br> |
| **Compiling MicroXRCE** agent: error in line 99 in **cmakelist.txt**                                                                    | Change 1.12.x to v1.12.2                                                         |
| Not all ros2 topics are being pubished                                                                                                  | [Match Domain ID between computer and pixhawk](https://github.com/casenblurg/NCPA_Rovers_ROS2/blob/main/LT2/uXRCE-DDS.md#domain-id) |

