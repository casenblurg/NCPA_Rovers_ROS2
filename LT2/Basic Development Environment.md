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

