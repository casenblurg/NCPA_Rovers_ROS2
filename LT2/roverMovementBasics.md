# Move rover morward with both motors at 10%: 



```bash
ros2 topic pub --rate 50 /fmu/in/actuator_motors px4_msgs/msg/ActuatorMotors "{timestamp: 0, control: [0.10, 0.10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```


right  left
[0.10, 0.10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
