# trajectory_mapper

An attempt to implement trajectory-based mapping with a particle filter.
Very much work-in-progress.

To install on my Raspberry Pi 5 boards running Ubuntu 24 and built-from-source ros2 iron:
```
scp /opt/ros/iron/lib/libpcl_msgs__rosidl_typesupport_c.so robotics@172.17.14.105:/home/robotics/ros2_iron/install/rosidl_runtime_c/lib
scp /opt/ros/iron/lib/libpcl_msgs__rosidl_typesupport_introspection_c.so robotics@172.17.14.105:/home/robotics/ros2_iron/install/rosidl_runtime_c/lib
scp /opt/ros/iron/lib/libpcl_msgs__rosidl_generator_c.so robotics@172.17.14.105:/home/robotics/ros2_iron/install/rosidl_runtime_c/lib
```
