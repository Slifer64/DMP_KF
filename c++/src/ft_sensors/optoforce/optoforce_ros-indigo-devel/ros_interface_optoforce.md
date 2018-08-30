# ForceDimension ROS interface

## optoforce_topic node

All interfaces are defined as topics, but might be a good idea to use services instead of subscribed topics due to their function.

### Published topics
- `/wrench_deviceSN`:  publishes the Wrench of the Optoforce device defined with its Serial Number. `Force in Newtons`, and `Torque in Newtown.meter`. ROS message type are `geometry_msgs/WrenchStamped`

### Subscribed topics
- `/start_publishing`:  subscribes to a command to start publishing Wrench data. ROS message type are `std_msgs/Bool`. 
- `/start_new_acquisition`:  subscribes to a command to start a new acquisition. ROS message type are `std_msgs/Bool`. 
- `/reset`:  subscribes to a command to re-make the Zero of the OptoForce device. ROS message type are `std_msgs/Bool`. 