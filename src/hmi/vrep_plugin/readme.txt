This plugin is an extension of the standard V-REP plugin for ROS (V3.1.3).

It allows for the use of IMU sensors that publish information in topics
of the type sensors_msgs/Imu. 

**********************************
Installation:
**********************************

1. Clone the whole directory to a catkin_ws/src folder;

2. Compile it (catkin_make);

3. Move the generated .so file 'catkin_ws/devel/lib/libv_repExtRos.so' into the V-REP home folder;

4. Launch V-REP and check if the ROS plugin is correctly loaded;
