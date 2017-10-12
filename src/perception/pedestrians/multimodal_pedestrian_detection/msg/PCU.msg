#the header of the message

Header header
uint32 id
geometry_msgs/Point point
uint32 pedestrian_true   #if 1 - pedestrian detected / 0 - pedestrian not found

geometry_msgs/Point initialpose
geometry_msgs/Point finalpose
geometry_msgs/Twist velocity
float64 size