# RobotState.msg
int32 robot_id
geometry_msgs/Point position
geometry_msgs/Twist velocity
sensor_msgs/BatteryState battery
sensor_msgs/Temperature temperature
sensor_msgs/LaserScan obstacle
std_msgs/Bool task_engaged