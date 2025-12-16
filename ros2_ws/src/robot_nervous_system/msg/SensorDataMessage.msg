# SensorDataMessage.msg
# Contains sensor readings from the robot (for bridging to AI agents)

builtin_interfaces/Time header.stamp
string sensor_type
string sensor_name
float64[] data
string frame_id