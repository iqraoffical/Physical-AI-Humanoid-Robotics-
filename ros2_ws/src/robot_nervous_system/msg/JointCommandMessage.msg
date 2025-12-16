# JointCommandMessage.msg
# Commands for robot joint control (from FR-004: Python agents send joint commands)

builtin_interfaces/Time stamp

string[] joint_names
float64[] positions
float64[] velocities
float64[] efforts