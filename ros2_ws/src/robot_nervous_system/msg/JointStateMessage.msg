# JointStateMessage.msg
# Represents the current state of robot joints (from FR-001: publisher/subscriber nodes)

builtin_interfaces/Time header.stamp
string header_frame_id

string[] name
float64[] position
float64[] velocity
float64[] effort