# PoseEvent.msg - Marks pose transitions
std_msgs/Header header
string pose_name
string event_type  # "start" or "end"
int32 pose_number

