# Anomaly_log.msg
builtin_interfaces/Time stamp

string node_name            # name of node publishing
string source               # camera, lidar, mic, etc. 
string description          # relevant info, e.g. front camera

string topic_name           # original topic name
string data_type            # image, float32, pointcloud, etc. 
uint8[] data                # relevant data



