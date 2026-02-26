# AnomalyMsg.msg
std_msgs/Header header 	    # holds stamp and frame ID

string publisher_name	    # name of file publishing
string source_type          # camera, lidar, mic, etc. 
string sensor_info	    # e.g. sensor location, confidence etc. 

string topic_name           # original topic name

string data_type            # string, wav2, etc.
sensor_msgs/msg/Image image  # image
string description	    # relevant text data

uint8[] data                # catch all



