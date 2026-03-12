# AnomalyMsg.msg
std_msgs/Header header 	    # timestamp and frame reference
string node_name            # name of publishing node

uint8 INFO=0
uint8 WARNING=1
uint8 ERROR=2
uint8 importance            # either info, warning, or error

uint8 TEXT=0
uint8 IMAGE=1
uint8 DATA=2
uint8 type                  # either text, image (+ optional msg) or data (+ optional msg)

string msg                  # may be accompanied by any type

sensor_msgs/Image image     # used when type == IMAGE

string data_type            # used when type == DATA
uint8[] data


