# AnomalyMsg.msg
std_msgs/Header header 	    # holds stamp and frame ID

string node_name            # name of the publishing node

uint8 INFO=0
uint8 WARNING=1
uint8 ERROR=2
uint8 importance            # either info, warning, or error

uint8 TEXT=0
uint8 IMAGE=1
uint8 DATA=2
uint8 type                  # either text, image, or data

string msg                  # type 0

sensor_msgs/Image image     # type 1

string data_type            # type 2
uint8[] data


