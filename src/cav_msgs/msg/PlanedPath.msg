
float64 timestamp   # ROS time in seconds
bool 	Go          # false, true
int16   Estop       # 0:none  1:low  2:medium   3:high

# move forward or backward, both of them should be set to ensure safety
bool forward_flag       #true
bool backward_flag      #false

# note that all of the vectors should be set
# t: must be 0.04 seconds per point, i.e., 25 points per second
# x: degree, in global GNSS frame
# y: degree, in global GNSS frame
# v: vector of desired speed m/s
# heading: vector of heading info, rad, north: pi/2, east:0
# cr: curvature, note + or - for each element
Point[] planedPath



