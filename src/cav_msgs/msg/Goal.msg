
##timestamp, ros time or utc time
float64     timestamp

#part 1: direct commands
int16 	Estop    # 0:none  1:low  2:medium   3:high
bool 	Go       # false, true

#part 2 -- lane center and drivable area
RefPoint[] refPath

