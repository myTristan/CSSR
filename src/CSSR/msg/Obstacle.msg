float64 	timestamp   #ros time
int32 		id
int16 		type        #refer to below definition       
float64 	size_x
float64 	size_y
float64 	size_z

float64 	x_global   #m, in the same coordinate system of the ego car
float64 	y_global   #m, in the same coordinate system of the ego car
float64 	h_global   #rad, heading, east:0 north:pi/2

float64 	x_local    #m, relative to ego car
float64 	y_local    #m, relative to ego car

float64 	speed_x    #not official definition

#future path
#OnePath[] pathVec       #all possible paths

## define object type
#define OBJECT_TYPE_INVALID             0
#define OBJECT_TYPE_UNKNOWN             1
#define OBJECT_TYPE_NOT_AVAILABLE       2
#define OBJECT_TYPE_UNCLASSIFIED        3
#define OBJECT_TYPE_UNDECIDED_SMALL     4
#define OBJECT_TYPE_UNDECIDED_BIG       5
#define OBJECT_TYPE_BARRIER             6
#define OBJECT_TYPE_PEDESTRIAN          7
#define OBJECT_TYPE_BIKE                8
#define OBJECT_TYPE_CAR                 9 
#define OBJECT_TYPE_TRUCK               10 
#define OBJECT_TYPE_TRAIN               11 
#define OBJECT_TYPE_ROAD_CONSTRUCTION   12 
#define OBJECT_TYPE_RUN_RED_LIGHT       13 
