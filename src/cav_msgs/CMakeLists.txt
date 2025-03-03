

cmake_minimum_required(VERSION 2.8.3)
project(cav_msgs)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	message_generation
	std_msgs
)
                      

add_message_files( DIRECTORY msg FILES
    #Vehicle Related
	Control.msg
	VehicleState.msg
	Gpybm.msg
    PlanedPath.msg
    Point.msg
    RefPoint.msg
    Goal.msg

    # Perception Related 
    Obstacle.msg
    ObstacleVec.msg
)
 
generate_messages(DEPENDENCIES
	std_msgs
)

catkin_package(CATKIN_DEPENDS	
	message_runtime
	std_msgs
)


