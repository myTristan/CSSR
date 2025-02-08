# !usr/bin/env python

import rospy
import math
# import time
from cssr.msg import Control #控制消息

if __name__ == "__main__":
    rospy.init_node("Controller")
    pub = rospy.Publisher("/Control",Control,queue_size=10)

    rate=rospy.Rate(25) #25 Hz
    i=1
    type = 1
    while not rospy.is_shutdown():
        con_msg = Control()
        con_msg.timestamp = rospy.get_time()
        con_msg.count = i
        i=i+1

        if type == 0:# Circle
            con_msg.steering_cmd_front_wheel = 0.03
            con_msg.gear_cmd = 4
            con_msg.turn_signal_cmd = 0
            con_msg.speed_cmd = 20
            con_msg.acceleration_cmd = 0
        elif type == 1:# Sine wave
            con_msg.steering_cmd_front_wheel = 0.03*(-1)**((int)(i/50))
            con_msg.gear_cmd = 4
            con_msg.turn_signal_cmd = 0
            con_msg.speed_cmd = 20
            con_msg.acceleration_cmd = 0
        elif type == 2:# Sine wave
            con_msg.steering_cmd_front_wheel = 0.2
            con_msg.gear_cmd = 4
            con_msg.turn_signal_cmd = 0
            con_msg.speed_cmd = 5
            con_msg.acceleration_cmd = 0

        pub.publish(con_msg)
        rospy.loginfo("count: %5d, speed: %6.3f, steer: %6.3f",i,con_msg.speed_cmd,con_msg.steering_cmd_front_wheel)
        rate.sleep()
    pass
