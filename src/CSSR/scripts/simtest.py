# !usr/bin/env python

import rospy
import time
from cssr.msg import Control #控制消息

if __name__ == "__main__":
    rospy.init_node("Controller")
    pub = rospy.Publisher("/Control",Control,queue_size=10)

    rate=rospy.Rate(1)
    i=1
    while not rospy.is_shutdown():
        con_msg = Control()
        con_msg.timestamp = time.time()
        con_msg.count = i
        i=i+1

        con_msg.steering_cmd_front_wheel = 0.1
        con_msg.gear_cmd = 4
        con_msg.turn_signal_cmd = 0
        con_msg.speed_cmd = 5
        con_msg.acceleration_cmd = 0


        pub.publish(con_msg)
        rospy.loginfo("Message count: %d",i)
        rate.sleep()
    pass
