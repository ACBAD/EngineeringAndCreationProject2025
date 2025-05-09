#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import json

l_data = []
r_data = []
freeze_count = 0
non_zero = False
defualt_l_data = None
defualt_r_data = None

def l_callback(msg: Int32):
    global non_zero, freeze_count, defualt_l_data
    if defualt_l_data is None:
        defualt_l_data = msg.data
        return
    if msg.data != defualt_l_data:
        non_zero = True
    if non_zero:
        if l_data and l_data[-1] == msg.data:
            freeze_count += 1
        l_data.append(msg.data)

def r_callback(msg: Int32):
    global non_zero, freeze_count, defualt_r_data
    if defualt_r_data is None:
        defualt_r_data = msg.data
        return
    if defualt_r_data != msg.data:
        non_zero = True
    if non_zero:
        if r_data and r_data[-1] == msg.data:
            freeze_count += 1
        r_data.append(msg.data)


if __name__ == "__main__":
    rospy.init_node("wheel_observer")
    rospy.logwarn("node init")
    sub_l = rospy.Subscriber('/lwheel_ticks', Int32, l_callback)
    sub_r = rospy.Subscriber('/rwheel_ticks', Int32, r_callback)
    rospy.logwarn("observe")
    while not rospy.is_shutdown():
        if freeze_count >= 20:
            rospy.logwarn("frozen")
            with open("wheel_ticks.json", 'w') as f:
                json.dump({'l_data': l_data, 'r_data': r_data}, f)
            break
