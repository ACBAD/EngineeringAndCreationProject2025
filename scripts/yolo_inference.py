#!/usr/bin/env python3
#coding=utf-8

import time
import rospy
from eac_pkg.msg import VrResult

if __name__ == "__main__":
    rospy.init_node("yolo_node")
    rospy.logwarn("yolo_node started")
    pub = rospy.Publisher("/visual_data", VrResult, queue_size=2)
    while not rospy.is_shutdown():
        rospy.loginfo("Start inference")
        time.sleep(1)
