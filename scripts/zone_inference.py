#!/usr/bin/env python3
#coding=utf-8

import time
from typing import cast
import cv2
from std_msgs.msg import UInt8
import rospy
from eac_pkg.msg import ZoneInfo
from yolov8n import inference, init_nn
parent_dir = '/home/khadas/catkin/src/EngineeringAndCreationProject2025/scripts'
import numpy as np

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 640
DISTANCE_SCALE = 1.0
ANGLE_SCALE = 1.0

last_log_time = time.time()
ips = 0

def limitValue(val: float):
    if val > 1:
        return 1
    elif val < 0:
        return 0
    return val
running_state = False
def set_running_state(target_state: UInt8):
    global running_state
    if target_state.data:
        running_state = True
    else:
        running_state = False

if __name__ == "__main__":
    rospy.init_node("zone_info_node")
    try:    
        side_color = rospy.get_param('side_color')
    except KeyError:
        rospy.logfatal('Please set side_color!!!')
        exit(1)
    init_nn(nn_lib=f'{parent_dir}/libnn_new_zone.so', nn_nb=f'{parent_dir}/new_zone.nb', cls_num=2)
    rospy.logwarn("zone_info_node started")
    pub = rospy.Publisher("/zone_data", ZoneInfo, queue_size=1)
    state_sub = rospy.Subscriber('/zond_detect_state', UInt8, set_running_state)
    cap = cv2.VideoCapture(1)
    while not rospy.is_shutdown():
        if not running_state:
            continue
        read_state, cap_img = cap.read()
        if not side_color == 0:
            cap_img = cv2.bitwise_not(cap_img)
        if not read_state:
            rospy.logwarn("Cam read failed")
            continue
        
        # Inference
        boxes, scores, classes = inference(cap_img)

        zone_msg = ZoneInfo()
        zone_msg.stamp = rospy.Time.now()
        zone_msg.angle = 0
        zone_msg.distance = 0
        cast(np.ndarray, boxes)
        if boxes is None:
            boxes = np.zeros(shape=(0, 4))
            classes = np.zeros(shape=(0, 4))
        # Start fill locations data
        classes = cast(np.ndarray, classes)
        for box, color_num in zip(boxes, classes):
            if color_num != 1:
                continue
            x1, y1, x2, y2 = box
            x1 = limitValue(x1)
            x2 = limitValue(x2)
            y1 = limitValue(y1)
            y2 = limitValue(y2)
            ltx_location = IMAGE_WIDTH * x1
            lty_location = IMAGE_HEIGHT * y1
            rbx_location = IMAGE_WIDTH * x2
            rby_location = IMAGE_HEIGHT * y2
            center_x, center_y = (ltx_location + rbx_location) / 2, (lty_location + rby_location) / 2
            angle = (IMAGE_WIDTH / 2 - center_x) * ANGLE_SCALE
            distance = center_y * DISTANCE_SCALE
            zone_msg.angle = angle
            zone_msg.distance = IMAGE_HEIGHT - distance
            break
        pub.publish(zone_msg)
        ips += 1
        if time.time() - last_log_time >= 1:
            rospy.loginfo(f'ips is {ips}')
            last_log_time = time.time()
            ips = 0
