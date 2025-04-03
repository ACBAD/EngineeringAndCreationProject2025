#!/usr/bin/env python3
#coding=utf-8

import time
from typing import cast
import cv2
import rospy
from eac_pkg.msg import ObjectInfo, ObjectInfoArray
from yolov8n import inference
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

if __name__ == "__main__":
    rospy.init_node("yolo_node")
    try:    
        side_color = rospy.get_param('side_color')
    except KeyError:
        rospy.logfatal('Please set side_color!!!')
        exit(1)
    rospy.logwarn("yolo_node started")
    pub = rospy.Publisher("/objects_data", ObjectInfoArray, queue_size=2)
    cap = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
        read_state, cap_img = cap.read()
        if not read_state:
            rospy.logwarn("Cam read failed")
            continue
        
        # Inference
        boxes, scores, classes = inference(cap_img)

        objs_msg = ObjectInfoArray()
        objs_msg.data = cast(list, objs_msg.data)
        objs_msg.stamp = rospy.Time.now()
        cast(np.ndarray, boxes)
        if boxes is None:
            boxes = np.ndarray()
            classes = np.ndarray()
        # Start fill locations data
        classes = cast(np.ndarray, classes)
        for box, color_num in zip(boxes, classes):
            this_obj = ObjectInfo()
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
            this_obj.angle = angle
            this_obj.distance = distance
            this_obj.color = color_num
            this_obj.shape = 0
            objs_msg.data.append(this_obj)
        pub.publish(objs_msg)
        ips += 1
        if time.time() - last_log_time >= 1:
            rospy.loginfo(f'ips is {ips}')
            last_log_time = time.time()
            ips = 0
