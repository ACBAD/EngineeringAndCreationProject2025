#!/usr/bin/env python3
#coding=utf-8

import time
from typing import cast
import cv2
import rospy
from std_msgs.msg import MultiArrayDimension
from eac_pkg.msg import VrResult
from yolov8n import inference
import numpy

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 640

last_log_time = time.time()
ips = 0

if __name__ == "__main__":
    rospy.init_node("yolo_node")
    rospy.logwarn("yolo_node started")
    pub = rospy.Publisher("/visual_data", VrResult, queue_size=2)
    cap = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
        read_state, cap_img = cap.read()
        if not read_state:
            rospy.logwarn("Cam read failed")
        
        # Inference
        boxes, scores, classes = inference(cap_img)

        vr_msg = VrResult()
        vr_msg.header.frame_id = "map"
        vr_msg.header.stamp = rospy.Time.now()
        boxes = cast(numpy.ndarray, boxes)
        if boxes is None:
            continue
        vr_msg.count.data = len(boxes)
        # Start fill locations data
        vr_msg.locations.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        vr_msg.locations.layout.dim[0].size = len(boxes)
        vr_msg.locations.layout.dim[1].stride = 4
        location_data = []
        color_data = []
        shape_data = []
        classes = cast(numpy.ndarray, classes)
        for box, color_num in zip(boxes, classes):
            x1, y1, x2, y2 = box
            location_data.append(x1 * IMAGE_WIDTH)
            location_data.append(x2 * IMAGE_WIDTH)
            location_data.append(y1 * IMAGE_HEIGHT)
            location_data.append(y2 * IMAGE_HEIGHT)
            color_data.append(color_num)
            shape_data.append(0)
        vr_msg.locations.data = tuple(location_data)
        vr_msg.colors.data = tuple(color_data)
        vr_msg.shapes.data = tuple(shape_data)
        pub.publish(vr_msg)
        ips += 1
        if time.time() - last_log_time < 1:
            rospy.loginfo(f'ips is {ips}')
            last_log_time = time.time()
            ips = 0
