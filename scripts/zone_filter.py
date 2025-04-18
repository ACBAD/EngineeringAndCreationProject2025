#!/usr/bin/env python3
#coding=utf-8

import enum
import os
import cv2
import numpy as np


# 定义紫色的HSV范围（H值需要根据实际情况调整）
lower_purple = np.array([115, 50, 35])  # H最小值，S最小值，V最小值
upper_purple = np.array([150, 230, 230])  # H最大值，S最大值，V最大值

lower_red1 = np.array([0, 80, 70])
upper_red1 = np.array([10, 230, 230])
lower_red2 = np.array([170, 80, 70])
upper_red2 = np.array([180, 230, 230])

lower_blue = np.array([85, 100, 40])
upper_blue = np.array([120, 255, 230])


class Color(enum.Enum):
    PURPLE = 2
    RED = 0
    BLUE = 1


def createColorMask(img, color: Color, is_hsv=False):
    # 转换为HSV色彩空间
    if not is_hsv:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    else:
        hsv = img
    # 创建颜色遮罩
    if color is Color.PURPLE:
        mask = cv2.inRange(hsv, lower_purple, upper_purple)
    elif color is Color.RED:
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
    elif color is Color.BLUE:
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
    else:
        raise ValueError('No such color')
    return mask


def morphologyProcess(mask, kernel_shape=5):
    # kernel = np.ones((kernel_shape, kernel_shape), np.uint8)  # 5x5矩形结构
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_shape, kernel_shape))  # 椭圆结构
    closed_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
    return closed_mask


def calculateRatio(img, mask, color: Color):
    image_after_mask = cv2.bitwise_or(img, img, mask=mask)
    total_pixels = cv2.countNonZero(mask)
    hsv_image = cv2.cvtColor(image_after_mask, cv2.COLOR_BGR2HSV)
    if color == Color.RED:
        mask_pixels1 = cv2.countNonZero(cv2.inRange(hsv_image, lower_red1, upper_red1))
        mask_pixels2 = cv2.countNonZero(cv2.inRange(hsv_image, lower_red2, upper_red2))
        mask_pixels = mask_pixels1 + mask_pixels2
    elif color == Color.BLUE:
        mask_pixels = cv2.countNonZero(cv2.inRange(hsv_image, lower_blue, upper_blue))
    elif color == Color.PURPLE:
        mask_pixels = cv2.countNonZero(cv2.inRange(hsv_image, lower_purple, upper_purple))
    else:
        raise ValueError('No such color')
    if total_pixels == 0:
        return 0
    return mask_pixels / total_pixels


def createContourMask(mask, contours):
    contour_mask = np.zeros_like(mask)
    cv2.drawContours(contour_mask, [contours] if not isinstance(contours, list) else contours, -1, [255], -1)  # 填充白色凸包区域
    return contour_mask


def createConvexHullContour(mask, connect_radius=50, min_area=80):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= min_area]
    if not contours:
        return None
    # 找出最大轮廓
    max_contour = max(contours, key=cv2.contourArea)
    max_hull = cv2.convexHull(max_contour)
    # 用于合并的所有点集合
    all_points = list(max_hull.reshape(-1, 2))  # 初始是最大凸包的点
    for cnt in contours:
        if np.array_equal(cnt, max_contour):
            continue
        # 找出该轮廓上离最大凸包最近的点
        min_dist = float('inf')
        for point in cnt:
            pt = tuple(float(v) for v in point[0])  # 转为浮点数 (x, y)
            dist = cv2.pointPolygonTest(max_hull, pt, True)
            abs_dist = abs(dist)
            if abs_dist < min_dist:
                min_dist = abs_dist
        if min_dist <= connect_radius:
            all_points.extend(cnt.reshape(-1, 2))  # 合并所有点
    # 重新计算合并后的总凸包
    all_points_np = np.array(all_points).reshape(-1, 1, 2).astype(np.int32)
    final_hull = cv2.convexHull(all_points_np)
    return final_hull


def ifPointInZone(img, point):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    purple_mask = morphologyProcess(createColorMask(hsv_img, Color.PURPLE, is_hsv=True), kernel_shape=10)
    convex_hull_contour = createConvexHullContour(purple_mask, connect_radius=150)
    if convex_hull_contour is None:
        return False
    if cv2.pointPolygonTest(convex_hull_contour, point, True) >= 0:
        return True
    return False


def detectZone(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    purple_mask = morphologyProcess(createColorMask(hsv_img, Color.PURPLE, is_hsv=True), kernel_shape=10)
    convex_hull_contour = createConvexHullContour(purple_mask, connect_radius=150)
    if convex_hull_contour is None:
        return None
    convex_hull_mask = createContourMask(purple_mask, convex_hull_contour)
    zone_pixels_count = cv2.countNonZero(convex_hull_mask)
    if zone_pixels_count < 300:
        return None
    # blue_mask = createColorMask(hsv_img, Color.BLUE, is_hsv=True)
    # red_mask = createColorMask(hsv_img, Color.RED, is_hsv=True)
    rect_x, rect_y, rect_w, rect_h = cv2.boundingRect(convex_hull_contour)
    blue_ratio = calculateRatio(img, convex_hull_mask, Color.BLUE)
    red_ratio = calculateRatio(img, convex_hull_mask, Color.RED)
    if blue_ratio > red_ratio:
        zone_color = Color.BLUE
    elif blue_ratio < red_ratio:
        zone_color = Color.RED
    else:
        zone_color = Color.PURPLE
    zone_center = (rect_x + rect_w // 2, rect_y + rect_h // 2)
    # zone_size = np.sqrt(rect_w ** 2 + rect_h ** 2)
    zone_size = (rect_w, rect_h)
    return zone_color, zone_center, zone_size, (rect_x, rect_y, rect_w, rect_h)


def drawZone(img, rect_color, rect_info):
    text_x = rect_info[0] + rect_info[2] - 40
    text_y = rect_info[1] + 20  # 20 是一个经验值，可微调字体高度
    if rect_color is Color.BLUE:
        text_color = 'blue'
    else:
        text_color = 'red'
    cv2.putText(img, text_color, (text_x, text_y),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=(0, 0, 255), thickness=2)
    cv2.rectangle(img, (rect_info[0], rect_info[1]),
                  (rect_info[0] + rect_info[2], rect_info[1] + rect_info[3]), (0, 255, 0), 2)


def rosRun():
    ANGLE_SCALE = 1
    DISTANCE_SCALE = 1
    import rospy
    from eac_pkg.msg import ZoneInfo
    rospy.init_node('zone_info_node')
    try:
        side_color = rospy.get_param('side_color')
    except KeyError:
        rospy.logfatal('Please set side_color!!!')
        exit(1)
    rospy.logwarn("zone_info_node started")
    pub = rospy.Publisher("/zone_data", ZoneInfo, queue_size=1)
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    while not rospy.is_shutdown():
        read_state, cap_img = cap.read()
        if not read_state:
            rospy.logwarn("Cam read failed")
            continue
        crop_height = 270
        IMAGE_HEIGHT, IMAGE_WIDTH = cap_img.shape[:2]
        if IMAGE_HEIGHT < crop_height:
            raise ValueError(f'Image height is {IMAGE_HEIGHT}, can not crop')
        cap_img = cap_img[IMAGE_HEIGHT - crop_height: IMAGE_HEIGHT, :]
        IMAGE_HEIGHT = crop_height
        zone_msg = ZoneInfo()
        zone_msg.stamp = rospy.Time.now()
        zone_msg.angle = 0
        zone_msg.distance = 0
        detect_result = detectZone(cap_img)
        if detect_result is None:
            pub.publish(zone_msg)
            continue
        color, center, size, rect_info = detect_result
        if color.value != side_color:
            pub.publish(zone_msg)
            continue
        angle = (IMAGE_WIDTH / 2 - center[0]) * ANGLE_SCALE
        zone_msg.angle = angle
        distance = center[1] * DISTANCE_SCALE
        zone_msg.distance = IMAGE_HEIGHT - distance
        pub.publish(zone_msg)


def testMain():
    def show_if_in(gimage, x, y):
        print(f'Point: {x}, {y}')
        print(ifPointInZone(gimage, (x, y)))
    from hsv_window import ClickHSVWindow
    for file in os.listdir('data'):
        image = cv2.imread(os.path.join('data', file))
        image = cv2.resize(image, (960, 540))
        purple_mask = morphologyProcess(createColorMask(image, Color.PURPLE), kernel_shape=10)
        convex_hull_contour = createConvexHullContour(purple_mask, connect_radius=150)
        convex_hull_mask = createContourMask(purple_mask, convex_hull_contour)
        ClickHSVWindow(f'{file}: image', image, custom_callback=show_if_in)
        cv2.imshow(f'{file}: convex_hull_mask', convex_hull_mask)
        try:
            cv2.waitKey(0)
        except KeyboardInterrupt:
            break
        cv2.destroyAllWindows()
        ClickHSVWindow.close_all()

if __name__ == '__main__':
    rosRun()
