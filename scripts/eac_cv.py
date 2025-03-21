import cv2 as cv
import numpy as np

lower_red1 = np.array([0, 80, 70])
upper_red1 = np.array([10, 255, 255])

lower_red2 = np.array([170, 80, 70])
upper_red2 = np.array([180, 255, 255])

lower_blue = np.array([100, 50, 50])
upper_blue = np.array([140, 230, 230])


def extract_pixels(image, color_mask: int):
    if color_mask == 0:
        mask1 = cv.inRange(image, lower_red2, upper_red2)
        mask2 = cv.inRange(image, lower_red1, upper_red1)
        mask = cv.bitwise_or(mask1, mask2)
    elif color_mask == 1:
        mask = cv.inRange(image, lower_blue, upper_blue)
    else:
        mask1 = cv.inRange(image, lower_red2, upper_red2)
        mask2 = cv.inRange(image, lower_red1, upper_red1)
        mask_red = cv.bitwise_or(mask1, mask2)
        mask_blue = cv.inRange(image, lower_blue, upper_blue)
        mask = cv.bitwise_or(mask_red, mask_blue)
    return mask


def rect_normalize(shape, box):
    x1, y1, x2, y2 = box
    # print('class: {}, score: {}'.format(CLASSES[cl], score))
    # print('box coordinate left,top,right,down: [{}, {}, {}, {}]'.format(x1, y1, x2, y2))
    x1 *= shape[1]
    y1 *= shape[0]
    x2 *= shape[1]
    y2 *= shape[0]
    left = max(0, np.floor(x1 + 0.5).astype(int))
    top = max(0, np.floor(y1 + 0.5).astype(int))
    right = min(shape[1], np.floor(x2 + 0.5).astype(int))
    bottom = min(shape[0], np.floor(y2 + 0.5).astype(int))
    return left, top, right, bottom


def create_mask(image, rect):
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    left, top, right, bottom = rect_normalize(image.shape, rect)
    cx, cy = int(right + left) // 2, int(bottom + top) // 2
    radius = int(np.sqrt((right - left) ** 2 + (bottom - top) ** 2) / 2)
    cv.circle(mask, (cx, cy), radius, [255], thickness=-1)
    cv.rectangle(mask, (left, top), (right, bottom), [0], thickness=-1)
    return mask


def calculate_ratio(image, mask, is_red: bool):
    image_after_mask = cv.bitwise_or(image, image, mask=mask)
    total_pixels = cv.countNonZero(mask)
    hsv_image = cv.cvtColor(image_after_mask, cv.COLOR_BGR2HSV)
    if is_red:
        mask_pixels1 = cv.countNonZero(cv.inRange(hsv_image, lower_red1, upper_red1))
        mask_pixels2 = cv.countNonZero(cv.inRange(hsv_image, lower_red2, upper_red2))
        mask_pixels = mask_pixels1 + mask_pixels2
    else:
        mask_pixels = cv.countNonZero(cv.inRange(hsv_image, lower_blue, upper_blue))
    return mask_pixels / total_pixels


def calc_for_rects(image, rects):
    ratios = []
    if rects is None:
        return None
    for rect in rects:
        singel_mask = create_mask(image, rect)
        ratios.append(calculate_ratio(image, singel_mask, False))
    return ratios
