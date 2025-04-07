import numpy as np
import os
import urllib.request
from matplotlib import gridspec
from matplotlib import pyplot as plt
from PIL import Image
import argparse
import sys
import math
from ksnn.api import KSNN
from ksnn.types import *
import cv2 as cv
import time
from eac_cv import create_mask, calculate_ratio

GRID0 = 20
GRID1 = 40
GRID2 = 80
SPAN = 1
NUM_CLS = 4
LISTSIZE = NUM_CLS + 64
MAX_BOXES = 500
OBJ_THRESH = 0.4
NMS_THRESH = 0.5
mean = [0, 0, 0]
var = [255]

constant_martix = np.array([[0, 1, 2, 3,
                             4, 5, 6, 7,
                             8, 9, 10, 11,
                             12, 13, 14, 15]]).T

CLASSES = ('red', 'yellow', 'blue', 'black')


def sigmoid(x):
    return 1 / (1 + np.exp(-x))


def softmax(x, axis=0):
    x = np.exp(x)
    return x / x.sum(axis=axis, keepdims=True)


def process(input):
    grid_h, grid_w = map(int, input.shape[0:2])

    box_class_probs = sigmoid(input[..., :NUM_CLS])

    box_0 = softmax(input[..., NUM_CLS: NUM_CLS + 16], -1)
    box_1 = softmax(input[..., NUM_CLS + 16:NUM_CLS + 32], -1)
    box_2 = softmax(input[..., NUM_CLS + 32:NUM_CLS + 48], -1)
    box_3 = softmax(input[..., NUM_CLS + 48:NUM_CLS + 64], -1)

    result = np.zeros((grid_h, grid_w, 1, 4))
    result[..., 0] = np.dot(box_0, constant_martix)[..., 0]
    result[..., 1] = np.dot(box_1, constant_martix)[..., 0]
    result[..., 2] = np.dot(box_2, constant_martix)[..., 0]
    result[..., 3] = np.dot(box_3, constant_martix)[..., 0]
    col = np.tile(np.arange(0, grid_w), grid_w).reshape(-1, grid_w)
    row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_h)

    col = col.reshape(grid_h, grid_w, 1, 1)
    row = row.reshape(grid_h, grid_w, 1, 1)
    grid = np.concatenate((col, row), axis=-1)

    result[..., 0:2] = (0.5 - result[..., 0:2] + grid) / (grid_w, grid_h)
    result[..., 2:4] = (0.5 + result[..., 2:4] + grid) / (grid_w, grid_h)

    return result, box_class_probs


def filter_boxes(boxes, box_class_probs):
    box_classes = np.argmax(box_class_probs, axis=-1)
    box_class_scores = np.max(box_class_probs, axis=-1)
    pos = np.where(box_class_scores >= OBJ_THRESH)

    boxes = boxes[pos]
    classes = box_classes[pos]
    scores = box_class_scores[pos]

    return boxes, classes, scores


def nms_boxes(boxes, scores):
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    keep = np.array(keep)
    return keep


def yolov3_post_process(input_data):
    boxes, classes, scores = [], [], []
    for i in range(3):
        result, confidence = process(input_data[i])
        b, c, s = filter_boxes(result, confidence)
        boxes.append(b)
        classes.append(c)
        scores.append(s)

    boxes = np.concatenate(boxes)
    classes = np.concatenate(classes)
    scores = np.concatenate(scores)

    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]

        keep = nms_boxes(b, s)

        nboxes.append(b[keep])
        nclasses.append(c[keep])
        nscores.append(s[keep])

    if not nclasses and not nscores:
        return None, None, None

    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)

    return boxes, scores, classes


def draw(image, boxes, scores, classes):
    for box, score, cl in zip(boxes, scores, classes):
        x1, y1, x2, y2 = box
        # print('class: {}, score: {}'.format(CLASSES[cl], score))
        # print('box coordinate left,top,right,down: [{}, {}, {}, {}]'.format(x1, y1, x2, y2))
        x1 *= image.shape[1]
        y1 *= image.shape[0]
        x2 *= image.shape[1]
        y2 *= image.shape[0]
        left = max(0, np.floor(x1 + 0.5).astype(int))
        top = max(0, np.floor(y1 + 0.5).astype(int))
        right = min(image.shape[1], np.floor(x2 + 0.5).astype(int))
        bottom = min(image.shape[0], np.floor(y2 + 0.5).astype(int))

        cv.rectangle(image, (left, top), (right, bottom), (255, 0, 0), 2)
        cv.putText(image, '{0} {1:.2f}'.format(CLASSES[cl], score),
                   (left, top - 6),
                   cv.FONT_HERSHEY_SIMPLEX,
                   0.6, (0, 0, 255), 2)

init_state = False

def inference(input_image, do_filter=False):
    if not init_state:
        print('Warning! KSNN not init, using default configuration')
        init_nn()

    orig_img = input_image
    img = cv.resize(orig_img, (640, 640)).astype(np.float32)
    img[:, :, 0] = img[:, :, 0] - mean[0]
    img[:, :, 1] = img[:, :, 1] - mean[1]
    img[:, :, 2] = img[:, :, 2] - mean[2]
    img = img / var[0]

    img = img.transpose(2, 0, 1)
    # img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    cv_img = []
    cv_img.append(img)

    '''
        default input_tensor is 1
    '''
    data = yolov3.nn_inference(cv_img, platform='ONNX', reorder='2 1 0', output_tensor=3,
                               output_format=output_format.OUT_FORMAT_FLOAT32)

    input0_data = data[2]
    input1_data = data[1]
    input2_data = data[0]

    input0_data = input0_data.reshape(SPAN, LISTSIZE, GRID0, GRID0)
    input1_data = input1_data.reshape(SPAN, LISTSIZE, GRID1, GRID1)
    input2_data = input2_data.reshape(SPAN, LISTSIZE, GRID2, GRID2)

    input_data = list()
    input_data.append(np.transpose(input0_data, (2, 3, 0, 1)))
    input_data.append(np.transpose(input1_data, (2, 3, 0, 1)))
    input_data.append(np.transpose(input2_data, (2, 3, 0, 1)))

    boxes, scores, classes = yolov3_post_process(input_data)
    if not do_filter:
        return boxes, scores, classes
    if boxes is None:
        return None, None, None
    filtered_boxes = []
    filtered_scores = []
    filtered_classes = []
    for box, score, class_id in zip(boxes, scores, classes):
        if class_id not in (0, 2):
            filtered_boxes.append(box[None])
            filtered_scores.append(score[None])
            filtered_classes.append(class_id[None])
            continue
        mask = create_mask(orig_img, box)
        color_ratio = calculate_ratio(orig_img, mask, True if class_id == 0 else False)
        if color_ratio < 0.5:
            filtered_boxes.append(box[None])
            filtered_scores.append(score[None])
            filtered_classes.append(class_id[None])
        else:
            print(f"Detect dang zone at {box}")
    if not filtered_boxes:
        return None, None, None
    # print(f"Origin data: {(boxes, scores, classes)}")
    # print(f"My data: {(filtered_boxes, filtered_scores, filtered_classes)}")
    return np.concatenate(filtered_boxes), np.concatenate(filtered_scores), np.concatenate(filtered_classes)

def init_nn(nn_lib='/home/khadas/catkin/src/EngineeringAndCreationProject2025/scripts/libnn_new_eac.so', nn_nb='/home/khadas/catkin/src/EngineeringAndCreationProject2025/scripts/new_eac.nb', cls_num=4):
    global init_state, NUM_CLS, LISTSIZE
    NUM_CLS = cls_num
    LISTSIZE = NUM_CLS + 64
    global yolov3
    yolov3 = KSNN('VIM3')
    print(' |---+ KSNN Version: {} +---| '.format(yolov3.get_nn_version()))

    print('Start init neural network ...')
    yolov3.nn_init(library=nn_lib,
                   model=nn_nb, level=0)
    print('Done.')
    init_state = True

if __name__ == "__main__":
    print('Get input data ...')
    cap = cv.VideoCapture(0)
    while 1:
        # orig_img = cv.imread('./test.jpg', cv.IMREAD_COLOR)
        _, output_img = cap.read()
        boxes, scores, classes = inference(output_img, True)
        if boxes is not None:
            draw(output_img, boxes, scores, classes)
        cv.imshow("results", output_img)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv.destroyAllWindows()

