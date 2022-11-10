#! /usr/bin/env python3
"""trt_yolo.py

This script demonstrates how to do real-time object detection with
TensorRT optimized YOLO engine.
"""


import os
import time
import argparse
import numpy as np
import cv2
# issue solved 
import pycuda.driver as cuda    
import pycuda.autoinit  # This is needed for initializing CUDA driver
import sys
import message_filters

from cv_bridge import CvBridge, CvBridgeError
from utils.yolo_classes_red_hand import get_cls_dict #red_hand light has 8 classes
from utils.camera import add_camera_args, Camera
from utils.display import open_window, set_display, show_fps
from utils.visualization import BBoxVisualization
from utils.yolo_with_plugins import TrtYOLO
import rospy
from std_msgs.msg import Float32MultiArray # for sending msg class, confidence inform 
from vision_msgs.msg import Detection2D # for sending bouding box
from vision_msgs.msg import Detection2DArray # for sending bounding box
from vision_msgs.msg import BoundingBox2D # for sending bounding box
from vision_msgs.msg import ObjectHypothesisWithPose # for sending bounding box
from sensor_msgs.msg import Image


WINDOW_NAME = 'Esens Detection'
# loop and detection params
CV_BRIDGE = CvBridge()

def parse_args():
    """Parse input arguments."""
    desc = ('Capture and display live camera video, while doing '
            'real-time object detection with TensorRT optimized '
            'YOLO model on Jetson')
    parser = argparse.ArgumentParser(description=desc)
    parser = add_camera_args(parser)
    parser.add_argument(
        '-c', '--category_num', type=int, default=1, 
        help='number of object categories [1]')
    parser.add_argument(
        '-m', '--model', type=str, required=True,
        help=('[yolov3-tiny|yolov3|yolov3-spp|yolov4-tiny|yolov4|'
              'yolov4-csp|yolov4x-mish]-[{dimension}], where '
              '{dimension} could be either a single number (e.g. '
              '288, 416, 608) or 2 numbers, WxH (e.g. 416x256)'))
    parser.add_argument(
        '-l', '--letter_box', action='store_true',
        help='inference with letterboxed image [False]')
    args = parser.parse_args()
    return args

def loop_and_detect(image, bbox_pub=None, img_pub=None):
    """Continuously capture images from camera and do object detection.

    # Arguments
      image: bbox_pub ,the camera instance (video source).
      trt_yolo: the TRT YOLO object detector instance.
      conf_th: confidence/score threshold for object detection.
      vis: for visualization.
    """    
    r = rospy.Rate(50) #to downgrade FPS
    full_scrn = False

    # image callback function
    try:
        img = CV_BRIDGE.imgmsg_to_cv2(image, 'bgr8')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {}".format(e))

    # assert cv2.getWindowProperty(WINDOW_NAME, 0) < 0
    # assert img is None
    boxes, confs, clss = trt_yolo.detect(img, conf_th)
    rst = vis.draw_bboxes(img, boxes, confs, clss)
    
    # publish msg        
    detection2d = Detection2DArray() # create Detection2DArray ros msg (vision_msg)
    detection2d.header.stamp = rospy.Time.now()
    detection2d.header.frame_id = "camera"
    idx = int()
    box_size = float()
    box_arr = []
    
    for i in range(len(boxes)):
        detection = Detection2D()
        obj_info = ObjectHypothesisWithPose()
        detection.header.stamp = rospy.Time.now()
        detection.header.frame_id="camera"
            
        detection.bbox.center.x = boxes[i][0] + (boxes[i][2] - boxes[i][0])/2 # get bounding center x
        detection.bbox.center.y = boxes[i][1] + (boxes[i][3] - boxes[i][1])/2 # get bounding cetner y
        detection.bbox.center.theta = 0.0
        print(detection.bbox.center.x, detection.bbox.center.y, img.shape)
        detection.bbox.size_x = abs(boxes[i][0] - boxes[i][2]) # get bounding x size
        detection.bbox.size_y = abs(boxes[i][1] - boxes[i][3]) # get bounding y size
            
        #box size
        box_size = detection.bbox.size_x * detection.bbox.size_y
            
        #cls conf
        obj_info.id = int(clss[i])
        obj_info.score = confs[i]

        if (i==0):
            box_arr = np.append(box_arr, box_size)
        else:
            box_arr = np.append(box_arr, box_size)
            box_arr = np.sort(box_arr)[::-1]
            idxar = np.where(box_arr == box_size)
            idx = idxar[0][0]
        
        detection.results.append(obj_info)
        detection2d.detections.insert(idx, detection)
        
    img_pub.publish(CV_BRIDGE.cv2_to_imgmsg(rst, "bgr8"))# 
    bbox_pub.publish(detection2d) # send msg bounding box
    r.sleep() # to downgrade FPS


def main(realsense_img):
    rospy.init_node('red_hand', anonymous=False) # create node

    if not os.path.isfile('yolo/%s.trt' % args.model):
        raise SystemExit('ERROR: file (yolo/%s.trt) not found!' % args.model)

    open_window(
        WINDOW_NAME, 'Red Hand YOLOv4',
        640, 480)

    # Subscribe to topics
    image_sub = message_filters.Subscriber(realsense_img, Image)
    
    # Publish topics
    bbox_pub = rospy.Publisher('/yolo_bbox', Detection2DArray , queue_size=1) # create bbox msg 
    img_pub = rospy.Publisher('/yolo_img', Image , queue_size=1) # create bbox msg 
    
    # Synchronize the topic by time: image
    ats = message_filters.ApproximateTimeSynchronizer(
        [image_sub], queue_size=1, slop=0.1)
    ats.registerCallback(loop_and_detect, bbox_pub, img_pub)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down')


if __name__ == '__main__':
    args = parse_args()
    trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box, cuda_ctx=pycuda.autoinit.context)
    conf_th = 0.95
    cls_dict = get_cls_dict(args.category_num)
    vis = BBoxVisualization(cls_dict)
    realsense_img = '/camera/color/image_raw'
    main(realsense_img)