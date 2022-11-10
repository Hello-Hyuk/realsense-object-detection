import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import pyrealsense2 as rs2

# yolo 
from vision_msgs.msg import Detection2D # for sending bouding box
from vision_msgs.msg import Detection2DArray # for sending bounding box
from vision_msgs.msg import BoundingBox2D # for sending bounding box
from vision_msgs.msg import ObjectHypothesisWithPose # for sending bounding box
from sensor_msgs.msg import Image

if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

class ImageListener:
    def __init__(self, depth_image_topic, yolo_depth_image_topic,depth_info_topic):
        self.bridge = CvBridge()
        #subscriber
        self.sub = rospy.Subscriber(depth_image_topic, Image, self.imageDepthCallback)
        self.sub_info = rospy.Subscriber(depth_info_topic, CameraInfo, self.imageDepthInfoCallback)
        confidence_topic = depth_image_topic.replace('depth', 'confidence')
        self.sub_conf = rospy.Subscriber(confidence_topic, msg_Image, self.confidenceCallback)
        #publisher
        self.detect_obj_info_pub = rospy.Publisher('/yolo_obj_info', Detection2D)
        self.rate = rospy.Rate(20)
        # depth image
        self.depth_img = None
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None
        #bbox
        self.sub_bbox = rospy.Subscriber(yolo_depth_image_topic, Detection2DArray, self.bboxCallback)
        self.left_x = 0
        self.left_y = 0
        self.right_x = 0
        self.right_y = 0
        self.center_x = 0
        self.center_y = 0

    
    def bboxCallback(self,data):
        for detection in enumerate(data.detections):
            if detection[0] == 0 :
                self.center_x = int(detection[1].bbox.center.x)
                self.center_y = int(detection[1].bbox.center.y)
                self.left_x = self.center_x - (detection[1].bbox.size_x / 2.0)
                self.left_y = self.center_y - (detection[1].bbox.size_y / 2.0)
                self.right_x = self.center_x + (detection[1].bbox.size_x / 2.0)
                self.right_y = self.center_y + (detection[1].bbox.size_y / 2.0)
                # pick depth from center pixel of bounding box:
        
        if not data.detections :
            print("nothing detected \n")
        else :
            self.depth = self.depth_img_np[self.center_y,self.center_x]
            if self.intrinsics:
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [self.center_x, self.center_y], self.depth)
                print(f"result : {result}")
    
            # publisher
            obj_info = Detection2D()
            obj_info.bbox.center.x = self.center_x
            obj_info.bbox.center.y = self.center_y
            obj_info.bbox.center.theta = self.depth
            self.detect_obj_info_pub.publish(obj_info)

    def imageDepthCallback(self, data):
        try:
            self.depth_img = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return
        self.depth_img_np = np.array(self.depth_img, dtype=np.float32)

    def confidenceCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            grades = np.bitwise_and(cv_image >> 4, 0x0f)
            if (self.pix):
                self.pix_grade = grades[self.pix[1], self.pix[0]]
        except CvBridgeError as e:
            print(e)
            return

    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

def main():
    depth_image_topic = '/camera/aligned_depth_to_color/image_raw'
    depth_info_topic = '/camera/aligned_depth_to_color/camera_info'
    yolo_depth_image_topic = '/yolo_bbox'
    
    listener = ImageListener(depth_image_topic, yolo_depth_image_topic, depth_info_topic)
    rospy.spin()

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()