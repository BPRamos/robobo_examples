#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D

import cv2
from cv_bridge import CvBridge

import tensorflow as tf
import numpy as np

#import tflite_runtime.interpreter as tflite

def resize_and_crop(img, size):
    height, width = size
    in_height, in_width, _ = img.shape

    rospy.loginfo("Image Shape: %s", str((in_height, in_width)))
    rospy.loginfo("Self size: %s", str((height, width)))

    if ((in_width != width) or (in_height != height)):
        scale_X = width / float(in_width)
        scale_Y = height / float(in_height)


        if(scale_X > scale_Y):
            #width, scale_X
            new_height = int(in_height*scale_X)
            img_resized = cv2.resize(img, (width, new_height))
            center_y = new_height//2
            return img_resized[(center_y-(height//2)):(center_y+(height//2)), 0:width]   #rows(height), columns(width)
        else:
            #height
            new_width = int(in_width*scale_Y)
            img_resized = cv2.resize(img, (new_width, height))
            center_x = new_width//2
            return img_resized[0:height, (center_x-(width//2)):(center_x+(width//2))]   #rows(height), columns(width)


class ROBOBO_TENSORFLOW_INTERCEPT(object):
    def __init__(self, robobo_name='robobo', file_path="./detect.tflite"):

        self.robobo_name = robobo_name
        self.file_path = file_path
        rospy.init_node('RoboboObjectsIntercept', anonymous=True)

        self.bridge = CvBridge()

        self.size = (300, 300)
        """
        with(open("./labelmap.txt")) as f:
            self.labels = [line.rstrip() for line in f.readlines()]
        """

        #self.interpreter = tflite.Interpreter(model_path="./detect.tflite")
        self.interpreter = tf.lite.Interpreter(model_path=self.file_path)
        self.interpreter.allocate_tensors()

        self.inputDetails = self.interpreter.get_input_details()
        self.outputDetails = self.interpreter.get_output_details()

        # Topics
        rospy.Subscriber("/robot/robobo2/camera/image/compressed", Image, self.camera_callback)
        rospy.Subscriber("/robot/robobo2/detected_object", Image, self.detection_callback)

    def detection_callback(self, data):
        print("[Detection Received]")
        for detection in data.detections:
            print("Bbox:\n\tcenter_x={}\n\tcenter_y={}\n\tsize_x={}\n\tsize_y={}".format(detection.bbox.center.x, detection.bbox.center.y, detection.bbox.size_x, detection.bbox.size_y))
            print("Score: {}\n".format(detection.results[0].score))
            print("class_id ={}".format(int(detection.results[0].id)))



    def camera_callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)


        #inp = cv2.resize(cv_image, self.size)
        inp = resize_and_crop(cv_image, self.size)

        norm_image = cv2.normalize(inp, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        norm_rgb = cv2.cvtColor(norm_image, cv2.COLOR_BGR2RGB)

        #rgb = cv2.cvtColor(norm_image, cv2.COLOR_BGR2RGB)
        rgb = cv2.cvtColor(inp, cv2.COLOR_BGR2RGB)
        
        rgb_tensor = tf.convert_to_tensor(norm_rgb, dtype=tf.float32)

        input_data = np.expand_dims(rgb_tensor,axis=0)

        self.interpreter.set_tensor(self.inputDetails[0]['index'], input_data)
        self.interpreter.invoke()

        detection_boxes = self.interpreter.get_tensor(self.outputDetails[0]['index'])
        detection_classes = self.interpreter.get_tensor(self.outputDetails[1]['index'])
        detection_scores = self.interpreter.get_tensor(self.outputDetails[2]['index'])

        num_boxes = self.interpreter.get_tensor(self.outputDetails[3]['index'])


        frameSize = np.array([data.height, data.width, data.height, data.width])

        for i in range(int(num_boxes[0])):
            print("class_id ={}".int(detection_classes[0, i]))

            # TFlite returns bounding boxes in (ymin, xmin, ymax, xmax) format, in the [0,1] range.
            # We use the size of the frame image as a range instead. (Check if that is consistent!)
            bbox = detection_boxes[0][i] * frameSize

            center_x = np.divide((bbox[1] + bbox[3]), 2.0)
            center_y = np.divide((bbox[0] + bbox[2]), 2.0)
            size_x = (bbox[3] - bbox[1])
            size_y = (bbox[2] - bbox[0])
            print("Bbox:\n\tcenter_x={}\n\tcenter_y={}\n\tsize_x={}\n\tsize_y={}".format(center_x, center_y, size_x, size_y))
            print("Score: {}\n".format(detection_scores[0, i]))

    def run(self):
        rospy.spin()


def main():
    instancia = ROBOBO_TENSORFLOW_INTERCEPT()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down Robobo Object Detection module")
        
if __name__ == '__main__':
    main()
