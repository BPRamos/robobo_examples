#!/usr/bin/env python
# -*- coding: future_fstrings -*-

import datetime
from matplotlib import cm
from matplotlib.colors import Normalize
import numpy as np

import pathlib2
import rospy
import cv2
from cv_bridge import CvBridge

from std_msgs.msg import Int16, Int8, String, Bool
from geometry_msgs.msg import Pose, Point, Quaternion
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image, CompressedImage
from robobo_msgs.srv import SetModule, SetModuleRequest



def get_service_client(service_name, service_type):
    rospy.wait_for_service(service_name)
    try:
        service = rospy.ServiceProxy(service_name, service_type)
        return service
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

class snap_node(object):
    def __init__(self):
        self.cv_bridge = CvBridge()

        self.detection2DArray = None

        self.img = None

        with open('labelmap.txt') as f:
            self.labels = f.read().splitlines()

        rospy.init_node('snap_node', anonymous=True)

        self.set_module_service = ("/robot/robobo2/setModule", SetModule)
        
        rospy.Subscriber("/robot/robobo2/detected_object", Detection2DArray, self.detection_callback)
        rospy.Subscriber("/robot/robobo2/camera/image/compressed", CompressedImage, self.camera_callback)

        #initialize modules
        self.set_module(module_name="object-recognition", module_state=True)

    def snap(self):
        rospy.sleep(1.0)
        now = datetime.datetime.now()
        snap_path = pathlib2.Path("./Snap/{}".format(now.strftime("%d-%m-%Y_%H%M%S")))
        snap_path.mkdir(parents=True, exist_ok=True)

        if(self.detection2DArray):
            detections = self.detection2DArray.detections
        else:
            detections = []

        i = 0
        for detection in detections:
            file = snap_path / "{}.detection".format(i)
            with file.open(mode="w+") as f:
                self.write_detection(f, detection)
            i+=1
        self.write_image(snap_path)

        rospy.signal_shutdown("Finished Snap")


    def write_detection(self, file, detection):
        if(detection):
            file.write(u"Bbox:\n")
            file.write(u"\tsize_x:{}\n".format(detection.bbox.size_x))
            file.write(u"\tsize_y:{}\n".format(detection.bbox.size_y))
            file.write(u"\tcenter:\n")
            file.write(u"\t\ttheta:{}\n".format(detection.bbox.center.theta))
            file.write(u"\t\tx:{}\n".format(detection.bbox.center.x))
            file.write(u"\t\ty:{}\n".format(detection.bbox.center.y))

            file.write(u"Results:\n")
            j=0
            for result in detection.results:
                file.write(u"\t{}:\n".format(j))
                file.write(u"\t\tid:{}\n".format(result.id))
                file.write(u"\t\tscore:{}\n".format(result.score))
                j+=1


    def camera_callback(self, data):
        self.img = data

    def detection_callback(self, data):
        self.detection2DArray = data

    def set_module(self, module_name="", module_state=True):
        module_request = SetModuleRequest()

        module_request.moduleName = String(module_name)
        module_request.moduleState = Bool(module_state)

        service_client = get_service_client(self.set_module_service[0],self.set_module_service[1])
        response = service_client.call(module_request)

    def write_image(self, path, threshold=0.35):
        if(not self.img):
            return
        np_arr = np.fromstring(self.img.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imwrite(str(path / "original.png"), cv_image)

        if(self.detection2DArray):
            detections = self.detection2DArray.detections
        else:
            detections = []
        viridis = cm.get_cmap('viridis', 8)
        norm = Normalize(vmin=0, vmax=len(detections))
        
        i=0
        for detection in detections:
            if(detection.results[0].score > threshold):
                bb = [0.0, 0.0, 0.0, 0.0]
                bb[0] = int(detection.bbox.center.y - (detection.bbox.size_y / 2.0))
                bb[2] = int(detection.bbox.center.y + (detection.bbox.size_y / 2.0))
                bb[1] = int(detection.bbox.center.x - (detection.bbox.size_x / 2.0))
                bb[3] = int(detection.bbox.center.x + (detection.bbox.size_x / 2.0))

                class_id = detection.results[0].id
                cv2.rectangle(cv_image,(bb[1],bb[0]),(bb[3],bb[2]),viridis(norm(i)),3)
                cv2.putText(cv_image, "{}:{}".format(class_id,detection.results[0].score), (bb[1],bb[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, viridis(norm(i)), 2)
            i+=1
        cv2.imwrite(str(path / "bboxes.png"), cv_image)

def main():
    instancia = snap_node()
    try:
        instancia.snap()
    except rospy.ROSInterruptException:
        print("Shutting down TF IRL test module")

if __name__ == '__main__':
    main()
