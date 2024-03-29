#!/usr/bin/env python
# -*- coding: future_fstrings -*-
import numpy as np
import pathlib2

import rospy
#import tf2_ros
import cv2
from cv_bridge import CvBridge
import pandas as pd

import datetime
import subprocess
import json
from rospy import service

from std_msgs.msg import Int16, Int8, String, Bool
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModelRequest, SpawnModel, DeleteModel, DeleteModelRequest
from robobo_msgs.srv import MovePanTilt, MovePanTiltRequest, SetModule, SetModuleRequest
from robobo_msgs.msg import IRs
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image, CompressedImage
from aruco_msgs.msg import Marker

import matplotlib.cm as cm
from matplotlib.colors import Normalize

def get_service_client(service_name, service_type):
    rospy.wait_for_service(service_name)
    try:
        service = rospy.ServiceProxy(service_name, service_type)
        return service
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def write_marker(file, marker):
    if(marker):
        file.write(u"Header:\n")
        file.write(u"\tSeq:{}\n".format(marker.header.seq))
        file.write(u"\tStamp:{}\n".format(marker.header.stamp))
        file.write(u"Id:{}\n".format(marker.id))
        file.write(u"Pose:\n")
        pose = marker.pose.pose
        file.write(u"\tPosition: {} / {} / {}\n".format(pose.position.x,pose.position.y,pose.position.z))
        file.write(u"\tOrientation: [{} {} {} {}]\n".format(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w))

def create_experiment(path, objects, distances, pans, tilts):
    json_serial = dict({'path': str(path), 'objects' : objects, 'distances' : distances, 'pans': pans, 'tilts': tilts})
    serial_path = path / "serial.json"
    with serial_path.open(mode="w+", encoding="utf-8") as f:
        dump = json.dumps(json_serial, ensure_ascii=False)
        f.write(unicode(dump))
    exit_code = subprocess.call(["python3", "aruco_full_test_gen_exp.py", "-path", str(path)])
    experiment = pd.read_csv(path / "experiment.csv")
    return experiment

def write_report(experiment_path, notes):
    with (experiment_path / "report.txt").open(mode="w+") as f:
        f.write(u"Experiment ({})\n".format(datetime.datetime.now()))

        f.write(unicode(notes))
        now = datetime.datetime.now()
        f.write(u"Experiment began at {}\n".format(now.strftime("%Y-%m-%d %H:%M:%S")))

def write_report_coda(experiment_path):
    with (experiment_path / "report.txt").open(mode="a") as f:
        now = datetime.datetime.now()
        f.write(u"Experiment ended at {}\n".format(now.strftime("%Y-%m-%d %H:%M:%S")))

class aruco_irl_experiment_node(object):
    def __init__(self):
        self.pan_unlock_id = 1
        self.tilt_unlock_id = 1
        self.cv_bridge = CvBridge()

        self.marker = None
        self.curr_pan = None
        self.curr_tilt = None
        self.curr_range = None

        self.img = None

        rospy.init_node('aruco_irl_experiment_node', anonymous=True)

        self.pan_tilt_service = ("/robot/robobo2/movePanTilt", MovePanTilt)
        self.set_module_service = ("/robot/robobo2/setModule", SetModule)
        
        rospy.Subscriber("/robot/robobo2/tag", Marker, self.marker_callback)
        rospy.Subscriber("/robot/robobo2/camera/image/compressed", CompressedImage, self.camera_callback)
        rospy.Subscriber("/robot/robobo2/pan", Int16, self.pan_callback)
        rospy.Subscriber("/robot/robobo2/tilt", Int16, self.tilt_callback)
        rospy.Subscriber("/robot/robobo2/irs", IRs, self.irs_callback)

        self.run_experiments()
        rospy.signal_shutdown("aruco_experiment finished recording. Run again for a new recording")

    def irs_callback(self, data):
        self.curr_range = data.FrontC.range

    def pan_callback(self, data):
        self.curr_pan = data
    def tilt_callback(self, data):
        self.curr_tilt = data

    def camera_callback(self, data):
        self.img = data

    def marker_callback(self, data):
        self.marker = data

    def set_module(self, module_name="", module_state=True):
        module_request = SetModuleRequest()

        module_request.moduleName = String(module_name)
        module_request.moduleState = Bool(module_state)

        service_client = get_service_client(self.set_module_service[0],self.set_module_service[1])
        response = service_client.call(module_request)

    def set_pan_tilt(self, pan=None, pan_speed=50, tilt=None, tilt_speed=50):
        move_request = MovePanTiltRequest()
        if(pan is not None):
            move_request.panPos = Int16(pan)
            move_request.panSpeed = Int8(pan_speed)
            move_request.panUnlockId = Int16(self.pan_unlock_id)
        if(tilt is not None):
            move_request.tiltPos = Int16(tilt)
            move_request.tiltSpeed = Int8(tilt_speed)
            move_request.tiltUnlockId = Int16(self.tilt_unlock_id)

        service_client = get_service_client(self.pan_tilt_service[0], self.pan_tilt_service[1])
        response = service_client.call(move_request)
        if(pan is not None):
            self.pan_unlock_id+=1
        if(tilt is not None):
            self.tilt_unlock_id+=1

    def write_image(self, path, threshold=0.35):
        if(not self.img):
            return
        np_arr = np.fromstring(self.img.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imwrite(str(path / "original.png"), cv_image)

        """
        viridis = cm.get_cmap('viridis', 8)
        norm = Normalize(vmin=0, vmax=len(self.detection2DArray.detections))
        
        i=0
        for detection in self.detection2DArray.detections:
            if(detection.results[0].score > threshold):
                bb = [0.0, 0.0, 0.0, 0.0]
                bb[0] = int(detection.bbox.center.y - (detection.bbox.size_y / 2.0))
                bb[2] = int(detection.bbox.center.y + (detection.bbox.size_y / 2.0))
                bb[1] = int(detection.bbox.center.x - (detection.bbox.size_x / 2.0))
                bb[3] = int(detection.bbox.center.x + (detection.bbox.size_x / 2.0))

                class_id = self.labels[int(detection.results[0].id)+1]
                cv2.rectangle(cv_image,(bb[1],bb[0]),(bb[3],bb[2]),viridis(norm(i)),3)
                cv2.putText(cv_image, "{}:{}".format(class_id,detection.results[0].score), (bb[1],bb[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, viridis(norm(i)), 2)
            i+=1
        cv2.imwrite(str(path / "bboxes.png"), cv_image)
        """

    def write_experiment(self,exp_path, obj_name, index):
        path = exp_path / "measurements" / obj_name / str(index)
        path.mkdir(parents=True, exist_ok=True)

        marker = None
        id = None
        pose = None
        pos = None
        orient = None

        if(self.marker):
            marker = self.marker
            id = marker.id
            pose = marker.pose.pose
            pos = "{} {} {}".format(pose.position.x,pose.position.y,pose.position.z)
            orient = "{} {} {} {}".format(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
        """
        {'ExpID': [], 'Object': [], 'Pan': [], 'Tilt': [], 'Real Distance': [], 'IR Range': [], 'Marker ID': [], 'Marker Position': [], 'Marker Orientation': []}
        """
        e = {'ExpID': index, 'Object': obj_name, 'Pan': self.curr_pan, 'Tilt': self.curr_tilt, 'Real Distance': self.curr_distance, 'IR Range': self.curr_range, 'Marker ID': id, 'Marker Position': pos, 'Marker Orientation': orient}
        self.dataFrame = self.dataFrame.append(e, ignore_index=True)

        file = path / "Marker.info"
        with file.open(mode="w+") as f:
            write_marker(f, marker)
        self.write_image(path)

    def test_object(self, index, exp_path):
        #spawn_object(self.spawn_service,self.curr_object, self.curr_object_path, x,y,z, orientation)
        rospy.sleep(5.0)
        self.write_experiment(exp_path, self.curr_object, index)
        #delete_object(self.delete_service,self.curr_object)
        rospy.sleep(5.0)

    def parse_inputs(self):
        object = raw_input("Object? (or 'same') :")
        if(object == "stop"):
            return False
        if(object != "same"):
            self.curr_object = object

        distance = raw_input("Distance? (or 'same') :")
        if(distance == "stop"):
            return False
        if(distance != "same"):
            self.curr_distance = float(distance)

        return True


    def run_experiments(self):
        self.curr_object = None
        self.curr_distance = 0

        #Initialize pan/tilt
        self.set_pan_tilt(pan=None,tilt=90)

        #initialize modules
        self.set_module(module_name="tag", module_state=True)

        now = datetime.datetime.now()
        experiment_path = pathlib2.Path("./aruco/irl/experiments/{}".format(now.strftime("%d-%m-%Y_%H%M%S")))
        experiment_path.mkdir(parents=True, exist_ok=True)

        #objects = ["velocidad10_aruco"]

        #distances = [0.2 , 0.25, 0.3 , 0.35, 0.4 , 0.45, 0.5 , 0.55, 0.6 ,0.65, 0.7 , 0.75, 0.8 , 0.85, 0.9 , 0.95]

        #tilts = [None]
        #pans = [None]

        #has_pan = not (pans[0] == None)
        #has_tilt = not (tilts[0] == None)
        
        notes = "Nothing more to report"
        #experiments = create_experiment(experiment_path, objects, distances, pans, tilts)
        
        write_report(experiment_path, notes)

        columns  = {'ExpID': [], 'Object': [], 'Pan': [], 'Tilt': [], 'Real Distance': [], 'IR Range': [], 'Marker ID': [], 'Marker Position': [], 'Marker Orientation': []}
    
        self.dataFrame = pd.DataFrame(columns)
        exp_id = 0

        keep_working = self.parse_inputs()
        while(keep_working):
            self.test_object(exp_id, experiment_path)
            exp_id+=1
            keep_working = self.parse_inputs()

            
        write_report_coda(experiment_path)
        
        print(self.dataFrame)
        self.dataFrame.to_csv(experiment_path / "df.csv")


def main():
    instancia = aruco_irl_experiment_node()
    try:
        rospy.spin() # spin() simply keeps python from exiting until this node is stopped
    except rospy.ROSInterruptException:
        print("Shutting down Aruco test module")

if __name__ == '__main__':
    main()