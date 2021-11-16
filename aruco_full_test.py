#!/usr/bin/env python
# -*- coding: future_fstrings -*-
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

from std_msgs.msg import Int16, Int8
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModelRequest, SpawnModel, DeleteModel, DeleteModelRequest
from robobo_msgs.srv import MovePanTilt, MovePanTiltRequest
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from aruco_msgs.msg import Marker


import matplotlib.cm as cm
from matplotlib.colors import Normalize

#rosrun gazebo_ros spawn_model -file ./robobo-gazebo-models/cylinder_medium_blue/model.sdf -sdf -model cylinder_medium_blue -x 0
#/gazebo/spawn_sdf_model

#object_name aparently has to fit the name in the sdf

def spawn_object(spawn_service, object_name, object_path,x,y,z,orientation=(0,0,0,0)):
    with open(object_path, 'r') as f:
        object_xml = f.read()

    position = Point()
    position.x = x
    position.y = y
    position.z = z

    init_orientation = Quaternion()
    init_orientation.x = orientation[0]
    init_orientation.y = orientation[1]
    init_orientation.z = orientation[2]
    init_orientation.w = orientation[3]

    init_pose = Pose()
    init_pose.position = position
    init_pose.orientation = init_orientation

    spawn_request = SpawnModelRequest()
    spawn_request.model_name = object_name
    spawn_request.model_xml = object_xml
    spawn_request.initial_pose = init_pose
    
    service_client = get_service_client(spawn_service[0], spawn_service[1])
    service_client.call(spawn_request)

def delete_object(delete_service, model_name):
    delete_request = DeleteModelRequest()
    delete_request.model_name = model_name

    service_client = get_service_client(delete_service[0], delete_service[1])
    service_client.call(delete_request)

def get_service_client(service_name, service_type):
    rospy.wait_for_service(service_name)
    try:
        service = rospy.ServiceProxy(service_name, service_type)
        return service
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def write_marker(file, marker):
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

def write_report(experiment_path, notes, objects, distances, tilts, pans, has_pan, has_tilt, experiments):
    object_names = [object[0] for object in objects]
    with (experiment_path / "report.txt").open(mode="w+") as f:
        f.write(u"Experiment ({})\n".format(datetime.datetime.now()))
        f.write(u"Objects: {}\n".format(object_names))
        f.write(u"Distances: {}\n".format(distances))
        
        if(has_tilt):
            f.write(u"Tilts: {}\n".format(tilts))
        else:
            f.write(u"Tilt is STANDARD\n")
        
        if(has_pan):
            f.write(u"Pans: {}\n".format(pans))
        else:
            f.write(u"Pan is STANDARD\n")
        f.write(unicode(notes))
        now = datetime.datetime.now()
        f.write(u"Experiment began at {}\n".format(now.strftime("%Y-%m-%d %H:%M:%S")))
    
    experiments.to_csv(experiment_path / 'experiment.csv')

def write_report_coda(experiment_path):
    with (experiment_path / "report.txt").open(mode="a") as f:
        now = datetime.datetime.now()
        f.write(u"Experiment ended at {}\n".format(now.strftime("%Y-%m-%d %H:%M:%S")))

class aruco_experiment_node(object):
    def __init__(self):
        self.pan_unlock_id = 1
        self.tilt_unlock_id = 1
        self.cv_bridge = CvBridge()

        rospy.init_node('aruco_experiment_node', anonymous=True)

        self.spawn_service = ("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_service = ("/gazebo/delete_model", DeleteModel)

        self.pan_tilt_service = ("/robot/robobo/movePanTilt", MovePanTilt)
        
        rospy.Subscriber("/robot/robobo/tag", Marker, self.marker_callback)
        rospy.Subscriber("/robot/robobo/camera/image", Image, self.camera_callback)

        self.run_experiments()
        rospy.signal_shutdown("aruco_experiment finished recording. Run again for a new recording")

    def camera_callback(self, data):
        self.img = data

    def marker_callback(self, data):
        self.marker = data

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
        cv_image = self.cv_bridge.imgmsg_to_cv2(self.img, desired_encoding="bgr8")
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

        marker = self.marker
        
        file = path / "Marker.info"
        with file.open(mode="w+") as f:
            write_marker(f, marker, self.labels)
        self.write_image(path)

    def test_object(self, index_str, exp_path, x,y,z, orientation=(0,0,0,0)):
        spawn_object(self.spawn_service,self.curr_object, self.curr_object_path, x,y,z, orientation)
        rospy.sleep(5.0)
        self.write_experiment(exp_path, self.curr_object, index_str)
        delete_object(self.delete_service,self.curr_object)
        rospy.sleep(5.0)

    def run_experiments(self):
        init_x = 0
        init_y = 0
        init_z = 0.04 #0.975

        init_orientation = (0, 0, 0.7071068, -0.7071068) # 270 deg
        #init_orientation = (0, 0, 0, 0) # 270 deg

        self.curr_object = None
        self.curr_object_path = None
        self.curr_dist = 0
        self.curr_pan = None
        self.curr_tilt = None

        #Initialize pan/tilt
        self.set_pan_tilt(pan=None,tilt=90)

        now = datetime.datetime.now()
        experiment_path = pathlib2.Path("./aruco/experiments/{}".format(now.strftime("%d-%m-%Y_%H%M%S")))
        experiment_path.mkdir(parents=True, exist_ok=True)

        objects = [
                    ("velocidad50_aruco", "./../../robobo-gazebo-models/velocidad50_aruco/model.sdf"),
                    ("izquierda_aruco", "./../../robobo-gazebo-models/izquierda_aruco/model.sdf"),
                    ]

        distances = [0.2 , 0.25, 0.3 , 0.35, 0.4 , 0.45, 0.5 , 0.55, 0.6 ,0.65, 0.7 , 0.75, 0.8 , 0.85, 0.9 , 0.95]

        tilts = [None]
        pans = [None]

        has_pan = not (pans[0] == None)
        has_tilt = not (tilts[0] == None)
        
        notes = "Nothing more to report"
        experiments = create_experiment(experiment_path, objects, distances, pans, tilts)
        
        write_report(experiment_path, notes, objects, distances, tilts, pans, has_pan, has_tilt, experiments)

        for index, row in experiments.iterrows():
            if(not has_pan):
                self.curr_pan = None
            else:
                self.curr_pan = row["Pan (deg)"]

            if(not has_tilt):
                self.curr_tilt = None
            else:
                self.curr_tilt = row["Tilt (deg)"]

            self.curr_dist = row["Distancia (m)"]
            self.curr_object = row["Object"]
            self.curr_object_path = row["Object path"]
            if(has_pan or has_tilt):
                self.set_pan_tilt(pan=self.curr_pan,tilt=self.curr_tilt)
            self.test_object(self.curr_dist, experiment_path, init_x+self.curr_dist,init_y,init_z, orientation=init_orientation)
            
        write_report_coda(experiment_path)


def main():
    instancia = aruco_experiment_node()
    try:
        rospy.spin() # spin() simply keeps python from exiting until this node is stopped
    except rospy.ROSInterruptException:
        print("Shutting down Aruco test module")

if __name__ == '__main__':
    main()