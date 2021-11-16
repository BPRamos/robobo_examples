#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image 
from aruco_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovariance, Quaternion

from cv_bridge import CvBridge

import cv2
import numpy as np
from cv2 import aruco

import tf


class ROBOBO_ARUCOS_INTERCEPT(object):
    def __init__(self, robobo_name='robobo2', calibration_image_path=None,dictionary=aruco.DICT_4X4_100, marker_length=100):
        #self.camera_matrix = np.array([[53.31260399, 0., 337.47022316],[0., 207.91343066, 242.31044794],[0., 0.,1.]])
        #self.dist_coeff = np.array([[-3.22032638e-03, -1.47422975e-03, -2.45554419e-03, 1.42331365e-02, 4.98036513e-05]])

        self.robobo_name = robobo_name
        rospy.init_node('RoboboArucosIntercept', anonymous=True)

        self.bridge = CvBridge()
        self.aruco_dict = aruco.Dictionary_get(dictionary)
        self.parameters = aruco.DetectorParameters_create()
        self.parameters.minDistanceToBorder = 3
        self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        self.marker_length = marker_length

        # Topics
        rospy.Subscriber("/robot/robobo2/camera/image/compressed", Image, self.camera_callback)
        
    def camera_callback(self, data):
        print("received image of type: {}".format(data.format))

        np_arr = np.fromstring(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        # Arucos detection
        #                               cv2.aruc.detectMarkers(mat, Aruco.getPredefinedDictionary(currentTagDict), markerCorners, markerIds, parameters, rejectedCandidates, calibrationData.getCameraMatrixMat(), calibrationData.getDistCoeffsMat());
        (corners_list, ids, rejected) = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)

        if(ids is not None):
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners_list, self.marker_length, self.camera_matrix, self.dist_coeff)

            print(rvecs)
            print("---")
            print(tvecs)
            print("---")
            print(ids)
            print("-----")
            for (id, rvec, tvec) in zip(ids, rvecs, tvecs):
                print("{}:\t{}\t{}".format(id[0], rvec, tvec[0]))
                print(tvec[0][0])
                print(tvec[0][1])
                print(tvec[0][2])
            print("###############\n\n")

    def set_dictionary(self, dictionary):
        self.aruco_dict = aruco.Dictionary_get(dictionary)

    def run(self):
        rospy.spin()



def main():
    instancia = ROBOBO_ARUCOS_INTERCEPT()
    try:
        rospy.spin() # spin() simply keeps python from exiting until this node is stopped
    except rospy.ROSInterruptException:
        print("Shutting down Robobo Arucos Intercept module")
        
if __name__ == '__main__':
    main()
