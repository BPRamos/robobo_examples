#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

class charuco_calib(object):
    def __init__(self, robobo_name='robobo'):
        self.robobo_name = robobo_name
        self.bridge = CvBridge()
        rospy.init_node('charuco_calib', anonymous=True)

        # Topics
        rospy.Subscriber("/robot/robobo/camera/image", Image, self.camera_callback)

    def camera_callback(self, data):

        now = rospy.get_rostime()
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        cv2.imwrite("calibrationImgs/{}-{}.png".format(now.secs, now.nsecs), cv_image)
        rospy.signal_shutdown("charuco_calib finished recording. Run again for a new recording")
        

    def run(self):
        rospy.spin()



def main():
    instancia = charuco_calib()
    try:
        rospy.spin() # spin() simply keeps python from exiting until this node is stopped
    except rospy.ROSInterruptException:
        print("Shutting down Charuco calibration module")
        
if __name__ == '__main__':
    main()
