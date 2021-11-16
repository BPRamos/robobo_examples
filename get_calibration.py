#!/usr/bin/env python

import cv2
import glob


arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
board = cv2.aruco.CharucoBoard_create(8, 11, 0.015, 0.012, arucoDict)
arucoParams = cv2.aruco.DetectorParameters_create()

print(board)

corners = []
ids = []
imageSize = None
for filename in glob.glob('calibrationImgs/*.png'):
    img = cv2.imread(filename)
    imageSize = img.shape[::][:-1]
    print(imageSize)
    (tagCorners, tagIds, rejected) = cv2.aruco.detectMarkers(img, arucoDict,parameters=arucoParams)
    if(len(tagCorners)>0):
        (retval , charucoCorners, charucoIds) = cv2.aruco.interpolateCornersCharuco(tagCorners, tagIds, img, board)
        if(charucoIds is not None):
                corners.append(charucoCorners);
                ids.append(charucoIds);

#print(corners)
if (len(corners) > 4):
    # Now that we've seen all of our images, perform the camera calibration
    # based on the set of points we've discovered
    calibration, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
            charucoCorners=corners,
            charucoIds=ids,
            board=board,
            imageSize=imageSize,
            cameraMatrix=None,
            distCoeffs=None)
    # Print matrix and distortion coefficient to the console
    print(cameraMatrix)
    print(distCoeffs)
    with open("camMatrix.txt", "w") as f:
        f.write("CameraMatrix:\n{}\n".format(cameraMatrix))
        f.write("DistCoefficients: {}\n".format(distCoeffs))