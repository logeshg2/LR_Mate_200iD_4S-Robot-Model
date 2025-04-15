#!/home/logesh/rtb/rtb_env/bin/python3

import cv2
import numpy as np
from utils.calibration import load_calib_data, find_aruco_pose  

cam = cv2.VideoCapture(2)

# camera parameters
cam_matrix = load_calib_data("./calib_data_realsense/camera_matrix.pkl")
dist_coef = load_calib_data("./calib_data_realsense/dist_coef.pkl")

while True:
    _, frame = cam.read()

    rotm, trans = find_aruco_pose(
        frame = frame,
        camera_matrix = cam_matrix,
        dist_coeffs = dist_coef,
        marker_length = 35,
        marker_dict=cv2.aruco.DICT_4X4_100
    )
    print(trans)

    cv2.imshow("Output", frame)
    key = cv2.waitKey(1)
    if (key == ord('q')):
        break

cv2.destroyAllWindows()
cam.release()