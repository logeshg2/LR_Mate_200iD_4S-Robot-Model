#!/home/logesh/rtb/rtb_env/bin/python3

# This script contains example movements scritp using Fanuc() model

import cv2
import numpy as np
import spatialmath as sm
from fanuc_model import Fanuc
import matplotlib.pyplot as plt 
from utils.calibration import load_calib_data, find_aruco_pose


# robot model
robot = Fanuc()
print(robot)
robot.plot(robot.qz, block=True)


###
# simple aruco pick and place
cam = cv2.VideoCapture(6)
# camera intrinsics
cam_mat = load_calib_data('./calib_data/camera_matrix.pkl')
dist_coef = load_calib_data('./calib_data/dist_coef.pkl')
# load hand eye calibration
rotm = load_calib_data('./calib_data/hand_eye_rotm.pkl')
trans = load_calib_data('./calib_data/hand_eye_trans.pkl')
# create transform (tf)
mat = np.eye(4)
mat[0:3,0:3] = rotm
trans = trans.reshape(3)
mat[0:3, 3] = trans
base2cam = sm.SE3(mat)
print('Base to Camera:', base2cam, sep='\n') 
# get frame
check, frame = cam.read()
while not check:
    check, frame = cam.read()
# get pose of aruco marker
marker_rot, marker_trans = find_aruco_pose(frame, cam_mat, dist_coef, 35, cv2.aruco.DICT_4X4_100)
frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
plt.grid(False)
plt.imshow(frame)
plt.show()
cam.release()

mat1 = np.eye(4)
mat1[0:3,0:3] = marker_rot
marker_trans = marker_trans.reshape(3)
mat1[0:3, 3] = marker_trans
print(mat1)
cam2aruco = sm.SE3(mat1)
print('Camera to Aruco: ', cam2aruco, sep='\n')
# base to aruco [cam2aruco @ base2cam]
base2aruco = base2cam * cam2aruco
print('Base to Aruco:', base2aruco, sep='\n')
# mm to m
base2aruco.t = base2aruco.t / 1000
base2aruco.t[1] += 0.015            # move y by 2 cm
base2aruco.t[2] += 0.05             # move z by 5 cm
# rotate y by 180
base2aruco = base2aruco * sm.SE3.RPY([0, np.pi, 0], order="xyz")
# rotate z by 45 deg
base2aruco = base2aruco * sm.SE3.RPY([np.pi/4, 0, 0], order="xyz")
# send base2aruco
robot.move_to(base2aruco)
###
cam.release()