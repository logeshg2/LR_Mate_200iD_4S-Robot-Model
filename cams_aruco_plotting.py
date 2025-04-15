# This script plots camera's and arucos in robot frame.

import cv2
import numpy as np
import spatialmath as sm
import matplotlib.pyplot as plt
from utils.calibration import load_calib_data, find_aruco_pose

# first load camera intrinsics
real_camMat = load_calib_data("./calib_data_realsense/camera_matrix.pkl")
fing_camMat = load_calib_data("./calib_data_finger/camera_matrix.pkl")
real_distCoef = load_calib_data("./calib_data_realsense/dist_coef.pkl")
fing_distCoef = load_calib_data("./calib_data_finger/dist_coef.pkl")

# load base to camera transforms and create transform (tf)
# 1. base to realsense
b2rc_rotm = load_calib_data('./calib_data_realsense/hand_eye_rotm.pkl')
b2rc_trans = load_calib_data('./calib_data_realsense/hand_eye_trans.pkl')
mat = np.eye(4)
mat[0:3,0:3] = b2rc_rotm
trans = b2rc_trans.reshape(3)
mat[0:3, 3] = trans
base2realCam = sm.SE3(mat)
# print("Base to Realsense Cam TF:", base2realCam, sep='\n')

# 1. base to finger
b2fc_rotm = load_calib_data('./calib_data_finger/hand_eye_rotm.pkl')
b2fc_trans = load_calib_data('./calib_data_finger/hand_eye_trans.pkl')
mat = np.eye(4)
mat[0:3,0:3] = b2fc_rotm
trans = b2fc_trans.reshape(3)
mat[0:3, 3] = trans
base2fingCam = sm.SE3(mat)
# print("Base to finger Cam TF:", base2fingCam, sep='\n')

# helper functions:
# plot function
def plot_frames(aruco_tf, frame_name):
    # plot cameras in robot base frame
    sm.SE3().plot(frame="Robot", width=2, length=100, color='yellow', style="rviz") #  dims=[0,800,0,800,0,800],
    base2realCam.plot(frame='RC', width=2, length=100, color='blue', style="rviz")
    base2fingCam.plot(frame='FC', width=2, length=100, color='black', style="rviz")
    aruco_tf.plot(frame=frame_name, width=2, length=80, color='black', style="rviz")

# aruco tf function
def get_aruco_tf(frame, cam_mat, dist_coef) -> sm.SE3:
    try:
        # find aruco    
        rot, trans = find_aruco_pose(
            frame=frame,
            camera_matrix=cam_mat,
            dist_coeffs=dist_coef,
            marker_length=35,
            marker_dict=cv2.aruco.DICT_4X4_50
        )
        # generate cam2aruco transform
        mat1 = np.eye(4)
        mat1[0:3,0:3] = rot
        trans = trans.reshape(3)
        mat1[0:3, 3] = trans
        return sm.SE3(mat1)
    except :
        return sm.SE3()


# camera objects
fc_cam = cv2.VideoCapture(1)
rc_cam = cv2.VideoCapture(7)


# get frames in both camera 
check, fc_frame = fc_cam.read()
# 1. finger
while not check:
    check, fc_frame = fc_cam.read()
# 2. realsense
check, rc_frame = rc_cam.read()
while not check:
    check, rc_frame = rc_cam.read()


# convert into gray scale
# fc_frame = cv2.cvtColor(fc_frame, cv2.COLOR_BGR2GRAY)
# rc_frame = cv2.cvtColor(rc_frame, cv2.COLOR_BGR2GRAY)


# find aruco in both the cameras 
# cam2aruco_1 = get_aruco_tf(fc_frame, fing_camMat, fing_distCoef)
# cam2aruco_2 = get_aruco_tf(rc_frame, real_camMat, real_distCoef) 


# show frames
# plt.imshow(fc_frame, cmap="gray"); plt.show()
# plt.imshow(rc_frame, cmap="gray"); plt.show()


# base2aruco transforms
# base2aruco_1 = base2fingCam @ cam2aruco_1
# base2aruco_2 = base2realCam @ cam2aruco_2


# plot aruco's
# plot_frames(base2aruco_1, "fing_aruco")
# plot_frames(base2aruco_2, "real_aruco")
# plt.show()


# plot in realtime
while True:
    check, fc_frame = fc_cam.read()
    cam2aruco_1 = get_aruco_tf(fc_frame, fing_camMat, fing_distCoef)
    base2aruco_1 = base2fingCam @ cam2aruco_1
    plot_frames(base2aruco_1, "fing_aruco")
    cam2aruco_2 = get_aruco_tf(rc_frame, real_camMat, real_distCoef) 
    base2aruco_2 = base2realCam @ cam2aruco_2
    plot_frames(base2aruco_2, "real_aruco")   
    
    plt.pause(0.05)
    plt.cla()