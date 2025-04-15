#!/home/logesh/rtb/rtb_env/bin/python3

'''
    The script for executing calibration and uses methods from "calibration.py" file.
    Calibration like:
        > Camera Calibration
        > Hand - Eye Calibration

    Procedure: (Camera + Robot)
        - Collect calibration data 
        - Perform calibration using OpenCV 
    
    Example calibration data can be viewed in './calib_data_realsense/' and './calib_data_finger/'
'''

# NOTE:
# choose calibration pattern properly
# 1. Aruco marker size (eg: 70mm)
# 2. Chessboard square_size and (nrow's & ncol's)

import cv2
import argparse
import numpy as np
from utils.calibration import *

# CLI arguments
arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--device", default=0, type=int, help="Camera device ID", required=True)
cam_id = arg_parser.parse_args().device

# choose the camera
realsense_camera = True 
finger_camera = False 

# flags to turn on calibration
camera_calib = False
hand_eye_calib = True

# camera object
cam = cv2.VideoCapture(cam_id)    

# load camera parameters
if realsense_camera and hand_eye_calib:
    cam_matrix = load_calib_data("./calib_data_realsense/camera_matrix.pkl")
    dist_coef = load_calib_data("./calib_data_realsense/dist_coef.pkl")
elif finger_camera and hand_eye_calib:
    cam_matrix = load_calib_data("./calib_data_finger/camera_matrix.pkl")
    dist_coef = load_calib_data("./calib_data_finger/dist_coef.pkl")
# print(cam_matrix, dist_coef, sep='\n')


# function to perform camera calibration
def perform_camera_calibration():
    # save directory:
    if realsense_camera:
        save_dir = "./calib_data_realsense/checkboard_data/"
        cam_mat_dir = "./calib_data_realsense/camera_matrix.pkl"
        dist_coef_dir = "./calib_data_realsense/dist_coef.pkl"
    else:
        save_dir = "./calib_data_finger/checkboard_data/"
        cam_mat_dir = "./calib_data_finger/camera_matrix.pkl"
        dist_coef_dir = "./calib_data_finger/dist_coef.pkl"

    # collect data
    image_array = collect_checker_board_images(
        camera = cam,
        save_dir = save_dir
    )
    image_array = np.array(image_array)

    # perform calibration
    _, cam_mat, dist_cof = calibrate_camera_checkerboard(
        images = image_array,
        cols = 9,
        rows = 6,
        square_size = 25
    )

    # save calibration
    save_calib_data(cam_mat, cam_mat_dir)
    save_calib_data(dist_cof, dist_coef_dir)

    # debug
    print(cam_mat, dist_cof, sep="\n")


# function to perform hand-eye calibration
def perform_hand_eye_calibration():
    # hand eye image data save dir
    if realsense_camera:
        img_save_dir = "./calib_data_realsense/hand_eye_data/images"
    else:
        img_save_dir = "./calib_data_finger/hand_eye_data/images"
    # collect data
    target_poses, robot_ee_poses = collect_eye_hand_data_fanuc(
        camera = cam,
        camera_matrix = cam_matrix,
        dist_coeffs = dist_coef,
        marker_length = 70,
        aruco = True,
        cols = 9,
        rows = 6,
        square_size = 25,
        image_save_dir = img_save_dir
    )

    # splitting tf array's 
    R_g2b = []
    t_g2b = []
    R_t2c = []
    t_t2c = []
    for each_pose in target_poses:
        R_t2c.append(each_pose[0])          # rotation
        t_t2c.append(each_pose[1])          # translation
    for each_pose in robot_ee_poses:
        R_g2b.append(each_pose[0])
        t_g2b.append(each_pose[1])

    # save calibration tf's
    if realsense_camera:
        save_calib_data(R_g2b, "./calib_data_realsense/hand_eye_data/R_g2b.pkl")
        save_calib_data(R_t2c, "./calib_data_realsense/hand_eye_data/R_t2c.pkl")
        save_calib_data(t_g2b, "./calib_data_realsense/hand_eye_data/t_g2b.pkl")
        save_calib_data(t_t2c, "./calib_data_realsense/hand_eye_data/t_t2c.pkl")
    else:
        save_calib_data(R_g2b, "./calib_data_finger/hand_eye_data/R_g2b.pkl")
        save_calib_data(R_t2c, "./calib_data_finger/hand_eye_data/R_t2c.pkl")
        save_calib_data(t_g2b, "./calib_data_finger/hand_eye_data/t_g2b.pkl")
        save_calib_data(t_t2c, "./calib_data_finger/hand_eye_data/t_t2c.pkl")

    # perform hand to eye calibration
    R_b2c, t_b2c = calibrate_eye_hand(
        R_gripper2base = R_g2b,
        t_gripper2base = t_g2b,
        R_target2cam = R_t2c,
        t_target2cam = t_t2c,
        eye_to_hand = True   
    )

    # save calibration data
    if realsense_camera:
        save_calib_data(R_b2c, "./calib_data_realsense/hand_eye_rotm.pkl")
        save_calib_data(t_b2c, "./calib_data_realsense/hand_eye_trans.pkl")
    else:
        save_calib_data(R_b2c, "./calib_data_finger/hand_eye_rotm.pkl")
        save_calib_data(t_b2c, "./calib_data_finger/hand_eye_trans.pkl")
    # debug
    print(R_b2c, t_b2c, sep='\n')# calibration procedure 


if __name__ == "__main__":
    if (not camera_calib and not hand_eye_calib):
        print("Enable Calibration Flags")
        exit(0)
    
    # camera
    if (camera_calib):
        perform_camera_calibration()
    
    # robot hand-eye
    if (hand_eye_calib):
        perform_hand_eye_calibration()