#!/home/logesh/rtb/rtb_env/bin/python3

'''
Description:
    This script contains the code to perform pick and place of aruco markers attached blocks.
    The setup has a Fanuc Robotic arm, over head realsense camera, inhand fingers camera.
    Execution steps:
        1. Realsense camera finds the aruco marker in the workspace.
        2. Coordinates are passed to robot in robot-frame (with the help of hand-eye calibration)
        3. After reaching the target position, inhand camera corrects the error by aligning to the aruco detected.
        4. Perform pick and drop.  
'''

import cv2
import time
import numpy as np
import spatialmath as sm
from fanuc_model import Fanuc
import matplotlib.pyplot as plt 
from utils.calibration import load_calib_data, find_aruco_pose


# robot model
robot = Fanuc()
print(robot)
robot.plot(robot.qz, block=True)


# camera's NOTE: camera id's will change
realCam = cv2.VideoCapture(8)
fingCam = cv2.VideoCapture(2)


# finger camera frame cx,cy
x_max = img_width = int(fingCam.get(cv2.CAP_PROP_FRAME_WIDTH))                       # x axis (image plane) -> along width 
y_max = img_height = int(fingCam.get(cv2.CAP_PROP_FRAME_HEIGHT))                     # y axis (image plane) -> along height
fing_cx, fing_cy = (int(x_max // 2), int(y_max // 2))

# camera intrinsics
# 1. Realsense 435iD
real_cam_mat = load_calib_data('./calib_data_realsense/camera_matrix.pkl')
real_dist_coef = load_calib_data('./calib_data_realsense/dist_coef.pkl')
# 2. Fingers
fing_cam_mat = load_calib_data('./calib_data_finger/camera_matrix.pkl')
fing_dist_coef = load_calib_data('./calib_data_finger/dist_coef.pkl')


# load realsense extrinsics
rotm = load_calib_data('./calib_data_realsense/hand_eye_rotm.pkl')
trans = load_calib_data('./calib_data_realsense/hand_eye_trans.pkl')
# create transform (tf)
mat = np.eye(4)
mat[0:3,0:3] = rotm
trans = trans.reshape(3)
mat[0:3, 3] = trans
base2cam = sm.SE3(mat)
print('Base to Camera:', base2cam, sep='\n')


# get frame (from camera)
def get_frame(cam: cv2.VideoCapture) -> np.ndarray:
    '''
    Description:
        This function reads the image frame from the input camera object.
    
    Args:
        - OpenCV camera objects.
    
    Returns:
        - Image frame (np.array)
    '''
    for _ in range(5):  # flush old frames
        cam.read()

    check, frame = cam.read()
    if (not check):
        print(f"No camera feed")
        exit(0)
    return frame


# get transform from camera to aruco (tf)
def get_aruco_tf(frame: np.ndarray, camMat: np.ndarray, distCoef: np.ndarray, mSize=35, arucoDict=cv2.aruco.DICT_4X4_100) -> sm.SE3:
    '''
    Description:
        This function find the aruco in the image frame and returns its transform from camera frame.
        Uses the intrinsic parameters of the camera to find the aruco pose using OpenCV.
    
    Args:
        - Camera frame
        - Camera intrinsics
        - Aruco properties     

    Returns:
        - Cam2Aruco transform (sm.SE3)
    '''
    try:
        marker_rot, marker_trans = find_aruco_pose(frame, camMat, distCoef, mSize, arucoDict)
        mat1 = np.eye(4)
        mat1[0:3,0:3] = marker_rot
        marker_trans = marker_trans.reshape(3)
        mat1[0:3, 3] = marker_trans
        cam2aruco = sm.SE3(mat1)
        print('Camera to Aruco: ', cam2aruco, sep='\n')

        return cam2aruco
    
    except Exception as E:
        print(f"No aruco found in scene")
        return None


# get aruco cx, cy
def get_aruco_cxcy(frame: np.ndarray, mSize=35, arucoDict=cv2.aruco.DICT_4X4_100) -> tuple:
    '''
    Description:
        This function detects the corners of the target aruco and return the p_cx, p_cy in pixels.
    
    Args: 
        - Image frame (np.ndarray)

    Returns:
        - (cx, cy) -> center of aruco
    '''
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(arucoDict)          
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    (corners, ids, rejected) = detector.detectMarkers(gray)
    # draw detected aruco
    cv2.aruco.drawDetectedMarkers(frame, corners=corners)

    # compute cxcy
    (topLeft, topRight, bottomRight, bottomLeft) = corners[0][0]
    cx = int(topLeft[0]) + (int(bottomRight[0]) - int(topLeft[0])) // 2
    cy = int(topLeft[1]) + (int(bottomRight[1]) - int(topLeft[1])) // 2
    # mark the center
    cv2.circle(frame, [cx, cy], 6, [0, 0, 255], -1)

    return (cx, cy) 


# show image frame
def show_image(frame: np.ndarray, block: bool = False, pause_time: int = 5) -> None:
    '''
    Description: 
        The function shows the image for seconds `time`.
    
    Args:
        - Image frame (np.ndarray)
        - Block image (bool)
        - pause_time (in seconds)
    '''

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)      # BGR to RGB
    plt.grid(False)
    plt.imshow(frame); plt.show(block=block)
    plt.pause(pause_time)
    plt.close()


# main function to perform pick and place
def main():
    # Step 1: get frame (RealSense Camera feed)
    frame = get_frame(cam = realCam)

    # Step 2: find aruco tf in image frame
    cam2aruco = get_aruco_tf(
        frame = frame,
        camMat = real_cam_mat,
        distCoef = real_dist_coef,
        mSize = 35
    )
    while (cam2aruco == None):
        # repeat Step1 and Step2
        frame = get_frame(cam = realCam)
        cam2aruco = get_aruco_tf(
            frame = frame,
            camMat = real_cam_mat,
            distCoef = real_dist_coef,
            mSize = 35
        )
        time.sleep(1)
    # visualize aruco frame
    show_image(frame = frame, pause_time = 2)

    # Step 3: compute base to aruco transform [cam2aruco @ base2cam]
    base2aruco = base2cam * cam2aruco
    print('Base to Aruco:', base2aruco, sep='\n')

    # Step 3.1: mm to m (robot uses `m` units)
    base2aruco.t = base2aruco.t / 1000
    base2aruco.t[2] += 0.05             # move z by 10 cm

    # Step 3.2: perform rotation
    aruco_ang = (base2aruco.eulervec()).copy()
    # base2aruco.R = sm.SO3.RPY([0, 0, 0], order="xyz")
    # rotate y by 180
    base2aruco = base2aruco * sm.SE3.RPY([0, np.pi, 0], order="xyz")
    # rotate z by 45 deg
    base2aruco = base2aruco * sm.SE3.RPY([-np.pi/4, 0, 0], order="xyz")

    # Step 4: Send base2aruco tf to robot
    robot.move_to(base2aruco)
    

    ### IMP:
    # perform alignment (using fingers camera)
    print("Inside x-y alignment")
    robot.show_plot = False
    while True:
        # 1. take picture from finger camera (in-hand camera)
        f_frame = get_frame(fingCam)
        ###
        # rotate image frame
        # get ee rotation angle (z-enough)
        ang = robot.cur_tf.eulervec()[0]                # NOTE: [z, y, x]
        # rotate image
        M = cv2.getRotationMatrix2D(center=[320, 240], angle=np.degrees(ang)+np.degrees(45), scale=1.0)
        f_frame = cv2.warpAffine(f_frame, M, (img_width, img_height))
        show_image(f_frame)
        ###

        # 2. find aruco cxcy (pixel values)
        (cx, cy) = get_aruco_cxcy(frame = f_frame, mSize=35)

        # plot the frame (imp to visualize)
        cv2.line(f_frame, pt1=[int(x_max//2), 0], pt2=[int(x_max//2), int(y_max)], color=[0,0,255])
        cv2.line(f_frame, pt1=[0, int(y_max//2)], pt2=[int(x_max), int(y_max//2)], color=[0,255,0])
        cv2.circle(f_frame, [fing_cx, fing_cy], 6, [255, 0, 0], -1)
        # draw tolerance circle
        cv2.circle(f_frame, [fing_cx, fing_cy], 30, [0, 165, 255], 2)
        show_image(f_frame, pause_time=2, block=False)

        # NOTE: Sample control for alignment
        # should implement base case -> stop alignment if inside tolerance circle (IMP)
        # increment of robot_x, robot_y should be determined by the difference of (fing_cx, cx) & (fing_cy, cy) 
        # coordinate might be problem (coordinate of image plane and robot base are different)
        
        # get current ee-pose
        # cur_tf = robot.get_curpos(only_tf=True)
        cur_tf = robot.cur_tf                       # NOTE: did not use tf from robot (which is wrong)


        # NOTE: BASE CONDITION   -> Logic might be wrong
        tol = tolerance = 15          # in px
        if (cx >= (fing_cx-tol) and cx <= (fing_cx+tol)):
            if (cy >= (fing_cy-tol) and cy <= (fing_cy+tol)):
                # stop the execution
                print("X-Y Alignment Over")        
                break


        # update position (NOTE: initial updates 5mm)
        if (cx < fing_cx and cy < fing_cy):
            # robot_x + 1 & robot_y + 1
            cur_tf.t[0] += 0.005
            cur_tf.t[1] += 0.005
        elif (cx > fing_cx and cy > fing_cy):
            # robot_x - 1 & robot_y - 1
            cur_tf.t[0] -= 0.005
            cur_tf.t[1] -= 0.005
        elif (cx < fing_cx and cy > fing_cy):
            # robot_x - 1 & robot_y + 1
            cur_tf.t[0] -= 0.005
            cur_tf.t[1] += 0.005
        elif (cx > fing_cx and cy < fing_cy):
            # robot_x + 1 & robot_y - 1
            cur_tf.t[0] += 0.005
            cur_tf.t[1] -= 0.005
        else:
            # stop the execution 
            print("X-Y Alignment Over")        
            break

        # send updated position
        robot.move_to(cur_tf)  
    print("Outside x-y alignment")

    ###
    '''
    print("Performing rotation adjustment")
    # rotate the ee to aruco pose after alignment
    robot.cur_tf.R = sm.base.rotz(aruco_ang[2])
    # rotate y by 180
    robot.cur_tf *= sm.SE3.RPY([0, np.pi, 0], order="xyz")
    # rotate z by 15 deg
    robot.cur_tf *= sm.SE3.RPY([np.pi/4, 0, 0], order="xyz")
    robot.move_to(robot.cur_tf)
    time.sleep(2)
    '''
    ###

    exit(0)
    # perform z-adjustment (using fingers camera) -> can also reduce the `extra z` added in the begining of program
    print("Inside Z alignment")

    # Step 1: get recent image frame
    f_frame = get_frame(fingCam)
    # Step 2: find aruco xyz
    rotm, trans = find_aruco_pose(
        frame = f_frame,
        camera_matrix = fing_cam_mat,
        dist_coeffs = fing_dist_coef,
        marker_length = 35,
        marker_dict=cv2.aruco.DICT_4X4_100
    )
    # Step 3: Calculating z-dist from gripper (NOTE: need to check the unit) -> convert every thing to meters
    slant_z = trans[2] / 10        # mm to cm
    z = np.sqrt((slant_z ** 2) - (7.5 ** 2)) - 5        # 7.5 & 5 are hand measured values (in cm)
    robot.cur_tf.t[2] -= (z / 100)       # cm to m
    robot.move_to(robot.cur_tf)
    robot.show_plot = True

    print("Outside Z alignment")

    ###

    # Gripper Control (on)

    # intermediate position

    # drop position

    # gripper control (off)


if __name__ == "__main__":
    main()


# release camera objects
realCam.release()
fingCam.release()