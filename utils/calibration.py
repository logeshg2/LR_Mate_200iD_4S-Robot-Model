import os
import pickle
# import spatialmath as sm
from fanuc_model import Fanuc
import numpy as np
import cv2

def save_calib_data(calib_data: object, calib_data_path: str):
    """Saves calibration data."""
    with open(calib_data_path, "wb") as f:
        pickle.dump(calib_data, f)


def load_calib_data(calib_data_path: str):
    """Loads calibration data."""
    with open(calib_data_path, "rb") as f:
        calib_data = pickle.load(f)
    return calib_data


def draw_axis(img, corners, imgpts, thickness=5):
    """Draws xyz axis."""
    # NOTE: opencv in BGR order, B-z, G-y, R-x
    corner = tuple(corners[0].astype(int).ravel())
    img = cv2.line(
        img,
        corner,
        tuple(imgpts[0].astype(int).ravel()),
        (0, 0, 255),
        thickness=thickness,
    )
    img = cv2.line(
        img,
        corner,
        tuple(imgpts[1].astype(int).ravel()),
        (0, 255, 0),
        thickness=thickness,
    )
    img = cv2.line(
        img,
        corner,
        tuple(imgpts[2].astype(int).ravel()),
        (255, 0, 0),
        thickness=thickness,
    )
    return img


def collect_checker_board_images(camera, save_dir):
    """Collects checkerboard images."""

    print("Type [c] to capture, [q] to exit.")
    array = []
    count = 1
    while True:
        _, frame = camera.read()
        cv2.imshow("Frame", frame)

        key = cv2.waitKey(1)
        if key == ord("q"):
            break
        elif key == ord("c"):
            cv2.imwrite(os.path.join(save_dir, f"image_{count}.png"), frame)
            array.append(frame)
            print(f"Saved data #{count}")
            count += 1

    camera.release()
    cv2.destroyAllWindows()
    return array


def calibrate_camera_checkerboard(images, cols, rows, square_size, verbose=True):
    """Calibrates camera to get camera matrix and distortion coefficients."""

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp = objp * square_size

    # arrays to store object points and image points from all the images
    objpoints = []
    imgpoints = []

    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, (cols, rows), None)

        if ret:
            objpoints.append(objp)

            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners)

            out_img = img.copy()
            if verbose:
                cv2.drawChessboardCorners(out_img, (cols, rows), corners, ret)
                cv2.imshow("Calibration", out_img)
                cv2.waitKey(0)

    cv2.destroyAllWindows()

    rmse, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    return rmse, camera_matrix, dist_coeffs

### NOTE:
# calibration aruco -> DICT_6X6_100
# pick and place aruco -> DICT_4X4_50

def find_aruco_pose(frame, camera_matrix, dist_coeffs, marker_length, marker_dict=cv2.aruco.DICT_4X4_50):
    """Finds aruco marker pose."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(marker_dict)          
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    (corners, ids, rejected) = detector.detectMarkers(gray)

    # pose of aruco
    marker_size = marker_length  # in mm
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    R_target2cam, t_target2cam = None, None
    if np.all(ids is not None):
        for i in range(len(ids)):
            ret, rvec, tvec = cv2.solvePnP(marker_points, corners[i], camera_matrix, dist_coeffs, False, cv2.SOLVEPNP_IPPE_SQUARE)    
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec,30,3)
            R_target2cam, _ = cv2.Rodrigues(rvec)
            t_target2cam = tvec.squeeze()

    return R_target2cam, t_target2cam


def find_checkerboard_pose(frame, camera_matrix, dist_coeffs, cols, rows, square_size):
    """Finds checkerboard pose."""
    R_target2cam, t_target2cam = None, None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp = objp * square_size

    axis = np.float32(
        [[3 * square_size, 0, 0], [0, 3 * square_size, 0], [0, 0, -3 * square_size]]
    ).reshape(-1, 3)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, (cols, rows), None)

    if ret:
        # refine corners
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # find pose
        ret, rvec, tvec = cv2.solvePnP(objp, corners, camera_matrix, dist_coeffs)
        R_target2cam, _ = cv2.Rodrigues(rvec)
        t_target2cam = tvec.squeeze()

        imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
        frame = draw_axis(frame, corners, imgpts)

    return R_target2cam, t_target2cam


def collect_eye_hand_data(
    camera,
    camera_matrix,
    dist_coeffs,
    marker_length,
    robot,
    aruco,
    cols,
    rows,
    square_size,
):
    """Collects eye hand calibration data."""
    target_poses = []
    robot_ee_poses = []
    count = 0
    while True:
        _, frame = camera.read()
        if aruco:
            R_target2cam, t_target2cam = find_aruco_pose(
                frame=frame,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
                marker_length=marker_length,
            )
        else:
            R_target2cam, t_target2cam = find_checkerboard_pose(
                frame=frame,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
                cols=cols,
                rows=rows,
                square_size=square_size,
            )
        cv2.imshow("frame", frame)

        key = cv2.waitKey(1)
        if key == ord("q"):
            break
        if key == ord("c"):
            if R_target2cam is None:
                print("None pose detected, try again")
            else:
                target_poses.append((R_target2cam, t_target2cam))

                robot.connect()
                xyzrpw = robot.get_curpos()
                robot_ee_poses.append(xyzrpw)
                count += 1
                print(f"Collected data: {count}")

    return target_poses, robot_ee_poses


def collect_eye_hand_data_fanuc(
    camera,
    camera_matrix,
    dist_coeffs,
    marker_length,
    aruco,
    cols,
    rows,
    square_size,
    image_save_dir
):  
    """Collects eye hand calibration data."""
    target_poses = []
    robot_ee_poses = []
    count = 0
    # robot model
    robot = Fanuc()
    while True:
        _, frame = camera.read()
        if aruco:
            R_target2cam, t_target2cam = find_aruco_pose(
                frame=frame,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
                marker_length=marker_length,
                marker_dict=cv2.aruco.DICT_6X6_100
            )
        else:
            R_target2cam, t_target2cam = find_checkerboard_pose(
                frame=frame,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
                cols=cols,
                rows=rows,
                square_size=square_size,
            )
        cv2.imshow("frame", frame)

        key = cv2.waitKey(1)
        if key == ord("q"):
            break
        if key == ord("c"):
            if R_target2cam is None:
                print("None pose detected, try again")
            else:
                target_poses.append((R_target2cam, t_target2cam))

                # robot.connect()
                ee_pose = robot.get_curpos()            # rotm, translation
                ee_pose[1] = ee_pose[1] * 1000                              # m to mm
                # debug
                # print(ee_pose[1])   
                robot_ee_poses.append(ee_pose)
                count += 1
                print(f"Collected data: {count}")
                # save the image while performing hand eye calibration
                cv2.imwrite(f"{image_save_dir}/image{count}.png", frame)  
                
    return target_poses, robot_ee_poses


def calibrate_eye_hand(
    R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, eye_to_hand=True
):
    tts = []
    if eye_to_hand:
        # change coordinates from gripper2base to base2gripper
        R_base2gripper, t_base2gripper = [], []
        for R, t in zip(R_gripper2base, t_gripper2base):
            R_b2g = R.T
            t_b2g = -R_b2g @ t
            tts.append(t_b2g)
            R_base2gripper.append(R_b2g)
            t_base2gripper.append(t_b2g)

        # change parameters values
        R_gripper2base = R_base2gripper
        t_gripper2base = t_base2gripper

    # calibrate
    R, t = cv2.calibrateHandEye(
        R_gripper2base=R_gripper2base,
        t_gripper2base=t_gripper2base,
        R_target2cam=R_target2cam,
        t_target2cam=t_target2cam,
        method=cv2.CALIB_HAND_EYE_DANIILIDIS     # ANDREFF method give accurate values
    )

    return R, t

if __name__ == "__main__":
    R_g2b = load_calib_data("./calib_data/hand_eye_data/R_g2b.pkl")
    R_t2c = load_calib_data("./calib_data/hand_eye_data/R_t2c.pkl")
    t_g2b = load_calib_data("./calib_data/hand_eye_data/t_g2b.pkl")
    t_t2c = load_calib_data("./calib_data/hand_eye_data/t_t2c.pkl")

    R, t = calibrate_eye_hand(
        R_gripper2base=R_g2b,
        t_gripper2base=t_g2b,
        R_target2cam=R_t2c,
        t_target2cam=t_t2c,
        eye_to_hand=True
    )
    print("ANDREFF: ")
    print(R,t,sep='\n')

    # save_calib_data(R, "./calib_data/hand_eye_rotm.pkl")
    # save_calib_data(t, "./calib_data/hand_eye_trans.pkl")