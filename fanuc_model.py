import cv2
import time
import numpy as np
import spatialmath as sm
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from pycomm3 import LogixDriver
from roboticstoolbox import DHRobot, RevoluteDH, RevoluteMDH

# define the  robot class
class Fanuc(DHRobot):
    """
        -----
        Fanuc LR Mate 200iD 4s robot's model using DH parameters
        -----

        Description:
            Robot model developed using Robotics Toolbox by Peter Corke. This is the Python implementation of the robot model.
            For DH parameters refer this site: https://www.fanucamerica.com/cmsmedia/datasheets/LR%20Mate%20200iD%20Series_187.pdf and the below model also.
            
            The script / Fanuc Model also contains some extra abstracted feature for simpler use like move_to(), get_curpos().

        Author:
            - Logesh G
    """

    def __init__(self):
        # DH parameters as revolute joints
        Links = [
            RevoluteDH(
                d=0,
                a=0,
                alpha=np.pi/2,
                qlim=np.array(np.radians([-170, 170]))
            ),
            RevoluteDH(
                d=0.0,
                a=0.260,
                alpha=0,
                offset=np.pi/2,
                qlim=np.array(np.radians([-100, 145]))
            ),
            RevoluteDH(
                d=0.0,
                a=0.020,
                alpha=-np.pi/2,
                qlim=np.array(np.radians([-140, 140]))      # -140, 200
            ),
            RevoluteDH(
                d=-0.290,
                a=0.0,
                alpha=np.pi/2,
                qlim=np.array(np.radians([-150, 150]))      # -180, 180
            ),
            RevoluteDH(
                d=0.0,
                a=0.0,
                alpha=-np.pi/2,
                qlim=np.array(np.radians([-120, 120]))
            ),
            RevoluteDH(
                d=-0.070,
                a=0.0,
                alpha=np.pi,
                qlim=np.array(np.radians([-360, 360]))
            ),
        ]

        tool = sm.base.transl(0, 0, 0.13)                   # no rotation for servo gripper  [tool_offset = 13cm] -> tip of the gripper
        # tool = sm.base.transl(0, 0, 0)                    # aruco board tool (for calibration)

        super().__init__(
            links=Links,
            name="LR_Mate_200iD_4s",
            manufacturer="Fanuc",
            tool = tool
        )

        self.qz = np.zeros(6)
        self.addconfiguration("qz", self.qz)
        self.PLC_IP = '192.168.1.7'                         # ASRS PLC IP (Laptop -> ASRS PLC -> Fanuc PLC)


    # inverese kinematics function
    def __inverse_solver(self, tf):
        '''
        Description:
            - This is a inverse kinematics calculation function for the given transform as input.
            - The script specifically uses 'rtb.ikine_LM()' from 'DHRobot' base class.
        Args:
            - tf : Transform (from base_link to ee) -> SE3 (transform - 4x4)
        Returns:
            - goalconfig : Array (ndarray of len 6)
        '''
        
        print(f"Original TF:\n{tf}")
        qz = self.qz
        mask = [1, 1, 1, 1, 1, 1]
        
        # inverse calculation    
        goalconfig = self.ikine_LM(
            Tep=tf,
            q0=qz,
            mask=mask,
            joint_limits=True
        )
        
        # ikine calculation success handler
        if (not goalconfig.success):
            print("Problem with inverse kinematics")
            exit(0)
        print("Success in ikine")

        # plot robot and fkine
        self.plot(goalconfig.q, block=True)
        computed_tf = self.fkine(goalconfig.q)
        print(f"Computed TF:\n{computed_tf}")

        return goalconfig


    # plc function to send goalconfig
    def __send_config(self, config):
        '''
        Description:
            - The function send the computed goalconfig to the robot registers to execute motion.
            - For Fanuc Robot's few joints are interconnected as shown below (modified)
            - More about the modification can be seen in the project documentation page. 
        Args:
            - goalconfig : Array (ndarray of len 6)
        '''

        # modify the j2 and j3 angles (for fanuc robotic arm)
        config[1] = -config[1]
        config[2] = config[2] - config[1]

        # sol_q = np.round((goalconfig.q) * (180/ np.pi),2) + 125
        sol_q = np.degrees(config) + 360
        # print(sol_q)

        # passing jonit values to plc
        with LogixDriver(self.PLC_IP) as plc:
            if plc.connected:
                for i in range(6):
                    tag_name = f'Joint_{i+1}'
                    value = int(sol_q[i])
                    plc.write(tag_name, value)


    # control gripper (Servo gripper -> used firmata2 library)
    def on_gripper():
        pass
    def off_gripper():
        pass


    # read joint from Fanuc robot
    def __get_cur_joint_values(self):
        '''
        Description:
            - The function reads position registers from robot PLC and determines its current joint position
        Returns:
            - joint_values : ndarray of len 6 -> current joint values in the robot
        '''
        joint_values = [None, None, None, None, None, None]
        with LogixDriver(self.PLC_IP) as plc:
            for i in range(6):
                val = plc.read(f"FANUC:I.Data[{i}]")[1]             # FANUC:I.Data[0], 0, INT, None  --> from this take index [1]
                joint_values[i] = val
        return np.array(joint_values)


    # convert joint angles from fanuc to pose
    def get_curpos(self):
        '''
        Description:
            - The function reads the current position from above function and uses forward kinematics to compute current tf of ee.
            - Same modification is done to the cur_config (NOTE: Refer to project documentation)
        Returns:
            - [Rotm, trans] : Transform from base_link to end-effector
        '''

        # get current position
        cur_config = self.__get_cur_joint_values() - 360
        cur_config = np.radians(cur_config)                 # deg to rad
        cur_config[1] = -cur_config[1]
        cur_config[2] = cur_config[2] - cur_config[1]
        cur_tf = self.fkine(cur_config)
        return [cur_tf.R, cur_tf.t]                         # returns as rotm, trans (as np.array())


    # wait function
    def __wait_to_run(self, goal_config):
        '''
        Description:
            - The function read the goal_config and current config to wait for completion of movement
            - Tolerance of '1 deg' to '5 deg' is used.
        Args:
            - goal_config : ndarray (len == 6)
        '''

        # modify the j2 and j3 angles (for fanuc robotic arm)
        # goal_config[1] = -goal_config[1]
        # goal_config[2] = goal_config[2] - goal_config[1]
        # convert to degree
        goal_config = np.round(np.degrees(goal_config))
        # get current config
        cur_config = self.__get_cur_joint_values() - 360
        tol = 5            # tolerance
        while (True):       # wait until values become close
            cur_config = self.__get_cur_joint_values() - 360
            if (np.allclose(goal_config, cur_config, atol=tol)):
                break
            print("Goal: ", goal_config)
            print("Current: ", cur_config)
            time.sleep(0.5)
        print("Reached Goal Position")


    # move to function
    def move_to(self, trans, orient):
        '''
        Description:
            - The function takes goal translation and orientation of a point.
            - Calculates the joint angles using inverse kinematics and send the joint angle to the plc.
            - Waits until the joint angles reaches its target config 
        Args:
            - trans:  ndarray or list (size = 3 & meters)  -> (X, Y, Z)
            - orient: ndarray or list (size = 3 & radians) -> (roll, pitch, yaw)  
        '''

        # step 1: prepare the tf
        tf = sm.SE3(trans) * sm.SE3.RPY(orient, order="xyz")
        # step 2: calculate goal config
        goalconfig = self.__inverse_solver(tf)
        # step 3: send goalconfig to the plc
        self.__send_config(goalconfig.q)
        # step 4: wait until the goalconfig is reached
        self.__wait_to_run(goalconfig.q)


    # move to function
    def move_to(self, tf):
        '''
        Description:
            - The function takes goal translation and orientation of a point.
            - Calculates the joint angles using inverse kinematics and send the joint angle to the plc.
            - Waits until the joint angles reaches its target config 
        Args:
            - tf : transform of (4x4 matrix) with orientation + translation [base_link to end-effector]
        '''

        # step 1: calculate goal config
        goalconfig = self.__inverse_solver(tf)
        # step 2: send goalconfig to the plc
        self.__send_config(goalconfig.q)
        # step 3: wait until the goalconfig is reached
        self.__wait_to_run(goalconfig.q)


if __name__ == "__main__":

    robot = Fanuc()
    print(robot)
    robot.plot(robot.qz, block=True)

    '''
    # sample position
    tf = sm.SE3(0.35, 0.0, 0.05) * sm.SE3.RPY([0, np.pi, 0], order="xyz")
    print(tf)
    mask = [1, 1, 1, 1, 1, 1]
    # sample ikine
    sol = robot.ikine_LM(tf, q0=robot.qz, mask=mask, joint_limits=True)
    robot.plot(sol.q, block=True)
    # sample fkine
    print(robot.fkine(sol.q))
    '''