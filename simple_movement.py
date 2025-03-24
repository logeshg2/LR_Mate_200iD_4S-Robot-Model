import roboticstoolbox as rtb
import spatialmath as sm
import pycomm3
import time
import numpy as np
from pycomm3 import LogixDriver
from fanuc_model import Fanuc


# plc function
def send_config(config):
    # modify the j2 and j3 angles (for fanuc robotic arm)
    config[1] = -config[1]
    config[2] = config[2] - config[1]

    # sol_q = np.round((goalconfig.q) * (180/ np.pi),2) + 125
    sol_q = np.degrees(config) + 360
    # print(sol_q)

    # passing jonit values to plc
    with LogixDriver(PLC_IP) as plc:
        if plc.connected:
            for i in range(6):
                tag_name = f'Joint_{i+1}'
                value = int(sol_q[i])
                plc.write(tag_name, value)


# inverese kinematics function
def inverse_solver(tf):
    # the function takes SE3 (transform - 4x4) to compute ikine
    print(f"Original TF:\n{tf}")
    qz = robot.qz
    mask = [1, 1, 1, 1, 1, 1]
    
    # inverse calculation    
    goalconfig = robot.ikine_LM(
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
    # robot.plot(goalconfig.q, block=True)
    computed_tf = robot.fkine(goalconfig.q)
    print(f"Computed TF:\n{computed_tf}")

    return goalconfig


# read joint from Fanuc robot
def get_cur_joint_values():
    joint_values = [None, None, None, None, None, None]
    with LogixDriver(PLC_IP) as plc:
        for i in range(6):
            val = plc.read(f"FANUC:I.Data[{i}]")[1]             # FANUC:I.Data[0], 0, INT, None  --> from this take index [1]
            joint_values[i] = val
    return np.array(joint_values)


# wait function
def wait_for_run(goal_config):
    # modify the j2 and j3 angles (for fanuc robotic arm)
    # goal_config[1] = -goal_config[1]
    # goal_config[2] = goal_config[2] - goal_config[1]
    # convert to degree
    goal_config = np.round(np.degrees(goal_config))
    # get current config
    cur_config = get_cur_joint_values() - 360
    tol = 5            # tolerance
    while (True):       # wait until values become close
        cur_config = get_cur_joint_values() - 360
        if (np.allclose(goal_config, cur_config, atol=tol)):
            break
        print("Goal: ", goal_config)
        print("Current: ", cur_config)
        time.sleep(0.5)
    print("Reached Goal Position")


# main 
PLC_IP = '192.168.1.7'      # ASRS PLC IP (Laptop -> ASRS PLC -> Fanuc PLC)

robot = Fanuc()

home_tf = sm.SE3(0.36, 0.0, 0.28)
home_mask = [0.25, 0.25, 0.25, 0.0, 0.0, 0.0]
tf1 = sm.SE3(0.35, 0.0, 0.05) * sm.SE3.RPY([0, np.pi, 0], order="xyz")
tf2 = sm.SE3(0.3, -0.2, 0.1) * sm.SE3.RPY([0, np.pi, 0], order="xyz")
tf3 = sm.SE3(0.2, 0.3, 0.05) * sm.SE3.RPY([0, np.pi, 0], order="xyz")
tf4 = sm.SE3(0.3, -0.3, 0.3) * sm.SE3.RPY([0, np.pi/2, 0], order="xyz")
tfs = [tf1, tf2, tf3, tf4]

# simple movements 

# for tf in tfs:
#     sol_config = inverse_solver(tf)
#     send_config(sol_config.q)
#     wait_for_run(sol_config.q)                          # waits until robot reaches the sol_config

# trajectory planning
'''
q1 = robot.qz
traj =  rtb.ctraj(robot.fkine(q1), tf2, 3)
# print(traj)
for t in traj:
    conf = inverse_solver(t)
    send_config(conf.q)
    wait_for_run(conf.q)
'''
q1 = robot.qz
traj = rtb.jtraj(q1, inverse_solver(tf2).q, 3)
# print(traj)
for t in traj.q:
    send_config(t)
    wait_for_run(t)


'''
inc = -0.30
for i in range(-10,10,1):
    tf = sm.SE3(0.35, inc, 0.05) * sm.SE3.RPY([0, np.pi, 0], order="xyz")
    config = inverse_solver(tf)
    send_config(config.q)
    wait_for_run(config.q)
    inc += 0.025        
'''

# go to home
# send_config(robot.qz)
# wait_for_run(robot.qz)