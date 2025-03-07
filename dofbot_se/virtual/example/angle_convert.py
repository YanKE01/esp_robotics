"""
Mapping Between Simulation Model and Real-World Model in Testing
"""

import time

import numpy as np
from Arm_Lib import Arm_Device
from roboticstoolbox import *

DFbot = DHRobot(
    [
        RevoluteDH(d=0.04145, alpha=np.pi / 2, qlim=np.array([-np.pi / 2, np.pi / 2])),
        RevoluteDH(a=-0.08285, qlim=np.array([-np.pi, 0])),
        RevoluteDH(a=-0.08285, qlim=np.array([-np.pi / 2, np.pi / 2])),
        RevoluteDH(alpha=-np.pi / 2, qlim=np.array([0, np.pi])),
        RevoluteDH(d=0.11, qlim=np.array([-np.pi / 2, np.pi])),
    ],
    name="DFbot",
)


def convert_deg(deg):
    d1 = deg[0] + 90
    d2 = - deg[1]
    d3 = 90 - deg[2]
    d4 = 180 - deg[3]
    d5 = deg[4] + 90
    return [d1, d2, d3, d4, d5]


if __name__ == '__main__':
    Arm = Arm_Device(com="COM13")
    time.sleep(1)
    Arm.Arm_serial_set_torque(1)
    time.sleep(1)

    state0 = [0, 0, 0, 0, 0]
    DFbot.plot(np.deg2rad(state0), block=True)
    real_deg0 = convert_deg(state0)
    Arm.Arm_serial_servo_write6(real_deg0[0], real_deg0[1], real_deg0[2], real_deg0[3], real_deg0[4], 0, 1000)
    time.sleep(1)

    state1 = [0, -90, 0, 90, 0]
    DFbot.plot(np.deg2rad(state1), block=True)
    real_deg1 = convert_deg(state1)
    Arm.Arm_serial_servo_write6(real_deg1[0], real_deg1[1], real_deg1[2], real_deg1[3], real_deg1[4], 0, 1000)
    time.sleep(1)

    state2 = [0, -40, 30, 160, 0]
    DFbot.plot(np.deg2rad(state2), block=True)
    real_deg2 = convert_deg(state2)
    Arm.Arm_serial_servo_write6(real_deg2[0], real_deg2[1], real_deg2[2], real_deg2[3], real_deg2[4], 120, 1000)

    del Arm
