import time

import numpy as np
from Arm_Lib import Arm_Device
from roboticstoolbox import *


def convert_sim_to_real(sim_deg):
    d1 = sim_deg[0] + 90
    d2 = -sim_deg[1]
    d3 = 90 - sim_deg[2]
    d4 = 180 - sim_deg[3]
    d5 = sim_deg[4] + 90
    return [d1, d2, d3, d4, d5]


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

Arm = Arm_Device(com="COM10")
time.sleep(1)

state0 = [0, -2 * np.pi / 3, np.pi / 2, np.pi, 0]
real_angle = convert_sim_to_real(np.rad2deg(state0))
Arm.Arm_serial_servo_write6(real_angle[0], real_angle[1], real_angle[2], real_angle[3], real_angle[4], 0, 1000)

T0 = DFbot.fkine(state0)
print("T0:", T0)

# horizontal movement
for num in np.arange(-0.13, 0.13, 0.02):
    T1 = np.array(T0)
    T1[1, -1] = num
    print("Y:{} T1:{}".format(num, T1))
    sol = DFbot.ikine_LM(T1, q0=state0, ilimit=10000, slimit=20000, mask=[1, 1, 1, 1, 1, 0], joint_limits=False)
    print(sol)
    print("\n")
    if sol.success:
        real_angle = convert_sim_to_real(np.rad2deg(sol.q))
        Arm.Arm_serial_servo_write6(real_angle[0], real_angle[1], real_angle[2], real_angle[3], real_angle[4], 0, 1000)
    time.sleep(1)

# forward motion
state0 = [0, -2 * np.pi / 3, np.pi / 2, np.pi, 0]
T0 = DFbot.fkine(state0)
real_angle = convert_sim_to_real(np.rad2deg(state0))
Arm.Arm_serial_servo_write6(real_angle[0], real_angle[1], real_angle[2], real_angle[3], real_angle[4], 0, 1000)

for num in np.arange(0.09, 0.18, 0.02):
    T1 = np.array(T0)
    T1[0, -1] = -num
    print("X:{} T1:{}".format(num, T1))
    sol = DFbot.ikine_LM(T1, q0=state0, ilimit=10000, slimit=20000, mask=[1, 1, 1, 1, 1, 0], joint_limits=False)
    print(sol)
    print("\n")
    if sol.success:
        real_angle = convert_sim_to_real(np.rad2deg(sol.q))
        Arm.Arm_serial_servo_write6(real_angle[0], real_angle[1], real_angle[2], real_angle[3], real_angle[4], 0, 1000)
    time.sleep(1)

# move upwards
state0 = [0, -1 * np.pi / 2, np.pi / 2, np.pi / 2, 0]
T0 = DFbot.fkine(state0)
real_angle = convert_sim_to_real(np.rad2deg(state0))
Arm.Arm_serial_servo_write6(real_angle[0], real_angle[1], real_angle[2], real_angle[3], real_angle[4], 0, 1000)

for num in np.arange(0.02, 0.17, 0.01):
    T1 = np.array(T0)
    T1[2, -1] = num
    print("Z:{} T1:{}".format(num, T1))
    sol = DFbot.ikine_QP(T1, q0=state0, ilimit=10000, slimit=20000, mask=[1, 1, 1, 1, 1, 0], joint_limits=False)
    print(sol)
    print("\n")
    if sol.success:
        real_angle = convert_sim_to_real(np.rad2deg(sol.q))
        Arm.Arm_serial_servo_write6(real_angle[0], real_angle[1], real_angle[2], real_angle[3], real_angle[4], 0, 1000)
