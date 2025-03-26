import time

import numpy as np
from Arm_Lib import Arm_Device
from roboticstoolbox import DHRobot, RevoluteDH


def convert_real_to_sim(real_deg):
    s1 = real_deg[0] - 90
    s2 = -real_deg[1]
    s3 = 90 - real_deg[2]
    s4 = 180 - real_deg[3]
    s5 = real_deg[4] - 90
    return [s1, s2, s3, s4, s5]


Arm = Arm_Device(com="COM10")
time.sleep(1)
Arm.Arm_serial_set_torque(0)
time.sleep(1)

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

real_angle_list = []

while True:
    real_angle_list.clear()
    for i in range(1, 6):
        real_angle_list.append(Arm.Arm_serial_servo_read(i))
        time.sleep(0.1)

    if None in real_angle_list:
        continue

    sim_angle = convert_real_to_sim(real_angle_list)
    sim_rad = np.deg2rad(np.array(sim_angle))
    print(sim_angle)
    T = DFbot.fkine(sim_rad)
    position = T.t
    rpy = T.rpy()
    print("Position (xyz):", position)
    print("Orientation (RPY):", rpy)
    print("\n")
