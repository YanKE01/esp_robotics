import numpy as np
from roboticstoolbox import *
from spatialmath.base import *

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


state0 = [0, 0, 0, 0, 0]
T0 = DFbot.fkine(state0)
print("T0:{}".format(T0))

state1 = [-np.pi / 6, -np.pi / 3, np.pi / 6, np.pi / 2, 0]
T1 = DFbot.fkine(state1)
print("T1:{}".format(T1))

T2 = transl(-0.1805, 0.1042, 0.2096) @ rpy2tr(roll=np.deg2rad(0), pitch=np.deg2rad(-60), yaw=np.deg2rad(-30))
print("T2:{}".format(T2))

sol = DFbot.ikine_LM(T2, q0=state0, ilimit=100, mask=[1, 1, 1, 1, 1, 0], joint_limits=True)
print(sol)
