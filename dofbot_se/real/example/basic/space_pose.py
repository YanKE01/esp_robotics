import numpy as np
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

state0 = [0, -2 * np.pi / 3, np.pi / 2, np.pi, 0]
T0 = DFbot.fkine(state0)
print("T0:", T0)

T1 = np.array(T0)
T1[0, -1] = -0.15
T1[1, -1] = -0.07
print("T1:", T1)

sol = DFbot.ikine_LM(T1, q0=state0, ilimit=5000, slimit=5000, mask=[1, 1, 1, 1, 1, 0], joint_limits=True)
print(sol)

qt = jtraj([0, -2 * np.pi / 3, np.pi / 2, np.pi, 0], sol.q, 10)
DFbot.plot(qt.q, backend='pyplot', block=True)
