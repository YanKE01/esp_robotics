import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import *

DFbot = DHRobot(
    [
        RevoluteDH(d=0.04145, alpha=np.pi / 2, qlim=np.array([-np.pi / 2, np.pi / 2])),
        RevoluteDH(a=-0.08285, qlim=np.array([-np.pi / 2, np.pi / 2])),
        RevoluteDH(a=-0.08285, qlim=np.array([-np.pi / 2, np.pi / 2])),
        RevoluteDH(alpha=-np.pi / 2, qlim=np.array([0, np.pi])),
        RevoluteDH(d=0.11, qlim=np.array([-np.pi / 2, np.pi])),
    ],
    name="DFbot",
)

# Joint Trajectory Planning
state0 = [0, 0, 0, 0, 0]
state1 = [-np.pi / 6, -np.pi / 3, np.pi / 6, np.pi / 2, 0]
qt = rtb.tools.trajectory.jtraj(state0, state1, 20)
qt.plot(block=True)
DFbot.plot(qt.q, backend='pyplot', block=True)
print(qt.q)

# Cartesian Trajectory Planning
T0 = DFbot.fkine(state0)
T1 = DFbot.fkine(state1)
qt1 = rtb.tools.trajectory.ctraj(T0, T1, 20)
print(qt1)

pose_list = []
for q in qt1:
    sol = DFbot.ikine_LM(q, q0=state0, ilimit=100, mask=[1, 1, 1, 1, 1, 0], joint_limits=True)
    if sol.success:
        pose_list.append(sol.q)

for pose in pose_list:
    print(pose)
