import matplotlib.pyplot as plt
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH

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

if __name__ == '__main__':
    N = 15000

    lim1_min = -np.pi / 2
    lim1_max = np.pi / 2

    lim2_min = -np.pi
    lim2_max = 0

    lim3_min = -np.pi / 2
    lim3_max = np.pi / 2

    lim4_min = 0
    lim4_max = np.pi

    lim5_min = -np.pi / 2
    lim5_max = np.pi

    lim1 = lim1_max - lim1_min
    lim2 = lim2_max - lim2_min
    lim3 = lim3_max - lim3_min
    lim4 = lim4_max - lim4_min
    lim5 = lim5_max - lim5_min

    theta1 = lim1_min + lim1 * np.random.rand(N)
    theta2 = lim2_min + lim2 * np.random.rand(N)
    theta3 = lim3_min + lim3 * np.random.rand(N)
    theta4 = lim4_min + lim4 * np.random.rand(N)
    theta5 = lim5_min + lim5 * np.random.rand(N)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for n in range(N):
        theta = np.array([theta1[n], theta2[n], theta3[n], theta4[n], theta5[n]])
        workspace = DFbot.fkine(theta)
        pos = workspace.t
        ax.plot([pos[0]], [pos[1]], [pos[2]], 'b.', markersize=1)

    plt.title("DFBOT Workspace")
    plt.show()
