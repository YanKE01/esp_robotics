import cv2
import numpy as np

# === 1. 加载相机内参和畸变参数 ===
mtx = np.array([
    [914.3218251, 0., 265.36182684],
    [0, 918.18995797, 285.79522081],
    [0, 0, 1]
])
dist = np.array([[-4.90700601e-01, 1.10994649e+00, -8.90844060e-03, 1.18636540e-03, -2.97322477e+00]])

# === 2. 设置棋盘格信息（9x6，格子大小5mm）===
grid_x, grid_y = 8, 6
square_size = 0.008  # 5mm

objp = np.zeros((grid_x * grid_y, 3), np.float32)
objp[:, :2] = np.mgrid[0:grid_x, 0:grid_y].T.reshape(-1, 2) * square_size

# === 3. 角点检测的优化参数 ===
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# === 4. 定义坐标轴的3D点（用于绘制坐标系）===
axis = np.float32([[0.01, 0, 0], [0, 0.01, 0], [0, 0, -0.02]]).reshape(-1, 3)


def draw_axis(img, corners, imgpts):
    """
    在图像上绘制3D坐标轴
    """
    corner = tuple(map(int, corners[0].ravel()))
    imgpts = np.int32(imgpts).reshape(-1, 2)

    img = cv2.line(img, corner, tuple(imgpts[0]), (255, 0, 0), 5)  # X轴 - 蓝色
    img = cv2.line(img, corner, tuple(imgpts[1]), (0, 255, 0), 5)  # Y轴 - 绿色
    img = cv2.line(img, corner, tuple(imgpts[2]), (0, 0, 255), 5)  # Z轴 - 红色
    return img


cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, (grid_x, grid_y), None)

    if ret:
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        retval, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)
        imagePoints, jacobian = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
        frame = draw_axis(frame, corners2, imagePoints)

    cv2.imshow('Real-time Pose Estimation', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
