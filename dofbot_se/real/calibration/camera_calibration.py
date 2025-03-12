import os

import cv2
import numpy as np


def compute_T(images_path, corner_point_long, corner_point_short, corner_point_size):
    print("标定板的中长度对应的角点的个数", corner_point_long)
    print("标定板的中宽度对应的角点的个数", corner_point_short)
    print("标定板一格的长度", corner_point_size)

    # 设置寻找亚像素角点的参数，采用的停止准则是最大循环次数30和最大误差容限0.001
    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)
    # 获取标定板角点的位置
    objp = np.zeros((corner_point_long * corner_point_short, 3), np.float32)

    objp[:, :2] = np.mgrid[0:corner_point_long, 0:corner_point_short].T.reshape(-1, 2)
    # 将世界坐标系建在标定板上，所有点的Z坐标全部为0，所以只需要赋值x和y
    objp = corner_point_size * objp

    obj_points = []  # 存储3D点
    img_points = []  # 存储2D点

    for i in range(0, 30):  # 标定好的图片在images_path路径下，从0.jpg到x.jpg   一次采集的图片最多不超过30张，遍历从0.jpg到30.jpg ，选择能够读取的到的图片
        image = f"{images_path}/{i}.jpg"  # ubuntu下
        if os.path.exists(image):
            img = cv2.imread(image)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            size = gray.shape[::-1]
            ret, corners = cv2.findChessboardCorners(gray, (corner_point_long, corner_point_short), None)
            if ret:
                obj_points.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)  # 在原角点的基础上寻找亚像素角点

                # 绘制检测到的角点
                cv2.drawChessboardCorners(img, (corner_point_long, corner_point_short), corners, ret)
                # 显示图片和提示信息
                cv2.imshow(f'Image {i}', img)
                cv2.putText(img, "Press ESC for next image", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
                cv2.imshow(f'Image {i}', img)

                # 等待按键事件
                key = cv2.waitKey(0)
                if [corners2]:
                    img_points.append(corners2)
                else:
                    img_points.append(corners)
        cv2.destroyAllWindows()
    N = len(img_points)
    # 标定,得到图案在相机坐标系下的位姿
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)
    print("内参矩阵:\n", mtx)  # 内参数矩阵
    print("畸变系数:\n", dist)  # 畸变系数   distortion cofficients = (k_1,k_2,p_1,p_2,k_3)
    print("-----------------------------------------------------")


if __name__ == '__main__':
    # 初始姿态: Arm.Arm_serial_servo_write6(100, 100, 10, 0, 90, 0, 1000)
    corner_point_long = 9  # 标定板角点数量  长边
    corner_point_short = 6
    corner_point_size = 0.005  # 标定板方格真实尺寸  m

    compute_T("asset", corner_point_long, corner_point_short, corner_point_size)
