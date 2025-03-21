import os
import cv2
import numpy as np


def compute_T(images_path, corner_point_long, corner_point_short, corner_point_size):
    print("标定板的中长度对应的角点的个数", corner_point_long)
    print("标定板的中宽度对应的角点的个数", corner_point_short)
    print("标定板一格的长度", corner_point_size)

    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)
    objp = np.zeros((corner_point_long * corner_point_short, 3), np.float32)
    objp[:, :2] = np.mgrid[0:corner_point_long, 0:corner_point_short].T.reshape(-1, 2)
    objp = corner_point_size * objp

    obj_points = []  # 3D点
    img_points = []  # 2D点

    image_files = sorted([f for f in os.listdir(images_path) if f.lower().endswith('.jpg')])  # 获取所有JPG文件

    for image_file in image_files:
        image_path = os.path.join(images_path, image_file)
        img = cv2.imread(image_path)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        ret, corners = cv2.findChessboardCorners(gray, (corner_point_long, corner_point_short), None)
        if ret:
            obj_points.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
            cv2.drawChessboardCorners(img, (corner_point_long, corner_point_short), corners, ret)
            cv2.imshow(f'Image {image_file}', img)
            cv2.putText(img, "Press ESC for next image", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
            cv2.imshow(f'Image {image_file}', img)
            key = cv2.waitKey(0)
            if [corners2]:
                img_points.append(corners2)
            else:
                img_points.append(corners)
        cv2.destroyAllWindows()

    if not img_points:
        print("未找到任何有效的标定图像。")
        return

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)
    print("内参矩阵:\n", mtx)
    print("畸变系数:\n", dist)
    print("-----------------------------------------------------")


if __name__ == '__main__':
    corner_point_long = 8
    corner_point_short = 6
    corner_point_size = 0.008

    compute_T("asset", corner_point_long, corner_point_short, corner_point_size)
