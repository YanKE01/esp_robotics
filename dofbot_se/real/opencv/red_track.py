"""
按键说明：
q 退出
a AI识别
g 移动到AI识别点或鼠标单机选点
h 复位机械臂
j 移动到OpenCV标记点
k 抓住
l 松开（每次运动时也会松开）
"""

import base64
import json
from math import sqrt
import os
import time
import cv2
import numpy as np
import iv_4dof as iv_4dof
from Arm_Lib import Arm_Device
from openai import OpenAI

K = np.array([
    [914.3218251, 0., 265.36182684],
    [0, 918.18995797, 285.79522081],
    [0, 0, 1]
])
dist = np.array([[-4.90700601e-01, 1.10994649e+00, -8.90844060e-03, 1.18636540e-03, -2.97322477e+00]])
K_inv = np.linalg.inv(K)  # 计算内参矩阵的逆矩阵
real_width_cm = 0.03  # 实际宽度3cm

M = np.array([[-6.30929943e-03, -6.02338057e-02,  1.47461503e+01],
 [-6.53951889e-02,  5.59609594e-03,  3.30212195e+01],
 [-8.71935853e-03,  7.46146126e-04,  4.06949593e+00]])
relax_degs = [60, 60, 60, 60]

Arm = Arm_Device("COM6")


client = OpenAI(
    # 若没有配置环境变量，请用百炼API Key将下行替换为：api_key="sk-xxx",
    api_key=os.getenv("DASHSCOPE_API_KEY"),
    base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
)


def uv_to_xyz(center_x, center_y, K, real_width_cm, detected_width_pix):
    """
    根据像素坐标和相机内参计算三维坐标（单位：厘米）
    :param center_x: 矩形中心点的像素坐标x
    :param center_y: 矩形中心点的像素坐标y
    :param K: 相机内参矩阵
    :param real_width_cm: 物体实际宽度（3cm）
    :param detected_width_pix: 图像中检测到的物体宽度（像素）
    :return: 三维坐标 (X, Y, Z)（单位：厘米）
    """
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]
    
    # 计算深度（单位：厘米）
    d = (real_width_cm * sqrt(fx * fy)) / detected_width_pix  # 假设 detected_width_pix 是物体宽度的像素值
    
    # 将像素坐标转换为归一化坐标
    X = (center_x - cx) * d / fx
    Y = (center_y - cy) * d / fy
    Z = d  # 深度（单位：厘米）
    
    return (X, Y, Z)

def nothing(x):
    pass


def create_trackbars():
    cv2.namedWindow('Trackbars')
    cv2.createTrackbar('Low H', 'Trackbars', 0, 180, nothing)
    cv2.createTrackbar('High H', 'Trackbars', 40, 180, nothing)
    cv2.createTrackbar('Low S', 'Trackbars', 121, 255, nothing)
    cv2.createTrackbar('High S', 'Trackbars', 255, 255, nothing)
    cv2.createTrackbar('Low V', 'Trackbars', 139, 255, nothing)
    cv2.createTrackbar('High V', 'Trackbars', 255, 255, nothing)

def to_base64url(image):
    success, encoded_image = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    if not success:
        print("图像编码失败")
        return
    # 转换为Base64
    base64_str = base64.b64encode(encoded_image).decode('utf-8')
    # 生成Data URL
    return f"data:image/jpeg;base64,{base64_str}"


x_pos = y_pos = 0
def on_mouse_click(event, x, y, flags, param):
    global clicked, x_pos, y_pos
    if event == cv2.EVENT_LBUTTONDOWN:  # 检测左键点击
        x_pos, y_pos = x, y

bbox = None

def detect_red_objects():
    cap = cv2.VideoCapture(0)
    create_trackbars()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.undistort(frame, K, dist)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 获取 trackbar 值
        l_h = cv2.getTrackbarPos('Low H', 'Trackbars')
        h_h = cv2.getTrackbarPos('High H', 'Trackbars')
        l_s = cv2.getTrackbarPos('Low S', 'Trackbars')
        h_s = cv2.getTrackbarPos('High S', 'Trackbars')
        l_v = cv2.getTrackbarPos('Low V', 'Trackbars')
        h_v = cv2.getTrackbarPos('High V', 'Trackbars')

        lower_red = np.array([l_h, l_s, l_v])
        upper_red = np.array([h_h, h_s, h_v])

        mask = cv2.inRange(hsv, lower_red, upper_red)

        kernel = np.ones((5, 5), np.uint8)
        for _ in range(5):
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 2000:
                # 获取最小外接矩形参数（可能旋转）
                rect = cv2.minAreaRect(contour)
                ((center_x, center_y), (width_pix, height_pix), angle) = rect
                
                # 计算实际尺寸对应的像素宽度（取较大边）
                detected_width_pix = max(width_pix, height_pix)
                
                # 计算三维坐标
                xyz = uv_to_xyz(center_x, center_y, K, real_width_cm, detected_width_pix)
                
                # 绘制旋转矩形
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
                
                # 绘制中心点和坐标
                cv2.circle(frame, (int(center_x), int(center_y)), 5, (255, 0, 0), -1)
                
                cv2.putText(frame, f"({int(center_x)}, {int(center_y)})", 
                        (int(center_x)+10, int(center_y)-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                
                # 显示三维坐标（单位：厘米）
                # cv2.putText(frame, f"({xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f})m", 
                #         (int(center_x)+10, int(center_y)-30),
                #         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                # 机械臂坐标
                xyz = M @ np.array([center_x, center_y, 1])
                cv2.putText(frame, f"({xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f})cm", 
                        (int(center_x)+10, int(center_y)-30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                
                # print(f"Detected object at: X={xyz[0]:.1f}cm, Y={xyz[1]:.1f}cm, Z={xyz[2]:.1f}cm")
        h, w, _ = frame.shape
        cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)  # 竖线
        cv2.line(frame, (0, h // 2), (w, h // 2), (255, 255, 255), 1)  # 横线

        global bbox, x_pos, y_pos
        cv2.circle(frame, (int(x_pos), int(y_pos)), 5, (255, 255, 0), -1)
        if bbox:
            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255, 255, 0), 1)
        cv2.imshow('Red Object Detection', frame)
        cv2.setMouseCallback('Red Object Detection', on_mouse_click)
        cv2.imshow('Mask', mask)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break
        elif key & 0xff == ord('a'):
            url = to_base64url(frame)
            completion = client.chat.completions.create(
                model="qwen2.5-vl-32b-instruct",
                messages=[{"role": "user","content": [
                        {"type": "text","text": 'Detect the object in the image and return its locations in the form of coordinates. The format of output should be like {"bbox_2d": [x1, y1, x2, y2]}'},
                        {"type": "image_url",
                        "image_url": {"url": url}}
                        ]}],
                response_format={"type": "json_object"},
            )
            resp = completion.choices[0].message.content
            try:
                data = json.loads(resp)
                bbox = data["bbox_2d"]
                print(bbox)
                x_pos = (bbox[0] + bbox[2]) / 2
                y_pos = (bbox[1] + bbox[3]) / 2
            except Exception:
                print("JSON格式错误")
        elif key & 0xff == ord('g'):
            xyz = M @ np.array([x_pos, y_pos, 1])
            # xyz[2] -= 2
            valid, *degs =iv_4dof.backward_kinematics(*xyz, 180)
            if valid:
                Arm.Arm_serial_servo_write(6, 90,  100)
                time.sleep(0.1)
                for i, d in enumerate(degs):
                    Arm.Arm_serial_servo_write(i + 1, d,  1000)
                    time.sleep(0.1)
        elif key & 0xff == ord('h'):
            Arm.Arm_serial_servo_write(6, 90,  100)
            time.sleep(0.1)
            for i, d in enumerate(relax_degs):
                Arm.Arm_serial_servo_write(i + 1, d,  1000)
                time.sleep(0.1)
        elif key & 0xff == ord('j'):
            valid, *degs =iv_4dof.backward_kinematics(*xyz, 180)
            if valid:
                Arm.Arm_serial_servo_write(6, 90,  100)
                time.sleep(0.1)
                for i, d in enumerate(degs):
                    Arm.Arm_serial_servo_write(i + 1, d,  1000)
                    time.sleep(0.1)
        elif key & 0xff == ord('k'):
            Arm.Arm_serial_servo_write(6, 130,  1000)
        elif key & 0xff == ord('l'):
            Arm.Arm_serial_servo_write(6, 90,  1000)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    detect_red_objects()
