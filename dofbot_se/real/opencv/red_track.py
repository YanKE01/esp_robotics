import cv2
import numpy as np


def nothing(x):
    pass


def create_trackbars():
    cv2.namedWindow('Trackbars')
    cv2.createTrackbar('Low H', 'Trackbars', 0, 180, nothing)
    cv2.createTrackbar('High H', 'Trackbars', 180, 180, nothing)
    cv2.createTrackbar('Low S', 'Trackbars', 177, 255, nothing)
    cv2.createTrackbar('High S', 'Trackbars', 255, 255, nothing)
    cv2.createTrackbar('Low V', 'Trackbars', 139, 255, nothing)
    cv2.createTrackbar('High V', 'Trackbars', 255, 255, nothing)


def detect_red_objects():
    cap = cv2.VideoCapture(0)
    create_trackbars()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

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
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 2000:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)  # 蓝色圆点
                    cv2.putText(frame, f"({cx}, {cy})", (cx + 10, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)  # 显示坐标

                    print(f"Detected object at: ({cx}, {cy})")

        h, w, _ = frame.shape
        cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)  # 竖线
        cv2.line(frame, (0, h // 2), (w, h // 2), (255, 255, 255), 1)  # 横线

        cv2.imshow('Red Object Detection', frame)
        cv2.imshow('Mask', mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    detect_red_objects()
