# 停车的第一版代码 预处理后根据大小筛选出停车标志
# 速度较快 优先使用

import cv2
import numpy as np
import time

# 打开视频文件
# cap = cv2.VideoCapture('/home/scnu-car/car2023_ws/src/racecar/src/parking_py/parking_test.webm')  # 替换为您的视频文件路径
cap = cv2.VideoCapture('/home/scnu-car/car2023_ws/src/racecar/src/parking_py/2023-11-07-163427.webm')  # 替换为您的视频文件路径
# cap = cv2.VideoCapture(0)  # 替换为您的视频文件路径

if not cap.isOpened():
    print("Error: Could not open video file.")
    exit()


while True:
    ret, frame = cap.read()
    if not ret:
        break

    img = frame.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 蓝色区域的阈值设置
    blue_lower = np.array([100, 100, 100], dtype=np.uint8)
    blue_upper = np.array([124, 255, 255], dtype=np.uint8)
    mask_blue = cv2.inRange(hsv, blue_lower, blue_upper)

    blurred = cv2.blur(mask_blue, (9, 9))

    _, binary = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)

    # 使区域闭合无空隙
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
    closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    # 腐蚀和膨胀
    erode = cv2.erode(closed, None, iterations=4)
    dilate = cv2.dilate(erode, None, iterations=4)

    # 查找轮廓
    contours, _ = cv2.findContours(dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    res = img.copy()
    result_image = np.zeros(frame.shape, dtype=np.uint8)

    # 遍历轮廓
    for con in contours:
        rect = cv2.minAreaRect(con)
        box = cv2.boxPoints(rect).astype(int)

        h1 = max(box[0, 1], box[1, 1], box[2, 1], box[3, 1])
        h2 = min(box[0, 1], box[1, 1], box[2, 1], box[3, 1])
        l1 = max(box[0, 0], box[1, 0], box[2, 0], box[3, 0])
        l2 = min(box[0, 0], box[1, 0], box[2, 0], box[3, 0])

        h = h1 - h2
        l = l1 - l2

        # 筛选出高大于等于150像素点，宽大于等于300像素点
        if h >= 150 and l >= 300:
            for i in range(4):
                cv2.line(res, tuple(box[i]), tuple(box[(i + 1) % 4]), (0, 0, 255), 2)
                print("识别到停车标志！")
        else:
            print("识别中....未识别到...")
            #pass

    # 显示识别结果
    cv2.imshow("res", res)
    # cv2.imshow("result_image", result_image)

    # 按q退出
    if cv2.waitKey(10) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
