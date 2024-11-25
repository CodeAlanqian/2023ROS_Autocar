# sift匹配停车的视频流
# 匹配模板为相机捕捉到的停车标志
# 停车的第二版代码 由于使用sift且模板较大 消耗资源 造成丢帧严重 但仍然可以实现识别功能

import cv2

# 读取模板图像
# templateImage = cv2.imread("/home/scnu-car/car2023_ws/src/racecar/src/siftparking/parktemplatefix.jpg")#车实际看到的停车标志进行匹配
templateImage = cv2.imread("/home/scnu-car/car2023_ws/src/racecar/src/siftparking/resize_parking.jpg")# 将图像缩小后的标志   似乎这个效果更好

# 创建SIFT检测器
sift = cv2.SIFT_create()

# 使用FLANN匹配器进行特征匹配
matcher = cv2.FlannBasedMatcher()

# 打开视频文件
# video_capture = cv2.VideoCapture('/home/scnu-car/car2023_ws/src/racecar/src/siftparking/parking_test.webm')  # 替换为你的视频文件路径
video_capture = cv2.VideoCapture('/home/scnu-car/car2023_ws/src/racecar/src/parking_py/2023-11-07-163427.webm')  # 替换为您的视频文件路径

# video_capture = cv2.VideoCapture(0)  # 替换为你的视频文件路径

while True:
    ret, frame = video_capture.read()
    if not ret:
        break

    # templateImage=cv2.normalize(templateImage,None,0,255,cv2.NORM_MINMAX).astype('uint8')
    # 检测关键点和生成描述符
    keypoints1, descriptors1 = sift.detectAndCompute(templateImage, None)
    keypoints2, descriptors2 = sift.detectAndCompute(frame, None)

    # 进行比率测试，筛选好的匹配
    matches = matcher.knnMatch(descriptors1, descriptors2, k=2)
    good_matches = []
    for m, n in matches:
        if m.distance < 0.75 * n.distance:
            good_matches.append(m)

    # 输出匹配成功的信息
    if len(good_matches) >= 40:  # 自定义阈值
        # print("匹配成功！共找到", len(good_matches), "个匹配点。")
        cv2.putText(frame, str(len(good_matches)), (5,50),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.75, (0,0,255),2)
    else:
        print("匹配失败。")

    # 在图像上标记匹配的关键点
    # image_with_matches = cv2.drawMatches(templateImage, keypoints1, frame, keypoints2, good_matches, None)

    # 显示图像
    # cv2.imshow("Video Frame with Matches", image_with_matches)
    cv2.imshow("Video Frame with Matches", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # 按ESC键退出
        break

video_capture.release()
cv2.destroyAllWindows()
