# sift匹配红绿灯的视频流

'''
在一定时间范围内，sift匹配成功的次数达到一定值，则认为识别到红绿灯，且此后不再进行识别
'''


import cv2

# 读取模板图像
templateImage = cv2.imread("/home/scnu-car/car2023_ws/src/racecar/src/siftred/Redsmall.png")

# 创建SIFT检测器
sift = cv2.SIFT_create()

# 使用FLANN匹配器进行特征匹配
matcher = cv2.FlannBasedMatcher()

# 打开视频文件
# 读本地视频文件
video_capture = cv2.VideoCapture('/home/scnu-car/car2023_ws/src/racecar/src/parking_py/2023-11-07-163427.webm')  # 替换为您的视频文件路径
# video_capture = cv2.VideoCapture('/home/scnu-car/car2023_ws/src/racecar/src/siftred/RedandGreen1.webm')  # 替换为你的视频文件路径

# 读电脑的内置摄像头

# video_capture =cv2.VideoCapture(0)

while True:
    ret, frame = video_capture.read()

    if not ret:
        break
    
    # 截取图像的下半部分进行匹配
    height,width, _ =frame.shape
    frame = frame[1*height // 3 :,:]
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

    # ##################阈值可能需要根据环境修改
    if len(good_matches) >= 1:  # 自定义阈值
        print("匹配成功！------共找到", len(good_matches), "个匹配点。")
    else:
        print("匹配失败", len(good_matches))

    ################## 需要根据识别算法在车上的实际运行速度，确定停车地点后，可考虑是否需要延时适当时间后再停车（因为会较远处就识别到红绿灯）即延时发送话题消息或车接收到话题之后先继续行驶一段时间



    # 在图像上标记匹配的关键点
    image_with_matches = cv2.drawMatches(templateImage, keypoints1, frame, keypoints2, good_matches, None)

    # 显示图像
    cv2.imshow("Video Frame with Matches", image_with_matches)

    if cv2.waitKey(1) & 0xFF == 27:  # 按ESC键退出
        break

video_capture.release()
cv2.destroyAllWindows()
