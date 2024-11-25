import cv2

video_capture = cv2.VideoCapture(0)

while True:
    ret, frame = video_capture.read()

    if not ret:
        break
    cv2.imshow("Video", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # 按ESC键退出
        break

video_capture.release()
cv2.destroyAllWindows()
