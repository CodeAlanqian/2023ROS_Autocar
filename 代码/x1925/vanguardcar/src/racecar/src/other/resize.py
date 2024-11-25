import cv2

image = cv2.imread("/home/scnu-car/car2023_ws/src/racecar/src/siftparking/parktemplatefix.jpg")

if image is not None:
    resized_image=cv2.resize(image,(image.shape[1]//2,image.shape[0]//2))

    cv2.imwrite("resize_parking.jpg",resized_image)

    cv2.destroyAllWindows()
else:
    print("无法读取到图片")