import cv2
import numpy as np


class mapBlur:

    def __init__(self) -> None:
        pass
    
    def mapBlur(self):
        # 读取图像并转换为灰度图像
        img = cv2.imread('originMap.pgm')
        # img_cut = img[325:390 + 1, 435:720 + 1]  #outer
        # img_cut = img[280:430 + 1, 390:760 + 1] #inner

        # 获取灰度图像
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # 获取二值化图像
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

        # 查找轮廓
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blk_contour =[] #保留的黑色区域，即障碍物
        filtered =[]    #需要被滤除的区域
        # 迭代所有的轮廓，计算其面积
        for contour in contours:
            area = cv2.contourArea(contour)   #计算黑点区域面积
            if area >= 0.30:                     #面积>=0.3的保留
                blk_contour.append(contour)
            else:
                filtered.append(contour)
            # print("黑点面积:", area)

        # 绘制黑点区域的轮廓
        blured_img = np.copy(img)
        # print(blk_contour)

        #用白色填充
        cv2.drawContours(blured_img, filtered, -1, (255, 255, 255), cv2.FILLED)  

        #标志被保留的部分，即增强
        cv2.drawContours(blured_img, blk_contour, -1, (0, 0, 0), 3)  

        #保存图片，在实车上跑时使用
        # saved_img = cv2.cvtColor(blured_img, cv2.COLOR_BGR2GRAY)
        # cv2.imwrite('map/mymap.pgm', saved_img)
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # cv2.imwrite('map/originMap.pgm', img)
        # print('Blured Map Finshed!')


        # 显示图像
        result = cv2.hconcat([img, blured_img])
        #result = cv2.resize(result, (result.shape[1]//2, result.shape[0]//2))
        # cv2.imshow('originMap', img)
        # cv2.imshow('bluredMap', blured_img)
        cv2.imshow('result', result)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    mapBlur().mapBlur()
