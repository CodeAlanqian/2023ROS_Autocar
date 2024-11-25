import cv2
import numpy as np
import time


#不开方，减少运算量，计算两点之间的距离的平方。
def distance_2(p1, p2):
    return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2

#计算斜率
def tan_2(p1, p2):
    if p1[0] - p2[0] != 0:
        return (p1[1] - p2[1])/(p1[0] - p2[0])
    else:  
        return 0


class mapBlur:
    def __init__(self) -> None:
        pass

    def mapBlur(self):

        #计算处理耗时
        start_time = time.perf_counter()

        # 读取图像并转换为灰度图像
        img = cv2.imread('originMap.pgm')
        #外圈轮廓
        img_cut = img[280:430 + 1, 390:760 + 1] 
        
        # 灰度图
        gray = cv2.cvtColor(img_cut, cv2.COLOR_BGR2GRAY)
        # 进行二值化处理
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

        # 查找轮廓
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #坐标转换
        for i in range(len(contours)):
            for j in range(len(contours[i])):
                contours[i][j][0][0] += 390
                contours[i][j][0][1] += 280


        blk_contour =[] #保留的黑色区域，即障碍物
        filtered =[]    #需要被滤除的区域
        for contour in contours:
            area = cv2.contourArea(contour)   #计算黑点区域面积
            if area >= 0.3:                     #面积>=0.3的保留
                blk_contour.append(contour)
            else:
                filtered.append(contour)
            # print("黑点面积:", area)

        # 绘制黑点区域的轮廓
        blured_img = np.copy(img)
        # print(blk_contour)

        #计算黑点区域的中心点
        points = []
        for i in blk_contour:
            points.append(i[int(i.shape[0]/2)][0])
        # print(points)

        line_points=[]
    
        cir_center = [
            (467,357),  #第一个圆心
            (582,357),  #第二个圆心
            (692,357)   #第三个圆心
        ]

        # cv2.circle(blured_img, (467,357), 3, (0, 0, 255), -1) #第一个圆心
        # cv2.circle(blured_img, (582,357), 3, (0, 0, 255), -1) #第二个圆心
        # cv2.circle(blured_img, (692,357), 3, (0, 0, 255), -1) #第三个圆心
        # cv2.circle(blured_img, (415,357), 3, (0, 0, 255), -1)
        # cv2.circle(blured_img, (442,357), 3, (0, 0, 255), -1)
        #大同心圆半径 = 476-415=61 
        #小同心圆半径 = 476-442=34

        cir_points_big = []  #大同心圆上的点
        for i in range(len(points)):
            #第1个圆的筛选条件：


            if distance_2(points[i], cir_center[0]) < 62**2 and distance_2(points[i], cir_center[0]) > 40**2:
                if tan_2(points[i], cir_center[0]) > 0.8 or tan_2(points[i], cir_center[0]) < -0.8 or cir_center[0][0] - points[i][0] >0:
                    # print(tan_2(points[i], cir_center[0]))
                    # cv2.circle(blured_img, points[i], 3, (0, 0, 255), -1)
                    cir_points_big.append(points[i])
                    

            #第2个圆的筛选条件：
            if distance_2(points[i], cir_center[1]) < 62**2 and distance_2(points[i], cir_center[1]) > 40**2:
                if tan_2(points[i], cir_center[1]) > 1.1 or tan_2(points[i], cir_center[1]) < -1.1:
                    # print(tan_2(points[i], cir_center[1]))
                    # cv2.circle(blured_img, points[i], 3, (255, 255, 0), -1)
                    cir_points_big.append(points[i])

            #第3个圆的筛选条件：
            if distance_2(points[i], cir_center[2]) < 62**2 and distance_2(points[i], cir_center[2]) > 40**2:
                if tan_2(points[i], cir_center[2]) > 0.8 or tan_2(points[i], cir_center[2]) < -0.8 or cir_center[2][0] - points[i][0] <0:
                    # print(tan_2(points[i], cir_center[2]))
                    # cv2.circle(blured_img, points[i], 3, (0, 255, 0), -1)
                    cir_points_big.append(points[i])


        # 大圆匹配
        for i in range(len(cir_points_big)):
            for j in range(i+1, len(cir_points_big)): #减少重复计算
                if i != j:
                    dis = distance_2(cir_points_big[i], cir_points_big[j])
                    if  dis < 34**2:
                        line_points.append([cir_points_big[i], cir_points_big[j]])

        # # #大圆连线
        # for n in range(len(line_points)):
        #     cv2.line(blured_img, line_points[n][0], line_points[n][1], (0, 0, 255), 2)


        cir_points_small = []  #小同心圆上的点
        for i in range(len(points)):
            #第1个圆的筛选条件：
            if distance_2(points[i], cir_center[0]) < 37**2:
                
                # cv2.circle(blured_img, points[i], 3, (0, 0, 255), -1)
                cir_points_small.append(points[i])
            
            #第2个圆的筛选条件：
            if distance_2(points[i], cir_center[1]) < 37**2:
                # if tan_2(points[i], cir_center[1]) > 1.1 or tan_2(points[i], cir_center[1]) < -1.1:
                #     # print(tan_2(points[i], cir_center[1]))
                # cv2.circle(blured_img, points[i], 3, (255, 255, 0), -1)
                cir_points_small.append(points[i])

            #第3个圆的筛选条件：
            if distance_2(points[i], cir_center[2]) < 37**2:
            
                # cv2.circle(blured_img, points[i], 3, (0, 255, 0), -1)
                cir_points_small.append(points[i])

        # 小圆匹配
        for i in range(len(cir_points_small)):
            for j in range(i+1, len(cir_points_small)): #减少重复计算
                if i != j:
                    dis = distance_2(cir_points_small[i], cir_points_small[j])
                    if  dis < 25**2:
                        line_points.append([cir_points_small[i], cir_points_small[j]])

        # # #小圆连线
        # for n in range(len(line_points)):
        #     cv2.line(blured_img, line_points[n][0], line_points[n][1], (0, 0, 255), 2)


        # time = [] * len(points) 
        # print(time)
        # #计算同一个点是否被使用了两次
        # for i in range(len(points)):
        #     if line_points[i][0] == points[i]:
        #     time[i] += 1
        #     time[line_points[i][1]] += 1


        # line_points.clear()
        # 按距离筛选
        for i in range(len(points)):
            for j in range(i+1, len(points)): #减少重复计算
                if i != j:
                    dis = distance_2(points[i], points[j])
                    if  dis < 24.5**2:
                        line_points.append([points[i], points[j]])



        #连线
        for n in range(len(line_points)):
            cv2.line(blured_img, line_points[n][0], line_points[n][1], (0, 0, 0), 1)


        #用白色填充，即滤除
        cv2.drawContours(blured_img, filtered, -1, (255, 255, 255), cv2.FILLED)  


        #标志被保留的部分，即增强
        cv2.drawContours(blured_img, blk_contour, -1, (0, 0, 0), 3)  



        # cv2.drawContours(blured_img, points, -1, (0, 0, 255), 2)
        #保存图片
        # saved_img = cv2.cvtColor(blured_img, cv2.COLOR_BGR2GRAY)
        # # cv2.imwrite('bluredMap.pgm', saved_img)

        # saved_img = cv2.cvtColor(blured_img, cv2.COLOR_BGR2GRAY)
        # cv2.imwrite('map/mymap.pgm', saved_img)
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # cv2.imwrite('map/originMap.pgm', img)
        # print('Blured Map Finshed!')

        end_time = time.perf_counter()
        execution_time = (end_time - start_time) * 1000

        print("程序运行时间：", execution_time, "毫秒")


        # 显示图像
        cv2.imshow('originMap', img)

        cv2.imshow('bluredMap', blured_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    map = mapBlur()
    map.mapBlur()
