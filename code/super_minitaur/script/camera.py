#!/usr/bin/env python
# coding:utf-8
import cv2
import numpy as np
import time
import math
# from super_minitaur.srv import StateFlag
# import rospy

# from numba import jit
# @jit


def checkbackgroundandstep(frame,background,contours,kernel = np.ones((10, 10), np.uint8),lower=np.array([70, 50, 20]),upper=np.array([200, 150, 80])):
    roi = np.zeros(frame.shape[0:2], dtype=np.uint8)  # 掩膜
    cv2.drawContours(roi, contours, background, 255, -1)
    result1 = cv2.bitwise_and(frame, frame, mask=roi)
    mask = cv2.inRange(result1, lower, upper)
    result = cv2.bitwise_and(result1, result1, mask=mask)
    # (mean, stddv) = cv2.meanStdDev(result)  # 计算掩膜抠出部分带黑色背景的平均rgb
    # area = cv2.contourArea(contours[background])  # 计算边缘包围面积
    # realbackmean = (mean[0][0] * width * height / area,mean[1][0] * width * height / area,mean[2][0] * width * height / area)
    backgray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    (_, backthresh) = cv2.threshold(backgray, 1, 255, cv2.THRESH_BINARY)  # 阈值分割
    backdilation = cv2.dilate(backthresh, kernel)  # 膨胀   闭运算
    backerosion = cv2.erode(backdilation, kernel)  # 腐蚀
    backcontours, hierarchy = cv2.findContours(backerosion, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  #背景轮廓抽象
    if backcontours != []:
        for i in range(len(backcontours)):
            epsilon = 0.01 * cv2.arcLength(backcontours[i], True)
            approx = cv2.approxPolyDP(backcontours[i], epsilon, False)    #直线拟合 返回点
            real = []    #记录点
            delete = []   #记录较近点序号
            for i in range(len(approx)):    #删除靠下方的点
                if approx[i, 0, 1] < 360:    #360可更改
                    real.append((approx[i, 0, 0], approx[i, 0, 1]))
            for i in range(len(real)):      #删除较近点
                for j in range(len(real) - i - 1):
                    if (pow(real[i][0] - real[j + i + 1][0], 2) + pow(real[i][1] - real[j + i + 1][1], 2) < 12100):
                        delete.append(j + i + 1)
            delete = list(set(delete))     #去重
            delete.sort()                  #排序
            # print real
            # print delete
            for i in range(len(delete)):
                del real[delete[len(delete) - i - 1]]
            min1 = (0, 480)                #记录目标点
            min2 = (0, 480)
            for i in range(len(real)):
                if real[i][1] < min1[1]:
                    min1 = real[i]
            if min1 != [] and min1 != (0, 480):
                real.remove(min1)
                for i in range(len(real)):
                    if real[i][1] < min2[1]:
                        min2 = real[i]
                if min2 != [] and min2 != (0, 480):
                    cv2.line(result, min1, min2, (0, 255, 0), 2)
                    cv2.imshow('result', result)
                    return [min1, min2]



def checklineandground(frame,contours,groundmean,linemean=50):  #传入frame、轮廓、背景灰度、线相对亮度
    countlight = []  # 记录亮轮廓
    countdark = []  # 记录暗轮廓
    maxarea = 0
    limitarea = 10000 # 最大轮廓面积
    for i in range(len(contours)):  # 筛选边缘，去除过小或过大轮廓
        if contours[i].size > 300:
            roi = np.zeros(frame.shape[0:2], dtype=np.uint8)  # 掩膜
            cv2.drawContours(roi, contours, i, 255, -1)
            result = cv2.bitwise_and(gray, gray, mask=roi)
            (mean, stddv) = cv2.meanStdDev(result)  # 计算掩膜抠出部分带黑色背景的平均灰度
            area = cv2.contourArea(contours[i])  # 计算边缘包围面积
            realmean = mean * width * height / area  # 计算掩膜抠出部分真实灰度
            if realmean > groundmean + linemean:  # 找出类似白色区域（高灰度）
                if (area > maxarea and area < limitarea):  # 排除场地背景反光区域
                    maxarea = area
                    countlight.insert(0, i)
                elif (area < limitarea):
                    countlight.append(i)
            else:
                if (area > maxarea):  # 排除场地背景反光区域
                    maxarea = area
                    countdark.insert(0, i)
                else:
                    countdark.append(i)
    if (len(countlight) == 1):
        cv2.drawContours(frame, contours, countlight[0], (0, 0, 255), -1)  # 标记目标红色区域
    elif (len(countlight) > 1):
        for i in range(len(countlight) - 1):
            cv2.drawContours(frame, contours, countlight[i+1], (0, 0, 255), -1)  # 标记目标红色区域
    if len(countdark):
        return countdark[0]

def judgedirection(distance,judge1 = 60):    #防线判断 judge是误差范围（可改成角度）
    if(distance[1] == 0):
        return 'forward'
    elif(distance[0] == 0):
        if (distance[2] != 0):
            return 'left'
        elif (distance[3] != 0):
            return 'right'
        else:
            return 'forward'
    else:
        if(abs(distance[0])<judge1 and abs(distance[1])<judge1):
            return 'forward'
        elif(abs(distance[0]-distance[1])<judge1):
            return 'forward'
        elif(distance[0]-distance[1]<0):
            return 'left'
        else:
            return 'right'



def checkdirection(img,line1 = 0.25,line2 = 0.75,point = 15,line3 = 0.25,line4 = 0.75):  #检测方向 line是辅助线1234上下左右
    height = img.shape[0]
    width = img.shape[1]
    k=[0,0,0,0]
    distance=[0,0,0,0]
    red = 255
    upper = np.array([0, 0, 255])  # 颜色上限
    mask = cv2.inRange(img, upper, upper)# 白线用rbg测试效果好于hsv hsl
    for i in range (width) :
        if mask.item(int(height*line1),i) == red :
            k[0]=k[0]+1
            distance[0]=distance[0]+i-width/2
        if mask.item(int(height*line2),i) == red :
            k[1]=k[1]+1
            distance[1]=distance[1]+i-width/2
    if k[0]>point:
        distance[0] = distance[0] / float(k[0])
    else:
        distance[0] = 0
        for i in range (height):
            if mask.item(i,int(height*line3)) == red :
                k[2] = k[2] + 1
                distance[2] = distance[2] + i - height / 2
            if mask.item(i,int(height*line4)) == red :
                k[3] = k[3] + 1
                distance[3] = distance[3] + i - height / 2
        if k[2] > point:
            distance[2] = distance[2] / float(k[2])
        else:
            distance[2] = 0
        if k[3] > point:
            distance[3] = distance[3] / float(k[3])
        else:
            distance[3] = 0

    if k[1]>point:
        distance[1] = distance[1] / float(k[1])
    else:
        distance[1] = 0

    realdirection=judgedirection(distance)

    cv2.line(img, (0, int(height * line1)), (width, int(height * line1)), (255,255,255),2)  # draw line
    cv2.line(img, (0, int(height * line2)), (width, int(height * line2)), (255,255,255),2)
    cv2.line(img, (int(width * line3), 0), (int(width * line3), height), (255,255,255),2)
    cv2.line(img, (int(width * line4), 0), (int(width * line4), height), (255,255,255),2)

    cv2.line(img, (0, int(height * 0.5)), (width, int(height * 0.5)), (0,0,0),2)
    cv2.line(img, (int(width * 0.5), 0), (int(width * 0.5), height), (0,0,0),2)

    print distance
    print realdirection


def pub (camerastate,number):
    # message=[0,0,0,0,0,0]
    # message[number]=1
    # camerastate.call(message)
    pass

if __name__=="__main__":
    # count = 0
    # t = time.time()
    jumpflag = [50,20,200*2,50,20,200*2]
    flag = [0,0,0,0,0,0]
    cap = cv2.VideoCapture(1)
    cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G')) #设置摄像头通信编码格式
    kernel = np.ones((3, 3), np.uint8) #腐蚀膨胀核心
    camerastate=1
    # camerastate = rospy.ServiceProxy("/state_flag_camera", StateFlag)
    # ret = cap.set(3, 320)   #设置分辨率，默认640*480，帧数120（90） 1280帧数60 若更改则需改变程序所有参数
    # ret = cap.set(4, 240)
    while True:
        # count += 1
        ret, frame = cap.read()
        height = frame.shape[0]
        width = frame.shape[1]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)    #二值化因为线与背景差别更大更明显
        # ret1, binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
        gradX = cv2.Sobel(gray, ddepth=cv2.CV_32F, dx=1, dy=0) #梯度处理
        gradY = cv2.Sobel(gray, ddepth=cv2.CV_32F, dx=0, dy=1)
        # gradient = cv2.subtract(gradX, gradY)
        gradientX = cv2.convertScaleAbs(gradX)
        gradientY = cv2.convertScaleAbs(gradY)
        gradient = cv2.addWeighted(gradientX, 1, gradientY, 1, 0) #梯度叠加
        blurred = cv2.GaussianBlur(gradient, (5, 5), 0) #高斯模糊
        (_, thresh) = cv2.threshold(blurred, 45, 200, cv2.THRESH_BINARY) #阈值分割
        dilation = cv2.dilate(thresh, kernel) #膨胀   闭运算
        erosion = cv2.erode(dilation, kernel) #腐蚀
        cv2.rectangle(erosion, (0, 0), (width-1, height-1), 255) #加外框边缘
        contours, hierarchy = cv2.findContours(erosion, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #边缘检测
        (groundmean, groundstddv) = cv2.meanStdDev(gray)  #计算环境平均灰度

        background = checklineandground(frame,contours, groundmean, linemean=50)  #标记线并检测背景  linemean是线相对亮度

        key = cv2.waitKey(1)
        if key == ord('b'):
            if flag != [1,1,1,1,1,1]:
                if background != None:
                    point = checkbackgroundandstep(frame, background, contours)
                    print point
                    if (point[0] != (0,480) and point[1] != (0,480)):
                        if flag[0] == 0:              #判断沙丘存在
                            if point[0][1] > 30  or  point[1][1]  > jumpflag[0]:
                                flag[0] = 1
                                pub(camerastate,0)
                        elif flag[0] == 1:
                            if flag[1] == 0:          #判断平行
                                if abs(point[0][1] - point[1][1]) < jumpflag[1]:
                                    flag[1] = 1
                                    pub(camerastate,1)
                            elif flag[1] == 1:
                                if flag[2] == 0:      #判断距离合适
                                    if point[0][1] + point[1][1]  > jumpflag[2]:
                                        flag[2] = 1
                                        pub(camerastate,2)
                                    elif flag[2] == 1:
                                        if flag[3] == 0:  # 判断绳子存在
                                            if point[0][1] > 30 or point[1][1] > jumpflag[3]:
                                                flag[3] = 1
                                                pub(camerastate, 3)
                                        elif flag[3] == 1:
                                            if flag[4] == 0:  # 判断平行
                                                if abs(point[0][1] - point[1][1]) < jumpflag[4]:
                                                    flag[4] = 1
                                                    pub(camerastate, 4)
                                            elif flag[4] == 1:
                                                if flag[5] == 0:  # 判断距离合适
                                                    if point[0][1] + point[1][1] > jumpflag[5]:
                                                        flag[5] = 1
                                                        pub(camerastate, 5)
            print flag


        if key == ord('q'):
            break
        if key == ord('c'):
            checkdirection(frame)
        # if (time.time() - t > 10):
        #     print count
        #     break
        if key == ord('p'):
            pass
            # for i in range(len(contours)):  # 筛选边缘
            #     if contours[i].size > 300 and contours[i].size < 2000:
            #         epsilon = 0.001 * cv2.arcLength(contours[i], True)
            #         contours[i] = cv2.approxPolyDP(contours[i], epsilon, True)
            #         roi = np.zeros(frame.shape[0:2], dtype=np.uint8)
            #         cv2.drawContours(roi, contours, i, 255, -1)
            #         roi1 = cv2.GaussianBlur(roi, (9, 9), 0)
            #         result = cv2.bitwise_and(gray, gray, mask=roi)
            #         (mean, stddv) = cv2.meanStdDev(result)
            #         area = cv2.contourArea(contours[i])
            #         realmean = mean * width * height / area
            #         (groundmean, groundstddv) = cv2.meanStdDev(gray)
            #         if realmean > groundmean + 50:
            #             dst = cv2.cornerHarris(roi1, 2, 5, 0.04)
            #             cv2.imshow('dst', dst)
            #             (dstmean, dststddv) = cv2.meanStdDev(dst)
            #             dstmean = dstmean * height * width
            #             print dstmean
        cv2.imshow('frame', frame   )
        # cv2.setMouseCallback("frame", getposBgr)

    cap.release()
    cv2.destroyAllWindows()


    # cap = cv2.VideoCapture(1)
    # while True:
    #     ret, frame = cap.read()
    #     blurred = cv2.GaussianBlur(frame, (3, 3), 0)
    #     gray = cv2.cvtColor(blurred, cv2.COLOR_RGB2GRAY)
    #     edge_output = cv2.Canny(gray, 35, 100)
    #     cv2.imshow("Canny Edge", gray)
    #     key = cv2.waitKey(1)
    #     if key == ord('q'):
    #         break
    # cap.release()
    # cv2.destroyAllWindows()