# 导入所需模块
import cv2
import numpy as np
import imutils

from jetbot import Robot

import time

import Jetson.GPIO as GPIO

from threading import Timer
import sys

prevData=0
p=10
q=0.01
r=0.005
kGain=0
def kalmanFilter_A(inData):
    global prevData
    global p
    global q
    global r
    global kGain
    p = p+q 
    kGain = p/(p+r)

    inData = prevData+(kGain*(inData-prevData)) 
    p = (1-kGain)*p

    prevData = inData

    return inData 

class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.clear()
    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.int_error = 0.0
        self.windup_guard = 20.0
        self.output = 0.0
    def update(self, feedback_value):
        error = self.SetPoint - feedback_value
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error
        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error#比例
            self.ITerm += error * delta_time#积分
            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard
            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time
            self.last_time = self.current_time
            self.last_error = error
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
    def setKp(self, proportional_gain):
        self.Kp = proportional_gain
    def setKi(self, integral_gain):
        self.Ki = integral_gain
    def setKd(self, derivative_gain):
        self.Kd = derivative_gain
    def setWindup(self, windup):
        self.windup_guard = windup
    def setSampleTime(self, sample_time):
        self.sample_time = sample_time

class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.clear()
    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.int_error = 0.0
        self.windup_guard = 0.5
        self.output = 0.0
    def update(self, feedback_value):
        error = self.SetPoint - feedback_value
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error
        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error#比例
            self.ITerm += error * delta_time#积分
            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard
            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time
            self.last_time = self.current_time
            self.last_error = error
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
            
            if self.output>0.2:
                self.output=0.2
            elif self.output<-0.2:
                self.output=-0.2

            # print('delta_time',delta_time)
            # print('P',self.PTerm,'I',self.ITerm,'D',self.DTerm)
            #print('error',error)

    def setKp(self, proportional_gain):
        self.Kp = proportional_gain
    def setKi(self, integral_gain):
        self.Ki = integral_gain
    def setKd(self, derivative_gain):
        self.Kd = derivative_gain
    def setWindup(self, windup):
        self.windup_guard = windup
    def setSampleTime(self, sample_time):
        self.sample_time = sample_time

#求最大连通域的中心点坐标
def centroid(max_contour):
    moment = cv2.moments(max_contour)
    if moment['m00'] != 0:
        cx = int(moment['m10'] / moment['m00'])
        cy = int(moment['m01'] / moment['m00'])
        return cx, cy
    else:
        return None   

def dirPID(inc):
    global delta_time_user,current_time_user,last_time_user
    global height
    global cnt_centroid

    dirControler.update(cnt_centroid[0])
    #print('output',dirControler.output)
    # last_time_user=current_time_user
    # current_time_user=time.time()
    # delta_time_user=current_time_user-last_time_user
    # print('delta time',delta_time_user)
    t = Timer(inc, dirPID,(inc,))
    t.start()

def robotOutput(control):
    if speedUpFlag:
        robot.left_motor.value=basic_output
        robot.right_motor.value=basic_output
    else:
        robot.left_motor.value=basic_output-control
        robot.right_motor.value=basic_output+control

basic_speed=0.39#0.4
low_speed=0.3
high_speed=0.45
medium_speed=0.37
basic_output=0

# 打开摄像头
#3:4
height=240
width=320

speedPoint1=0.32
speedPoint2=0.44
speedPoint3=0.5333

offset=5

cap = cv2.VideoCapture(0)

# dirControler.sample_time=0.02

#run
robot=Robot()

#count time
current_time_user=time.time()
last_time_user=current_time_user
#GPIO
LED_Pin = 38
switch1_Pin = 19
switch2_Pin = 21
switch3_Pin = 23
switch4_Pin = 29
switch5_Pin = 31
switch6_Pin = 33
switch7_Pin = 35
switch8_Pin = 37

GPIO.setmode(GPIO.BOARD)
GPIO.setup(LED_Pin, GPIO.OUT)
GPIO.output(LED_Pin, GPIO.HIGH)

GPIO.setup(switch1_Pin, GPIO.IN)
GPIO.setup(switch2_Pin, GPIO.IN)
GPIO.setup(switch3_Pin, GPIO.IN)
GPIO.setup(switch4_Pin, GPIO.IN)
GPIO.setup(switch5_Pin, GPIO.IN)
GPIO.setup(switch6_Pin, GPIO.IN)
GPIO.setup(switch7_Pin, GPIO.IN)
GPIO.setup(switch8_Pin, GPIO.IN)

startFlag=0
stopFlag=0
overCount=0
overFlag=0
speedUpFlag=0

cnt_centroid=(width//2-offset,0)
#PID
dirControler=PID(0.0004,0.000,0.0002)
dirControler.SetPoint=width//2-offset

dirPID(0.025)
startTime=time.time()

sum=0
speedUpTimer=0
currentTime=time.time()
while True:
    switch1_status=GPIO.input(switch1_Pin)
    switch7_status=GPIO.input(switch5_Pin)
    switch8_status=GPIO.input(switch6_Pin)
    #print('switch1_Pin:',switch1_status)
    #print('switch7_Pin:',switch7_status)
    #print('switch8_Pin:',switch8_status)
    
    lastTime=currentTime
    currentTime=time.time()
    sum+=(currentTime-lastTime)
    if switch1_status:
        startFlag=0
        stopFlag=0
        overCount=0
        overFlag=0
        speedUpFlag=0

        #basic_speed=0.45
        basic_output=0

        startTime=time.time()

        dirControler.SetPoint=width//2-offset
        cnt_centroid=(width//2-offset,0)
        # GPIO.output(LED_Pin, GPIO.LOW)
        time.sleep(0.5)
        GPIO.output(LED_Pin, GPIO.LOW)
        time.sleep(0.5)
        GPIO.output(LED_Pin, GPIO.HIGH)
        
        sum=0
        speedUpTimer=0
        currentTime=time.time()

    elif startFlag != 2 and startFlag!=3:
        startDeltaTime=time.time()-startTime
        # 读取每一帧
        _, frame = cap.read()
        # 重设图片尺寸以提高计算速度
        frame = imutils.resize(frame, height=height,width=width)#450*600
        # 进行高斯模糊
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        blurred = cv2.GaussianBlur(blurred, (11, 11), 0)
        blurred = cv2.GaussianBlur(blurred, (11, 11), 0)
        blurred = cv2.GaussianBlur(blurred, (11, 11), 0)
        # 转换颜色空间到HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # 定义绿色无图的HSV阈值
        lower = np.array([50, 60, 70])
        upper = np.array([90, 230, 140])
        # lower = np.array([100, 43, 46])
        # upper = np.array([124, 255, 255])
        # 对图片进行二值化处理
        mask = cv2.inRange(hsv, lower, upper)

        # # 腐蚀操作
        # mask = cv2.erode(mask, None, iterations=1)

        # 膨胀操作，先腐蚀后膨胀以滤除噪声
        mask = cv2.dilate(mask, None, iterations=1)

        #轮廓
        _,contours,hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        # print("number of contours:%d" % len(contours))
        cv2.drawContours(mask, contours, -1, (0, 255, 255), 2)

        #找到最大区域并填充
        area = []
        for i in range(len(contours)):
            area.append(cv2.contourArea(contours[i]))

        max_area=0
        if len(area)>0:
            max_area=max(area)
        #print('area',max_area)

        # if max_area>40000:
        #     GPIO.output(LED_Pin, GPIO.LOW)
        #     if not startFlag:
        #         startFlag=1
        #         startTime=time.time()
        # else:
        #     GPIO.output(LED_Pin, GPIO.HIGH)
        #     startFlag=0
        if startDeltaTime>3:#10
            GPIO.output(LED_Pin, GPIO.LOW)
            startFlag=1
            startTime=time.time()
        else:
            GPIO.output(LED_Pin, GPIO.HIGH)
            startFlag=0

        if startFlag==1:
            if startDeltaTime>2:#5
                for i in range(10):
                    time.sleep(0.2)
                    GPIO.output(LED_Pin, GPIO.LOW)
                    time.sleep(0.2)
                    GPIO.output(LED_Pin, GPIO.HIGH)
                startFlag=2
                startTime=time.time()

    elif startFlag==2:
        last_time_user=current_time_user
        current_time_user=time.time()
        delta_time_user=current_time_user-last_time_user
        #print('DT',delta_time_user)

        if (startFlag and startDeltaTime>2) or stopFlag:
            robotOutput(dirControler.output)

        # 读取每一帧
        _, frame = cap.read()
        # 重设图片尺寸以提高计算速度
        frame = imutils.resize(frame, height=height,width=width)#450*600
        # 进行高斯模糊
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        blurred = cv2.GaussianBlur(blurred, (11, 11), 0)
        blurred = cv2.GaussianBlur(blurred, (11, 11), 0)
        blurred = cv2.GaussianBlur(blurred, (11, 11), 0)
        # 转换颜色空间到HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # 定义绿色无图的HSV阈值
        # lower = np.array([0, 100, 150])
        # upper = np.array([6, 255, 220])
        lower = np.array([50, 60, 70])
        upper = np.array([90, 230, 140])
        #lower = np.array([50, 60, 80])
        #upper = np.array([90, 150, 160])
        # lower = np.array([100, 43, 46])
        # upper = np.array([124, 255, 255])
        # 对图片进行二值化处理
        mask = cv2.inRange(hsv, lower, upper)

        # # 腐蚀操作
        # mask = cv2.erode(mask, None, iterations=1)

        # 膨胀操作，先腐蚀后膨胀以滤除噪声
        mask = cv2.dilate(mask, None, iterations=3)
        # # 腐蚀操作
        # mask = cv2.erode(mask, None, iterations=1)

        #轮廓
        _,contours,hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        # print("number of contours:%d" % len(contours))
        # cv2.drawContours(mask, contours, -1, (0, 255, 255), 2)
        if speedUpFlag!=1:
            contours_temp=contours[:]
            contours=[]
            for cont in contours_temp:
                if centroid(cont)[1]<=110:
                    contours.append(cont)
            contours=np.array(contours)
        #找到最大区域并填充
        area = []
        for i in range(len(contours)):
            area.append(cv2.contourArea(contours[i]))
        # area.sort()
        idx=np.argsort(area)
        #max_idx=0
        #if len(area)>0:
        #    max_idx = np.argmax(area)

        #second_idx=0
        #secondArea=0
        #if len(area)>0:
        #    secondArea=min(area)
        #for i in range(len(contours)):
        #    if i != max_idx and cv2.contourArea(contours[i])>secondArea:
        #        secondArea=cv2.contourArea(contours[i])
        #        second_idx=i
        # cv2.drawContours(frame,contours,max_idx,(0,0,255),3) 

        # print('area:',area)
        # if len(contours)>0:
        #     print('contours\n',contours[max_idx])

        # black=np.zeros(mask.shape)
        # for i in range(max_idx - 1):
        #     cv2.fillConvexPoly(mask, contours[max_idx - 1], 0)
        #求最大连通域的中心坐标、填充
        cnt_centroid_x=width//2-offset
        cnt_centroid_y=0
        if len(contours)>0:
            # cnt_centroid = centroid(contours[max_idx])
            # print("Centroid : " + str(cnt_centroid))
            # cv2.fillConvexPoly(black, contours[max_idx], 255)
            if len(contours)>1:
                if abs(centroid(contours[idx[-2]])[1]-centroid(contours[idx[-1]])[1])<80 and (area[idx[-1]]-area[idx[-2]]<0.4*area[idx[-1]] or abs(centroid(contours[idx[-2]])[1]-centroid(contours[idx[-1]])[1])<20):
                    cnt_centroid_x,cnt_centroid_y = centroid(np.concatenate((contours[idx[-1]],contours[idx[-2]])))
                else:
                    cnt_centroid_x,cnt_centroid_y = centroid(contours[idx[-1]])            
            else:
                cnt_centroid_x,cnt_centroid_y = centroid(contours[idx[-1]])            
            #print('Row',(cnt_centroid_x,cnt_centroid_y))
            cnt_centroid_x=kalmanFilter_A(cnt_centroid_x)
            cnt_centroid=(int(cnt_centroid_x),int(cnt_centroid_y))
            #print("Centroid : " + str(cnt_centroid))
            # cv2.circle(frame, cnt_centroid, 1, (255,255,255), 10)            

        #速度设定
        if cnt_centroid[1]<height*speedPoint1 and not overCount:
            basic_output=0.12
        elif cnt_centroid[1]<height*speedPoint2 and not overCount:
            dirControler=PID(0.0006,0,0.0003)
            if switch7_status==0 and switch8_status==1:
                basic_output=low_speed
            elif switch7_status==0 and switch8_status==0:
                basic_output=high_speed
            elif switch7_status==1 and switch8_status==0:
                basic_output=medium_speed
            else:
                basic_output=basic_speed
            speedUpFlag=1
            
            if speedUpTimer==0:
                speedUpTimer=1

        if not overCount and speedUpTimer==2 and sum>0.45:#cnt_centroid[1]>height*speedPoint2 and cnt_centroid[1]<height*speedPoint3 and not overCount:
            stopFlag=1
            basic_output=-0.15
            startTime=time.time()
            overCount+=1
            
            sum=0
            speedUpTimer=3
        
        if speedUpTimer==1:
            speedUpTimer=2
            sum=0
            
        startDeltaTime=time.time()-startTime
        #print('basic_output',basic_output)
        #print('startDeltaTime',startDeltaTime)
        if overCount>0 and startDeltaTime>1:
            GPIO.output(LED_Pin, GPIO.LOW)
            basic_output=0
            robotOutput(0)
            startFlag=3
        # k = cv2.waitKey(5) & 0xFF
        # if k == 27:
        #     break
cap.release()
# cv2.destroyAllWindows()
overFlag=1