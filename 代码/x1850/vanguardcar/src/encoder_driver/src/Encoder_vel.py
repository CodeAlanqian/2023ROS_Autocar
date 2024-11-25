#!/usr/bin/env python3
#coding:UTF-8

import rospy
from rospy.timer import Rate
import  serial
import time

from nav_msgs.msg import Odometry



Bytenum_vel = 0               #读取到这一段的第几位
Bytenum_dir = 0
last_pose = 0
C = 227.45/1000 
 

def DueVelData(inputdata):   #新增的核心程序，对读取的数据进行划分，各自读到对应的数组里
      #在局部修改全局变量，要进行global的定义
    global  Bytenum_vel

    for data in inputdata:  #在输入的数据进行遍历 'ex:01 03 04 02 7A (D8 C6)'
        # print(data)
        if data==0x01 and Bytenum_vel==0: 
            Bytenum_vel = 1
            continue
        if data==0x03 and Bytenum_vel==1:
            Bytenum_vel = 2
            continue
        if data==0x02 and Bytenum_vel==2:
            Bytenum_vel = 3
            continue
        if Bytenum_vel==3:
            data_high = data
            Bytenum_vel = 4
            continue
        if Bytenum_vel==4:
            data_low = data
            Bytenum_vel = 0
            Angle_vel= data_high * 256 + data_low
            return float(Angle_vel)

def DueDirData(inputdata):   #新增的核心程序，对读取的数据进行划分，各自读到对应的数组里
      #在局部修改全局变量，要进行global的定义
    global  Bytenum_dir
    global  last_pose

    for data in inputdata:  #在输入的数据进行遍历 'ex:01 03 02 01 42 (39 E5)'
        # print(data)
        if data==0x01 and Bytenum_dir==0: 
            Bytenum_dir = 1
            continue
        if data==0x03 and Bytenum_dir==1:
            Bytenum_dir = 2
            continue
        if data==0x02 and Bytenum_dir==2:
            Bytenum_dir = 3
            continue
        if Bytenum_dir==3:
            data_high = data
            Bytenum_dir = 4
            continue
        if Bytenum_dir==4:
            data_low = data
            Bytenum_dir = 0
            position = data_high * 256 + data_low
            pose = position - last_pose
            last_pose = position

            if (pose>=0 and pose<512) or (pose> -1024 and pose< -512):
                direction = 1
            elif (pose<0 and pose> -512) or (pose < 1024 and pose > 512):
                direction = -1

            return int(direction)

def init_topic():
    

    return 0 

 
if __name__=='__main__': 
    rospy.init_node('encoder_vel', log_level=rospy.DEBUG)
    pub = rospy.Publisher('encoder', Odometry, queue_size=10)

    port = rospy.get_param('~serial_port', '/dev/encoder')
    baud = rospy.get_param('~baud_rate', 57600) # about 100hz
    k    = rospy.get_param('~k',1) # fix param
    
    ser = serial.Serial(port, baud) 
    # print(ser.is_open)

    while( not rospy.is_shutdown()):
        # time1 = time.time()
        send_data = bytes.fromhex('01 03 00 03 00 01 74 0A')  # read velocity value in 20ms
        ser.write(send_data)
        datahex = ser.read(7)
        angle_v = DueVelData(datahex) 

        send_data = bytes.fromhex('01 03 00 00 00 01 84 0A') #
        ser.write(send_data)
        datahex = ser.read(7)
        dir = DueDirData(datahex) 

        Vel = angle_v * dir * C / 1024.0 / 0.02 * k
        # print(Vel)
        # time2 = time.time()
        # print(1/(time2-time1))

        pub_vel = Odometry()
        pub_vel.header.frame_id = 'odom'
        pub_vel.child_frame_id = 'base_footprint'
        pub_vel.header.stamp = rospy.Time.now()
        pub_vel.twist.twist.linear.x = Vel
        pub.publish(pub_vel)


        



