#!/usr/bin/env python

import rospy
import rosnode
import roslaunch
from tf_conversions import transformations
from math import pi
import tf
import csv
from geometry_msgs.msg import PoseStamped,Twist,PoseWithCovarianceStamped
import threading
from goal_loop import MultiGoals
import numpy as np
import os
import rospkg
from mapBlur import mapBlur
import psutil



    def goal_loop(self):
        try:
            # ROS Init
            #rospy.init_node('multi_goals', anonymous=True)
            retry = 1
            goalList = []
            goalListX = []
            goalListY = []
            goalListZ = []
            goalListW = []
            map_frame = rospy.get_param('~map_frame', 'map')

            with open('nav_point.csv', 'r') as f:
                reader = csv.reader(f)

                for cols in reader:
                    goalList.append([float(value) for value in cols])

                goalList = np.array(goalList)
                print("read suc!!")
                goalListX = goalList[:, 0]
                goalListY = goalList[:, 1]
                goalListZ = goalList[:, 2]
                goalListW = goalList[:, 3]

            if len(goalListX) == len(goalListY) & len(goalListY) >= 1:
                # Constract MultiGoals Obj
                rospy.loginfo("Multi Goals Executing...")
                mg = MultiGoals(goalListX, goalListY,goalListW,goalListZ ,retry, map_frame)
                rospy.spin()
            else:
                rospy.loginfo("Lengths of goal lists are not the same")
        except KeyboardInterrupt:
            print("shutting down")


class Run_launch():
    def __init__(self,launch_dir):  
        self.uuid=roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.tracking_launch = roslaunch.parent.ROSLaunchParent(self.uuid, launch_dir) # launch 文件路径

    def start(self): # 运行launch文件
        self.tracking_launch.start()

    def shutdown(self): # 关闭launch文件
        self.tracking_launch.shutdown()

class Get_Point:
    def __init__(self,csv_write):
        self.tf_listener = tf.TransformListener()
        self.csv_write=csv_write
        
        try:
            self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))
            
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return
        
    def get_pos(self): 
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        # rospy_Time(0)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf Error")
            return None
        return trans[0],trans[1],rot[2],rot[3]

class Judgement():
    def __init__(self):
        self.laser_controller=rospy.Subscriber(
                "laser_control", Twist , self.laser_control_info, queue_size=10) # 订阅雷达控制信息
        self.nav_controller=rospy.Subscriber(
                "nav_control", Twist , self.nav_control_info, queue_size=10) # 订阅导航控制信息
        self.light_controller=rospy.Subscriber(
                "reg_green_light", int8 , self. cv_control_info, queue_size=10) # 订阅红绿灯识别
        self.pub = rospy.Publisher( 
            "/car/cmd_vel", Twist, queue_size=10) # 发布最终的控制信息
        self.control_info=Twist()
        self.state="laser" 
        self.is_red_green_light=0

    def laser_control_info(self,data):
        if self.state=="laser":
            self.control_info.linear.x=data.linear.x
            self.control_info.angular.z=data.angular.z
            self.pub.publish(self.control_info)
        elif self.state=="stop":
            self.control_info.linear.x=1500
            self.control_info.angular.z=90
            self.pub.publish(self.control_info)

    def nav_control_info(self,data):
        if self.state=="nav":
            self.control_info.linear.x = data.linear.x
            self.control_info.angular.z = data.angular.z
            self.pub.publish(self.control_info)
         elif self.state=="stop":
            self.control_info.linear.x=1500
            self.control_info.angular.z=90
            self.pub.publish(self.control_info)

    def cv_control_info(self,data):
        if data==1:
            self.is_red_green_light=1


    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node('main')
    #此处文件路径为实际路径，需要根据实际情况修改
    rviz=Run_launch(["/home/scnu-car/vanguardcar/src/racecar/launch/rviz.launch"])
    gmapping=Run_launch(["/home/scnu-car/vanguardcar/src/racecar/launch/Run_gmapping.launch"]) 
    amcl=Run_launch(["/home/scnu-car/vanguardcar/src/racecar/launch/amcl_nav_teb.launch"])
    map_save=Run_launch(["/home/scnu-car/vanguardcar/src/racecar/launch/map_save.launch"])
    map_adv=Run_launch(["/home/scnu-car/vanguardcar/src/racecar/launch/map_adv.launch"])
    gmapping.start() # 开建图
    rviz.start()  # 开rviz
    initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1) # 开导航后用于发布初始位置
    pose_msg = PoseWithCovarianceStamped() 
    judger=Judgement() # 实例化对象
    t1=threading.Thread(target = judger.spin) # 创建线程，该线程执行judge.spin()
    t1.start() # 开启线程
    t2=threading.Thread(target = loop_run) # 创建线程，该线程执行judge.spin()
    t2.start() # 开启线程
    csv_dir = "nav_point.csv" 
    with open(csv_dir,'w',encoding='utf8',newline='') as f :
        csv_write = csv.writer(f, delimiter=',', quoting=csv.QUOTE_MINIMAL)
        nav_point = Get_Point(csv_write)
        r= rospy.Rate(10) 
        last_x=0
        last_y=0
        flg=0 
        while not rospy.is_shutdown():
            
            if judger.state=="laser":
                x,y,z,w=nav_point.get_pos()
                if judger.is_red_green_light==1: # 红绿灯停车
                    judger.state="stop" 
                    flg=0
                    f.close()
                    map_save.start()
                    rospy.sleep(1) 
                    amcl.start()
                    rospy.sleep(1) 
                    pose_msg.header.frame_id = "map"
                    pose_msg.pose.pose.position.x = x
                    pose_msg.pose.pose.position.y = y
                    pose_msg.pose.covariance[0] = 0.25
                    pose_msg.pose.covariance[6 * 1 + 1] = 0.25
                    pose_msg.pose.covariance[6 * 5 + 5] = 0.06853892326654787
                    pose_msg.pose.pose.orientation.z = z
                    pose_msg.pose.pose.orientation.w = w
                    initial_pose_pub.publish(pose_msg)
                    t2.start() 
                    judger.state="nav"
                    rosnode.kill_nodes(["slam_gmapping"]) # 关闭建图
                    rosnode.kill_nodes(["laser_go"]) # close laser_control
                      
                elif ((x-last_x)**2+(y-last_y)**2)**(0.5)>=1 or flg==0: #欧式距离采点值
                    csv_write.writerow([x,y,z,w])
                    flg=1
                    last_x=x
                    last_y=y
            r.sleep()
        amcl.shutdown()
        map_save.shutdown()
   