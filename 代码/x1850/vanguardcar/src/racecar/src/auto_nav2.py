#!/usr/bin/env python

#赛道红绿灯在前，停车点在后

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
from multiprocessing import Process
y_n=-1.75   #正
y_p=1       #负
stop_x=np.array([[-9.5,-9.0],[-8.67,-8.17],[-7.5,-7],[-6.37,-5.87],[-5.24,-4.74],[-4.1,-3.61],[-3.6,-3.1]])
light_x=np.array([[-9.5,-9.0],[-8.67,-8.17],[-7.5,-7],[-6.37,-5.87],[-5.24,-4.74],[-4.1,-3.61],[-3.6,-3.1]])
stop_idx=5-1
light_idx=3-1


def loop_run():
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
        self.tracking_launch = roslaunch.parent.ROSLaunchParent(self.uuid, launch_dir)

    def start(self):
        self.tracking_launch.start()

    def shutdown(self):
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

        # data = PoseStamped()  
        # data.pose.position.x = trans[0]
        # data.pose.position.y = trans[1]
        # data.pose.orientation.z = rot[2]
        # data.pose.orientation.w = rot[3]
        # pub.publish(data)
        #print(1)
        # self.csv_write.writerow([trans[0],trans[1],rot[2],rot[3]])  
        return trans[0],trans[1],rot[2],rot[3]

class Judgement():
    def __init__(self):
        self.laser_controller=rospy.Subscriber(
                "laser_control", Twist , self.laser_control_info, queue_size=10)
        self.nav_controller=rospy.Subscriber(
                "nav_control", Twist , self.nav_control_info, queue_size=10)
        self.pub = rospy.Publisher(
            "/car/cmd_vel", Twist, queue_size=10)
        self.control_info=Twist()
        self.state="laser"

    def laser_control_info(self,data):
        if self.state=="laser":
            self.control_info.linear.x=data.linear.x
            self.control_info.angular.z=data.angular.z
            self.pub.publish(self.control_info)
        elif self.state=="stop":
            self.control_info.linear.x=1500
            self.control_info.angular.z=90
            self.pub.publish(self.control_info)
        elif self.state=="laser1":
            self.control_info.linear.x=1880
            self.control_info.angular.z=data.angular.z
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
        pass

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node('main')
    rviz=Run_launch(["/home/scnu-car/vanguardcar/src/racecar/launch/rviz.launch"])
    gmapping=Run_launch(["/home/scnu-car/vanguardcar/src/racecar/launch/Run_gmapping.launch"]) 
    amcl=Run_launch(["/home/scnu-car/vanguardcar/src/racecar/launch/amcl_nav_teb_light.launch"]) # amcl 里面删掉goal_loop和保存地图
    map_save=Run_launch(["/home/scnu-car/vanguardcar/src/racecar/launch/map_save.launch"])#重新搞launch文件
    #map_adv=Run_launch(["/home/scnu-car/vanguardcar/src/racecar/launch/map_adv.launch"])]
    amcl.start()
    gmapping.start()
    #amcl.start()
    rviz.start()
    #mapblur=mapBlur()
    csv_dir = "nav_point.csv" 
    judger=Judgement()
    t1=threading.Thread(target = judger.spin)
    t1.start()
    goal_loop=threading.Thread(target = loop_run)
    with open(csv_dir,'w',encoding='utf8',newline='') as f :
        csv_write = csv.writer(f, delimiter=',', quoting=csv.QUOTE_MINIMAL)
        nav_point = Get_Point(csv_write)
        r= rospy.Rate(40) 
        last_x=0
        last_y=0 
        flg_start=0
        flg=0
        while not rospy.is_shutdown():
            x,y,z,w=nav_point.get_pos()
            if x<=light_x[light_idx,1] and x>=light_x[light_idx,0]and y<=y_p and y>=y_n and flg: #第一圈遇到红绿灯
                if(judger.state=="laser"):
                    judger.state="stop"
                    #print(1)
                    flg=0
                 #是否在这里关gmapping和保存地图，如果红绿灯靠中间就不用该条件
                    map_save.start()
                    rosnode.kill_nodes(["slam_gmapping"])
                    #rospy.sleep()
                    judger.state="laser1"  
                else:
                    judger.state="stop"
                    rospy.sleep(3) 
                    judger.state="nav"
            #按格来算位置
            elif x<=-2 and x>=-3.5 and y<=y_p and y>=y_n and judger.state=="laser1" and flg_start==0: # 到直到中间位置,如果红绿灯靠中间就不用该条件
                #保存地图，关闭gmapping，直接插点开启goal_loop
                #map_save.start()
                #rosnode.kill_nodes(["slam_gmapping"])
                f.close()
                goal_loop.start()
                flg_start=1
            elif x<=0 and x>=-1 and y<=y_p and y>=y_n  and judger.state=="laser1": #第一圈结束，切换成导航
                #如果停车点距离原点很近
                rosnode.kill_nodes(["laser_go"])
                judger.state="nav"
                flg=1
                #pass
            elif x<=stop_x[stop_idx,1] and x>=stop_x[stop_idx,0] and y<=y_p and y>=y_n  and judger.state=="nav":  #停车点
                judger.state="stop"

            elif (((x-last_x)**2+(y-last_y)**2)**(0.5)>=0.8 and judger.state=="laser") : #收集导航点
                csv_write.writerow([x,y,z,w])
                last_x=x
                last_y=y
                flg=1

            r.sleep()
        amcl.shutdown()