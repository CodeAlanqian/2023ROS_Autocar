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


csv_dir = "nav_point.csv" 
with open(csv_dir,'w',encoding='utf8',newline='') as f :
    csv_write = csv.writer(f, delimiter=',', quoting=csv.QUOTE_MINIMAL)
    nav_point = Get_Point(csv_write)
    r= rospy.Rate(10) 
    last_x=0
    last_y=0
    flg=0  #是否是原点
    while not rospy.is_shutdown():
        if ((x-last_x)**2+(y-last_y)**2)**(0.5)>=1 or flg==0: #欧式距离采点或者根据时间采点 time时间戳差值
                csv_write.writerow([x,y,z,w])
                flg=1
                last_x=x
                last_y=y
        r.sleep()