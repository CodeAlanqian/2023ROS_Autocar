#导包
import rospy
#导入Twist信息
from geometry_msgs.msg import Twist

#初始化控制节点
rospy.init_node('racecar_control')

#创建发布者
pub = rospy.Publisher('~/car/cmd_vel', Twist, queue_size=5)
twist = Twist()

#设置线速度和角度
twist.linear.x = 1700
twist.linear.y = 0
twist.linear.z = 0
                    
twist.angular.x = 0
twist.angular.y = 0
twist.angular.z = 90
while(True):
    #一直发布该信息，使智能小车以1700的线速度向前运动
    pub.publish(twist)