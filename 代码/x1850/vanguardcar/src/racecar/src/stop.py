import rospy
from geometry_msgs.msg import Twist

rospy.init_node('stop')

pub = rospy.Publisher('/car/cmd_vel', Twist, queue_size=5)

twist = Twist()

twist.linear.x = 1500
twist.linear.y = 0
twist.linear.z = 0

twist.angular.x = 0
twist.angular.y = 0
twist.angular.z = 90

while  not rospy.is_shutdown():
    pub.publish(twist)