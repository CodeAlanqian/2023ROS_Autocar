#get the point and ore form rviz to txt
#Type: geometry_msgs/PoseStamped  /move_base_simple/goal
#todo: change the count to the key "finish"
#todo: not the cover the last csv 
import rospy
import math
import sys,select,termios,tty
import tf
import csv
import codecs
from geometry_msgs.msg import PoseStamped

count = 0
point=[]
def PoseStampedCB(data):
    global count
    gx = data.pose.position.x
    gy = data.pose.position.y
    gz = data.pose.orientation.z
    gw = data.pose.orientation.w
    #count = count + 1
    point.append([gx,gy,gz,gw])
    if(count == 3):
        data_write_csv('test.csv',point)
    # with open('test.csv','wb') as csvfile:
    #     pointwriter = csv.writer(csvfile,delimiter=' ',quotechar='|',quoting=csv.QUOTE_MINIMAL)
    #     pointwriter.writerow([gx,gy,gz,gw])
    #     print("write succ!!")

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist,_,_ = select.select([sys.stdin],[],[],0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin,termios.TCSADRAIN,settings)
    return key
 
def data_write_csv(file_name, datas):
    file_csv = codecs.open(file_name,'w+','utf-8')
    writer = csv.writer(file_csv, delimiter=',', quoting=csv.QUOTE_MINIMAL)
    for data in datas:
        writer.writerow(data)
    print("write succ!!")

if __name__ == '__main__':
    try:
        rospy.init_node("getPoint",anonymous=True)
        rospy.Subscriber('/move_base_simple/goal',PoseStamped,PoseStampedCB,queue_size=10)
        settings = termios.tcgetattr(sys.stdin)
        while(1):
            Key = getKey()
            if Key == 'f' or Key == 'F':
                data_write_csv('test.csv',point)
            if Key == '\x03':
                break
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 
