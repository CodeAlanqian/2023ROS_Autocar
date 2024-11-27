#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include "Control.h"


#define freq 1440//每转一圈雷达扫描次数
#define speed 1875

typedef struct //极坐标系下的点
{   
    double range;
    double theta;
}Point_polar;

typedef struct //直角坐标系下的点
{
    double x;
    double y;
}Point_rectangular;

class PubAndSub
{
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    geometry_msgs::Twist twist;
public:
    PubAndSub()
    {
        pub_ = n_.advertise<geometry_msgs::Twist>("/car/cmd_vel",5);
        sub_ = n_.subscribe("/scan",5,&PubAndSub::callback,this);
    }
    void callback(const sensor_msgs::LaserScan::ConstPtr &laser);
    ~PUbAndSub()
    {
        twist.angular.y = 0;
        twist.angular.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;
        pub_.publish(twist);
        ROS_INFO("shut down");
    }
};


Control::PID pid;

void PubAndSub::callback(const sensor_msgs::LaserScan::ConstPtr &laser)
{
    char negetiveNum = 0,positiveNum = 0;
    int i,j=0;
    double range = 0,error = 0,negetiveSum = 0,positiveSum = 0;
    Point_polar pp[30] = {0,0};
    Point_rectangular pr[30] = {0,0};

    for (i = 1;i < freq;i++)
    {
        if(laser->ranges[i-1] - laser->ranges[i] >= 2.0)
            if(laser->ranges[i] > 0.75 && laser->ranges[i] < 2.5)
            {
                pp[j].range = laser->ranges[i];
                pp[j].theta = i*laser->angle_increment + laser->angle_min;//获得2.5米范围内各锥桶的极坐标
                j++;
            }
    }
    for(i = 0;i < 30;i++)
    {
        if(pp[i].range)
        {
            pr[i].x = pp[i].range*sin(pp[i].theta);
            pr[i].y = pp[i].range*cos(pp[i].theta);//获得2.5米范围内各锥桶的直角坐标(激光雷达为原点)
            if(pr[i].y >= -0.2)//排除后方距离超过1米的锥桶
            ROS_INFO("found bucket point:(%.2f,%.2f)\n",pr[i].x,pr[i].y);
            else {
                pr[i].x = 0;
                pr[i].y = 0;
            }
            if(pr[i].x>0)
            {
                positiveNum ++;
                positiveSum += pr[i].x;
            }
            else if(pr[i].x)
            {
                negetiveNum ++;
                negetiveSum += pr[i].x;
            }
        }
    }

    for(i = 0;i< 30;i++)
    {
        if(pr[i].x>0)
            if(pr[i].x >= 1.775*positiveSum/positiveNum)
            {
                positiveSum -= pr[i].x;
                positiveNum --;
            }
        else if(pr[i].x<0)
            if(pr[i].x <= 1.775*negetiveSum/negetiveNum)
            {
                negetiveSum -= pr[i].x;
                negetiveNum --;
            }
                
    }
    error = negetiveSum + positiveSum;//通过锥桶的坐标算出误差(加权)

    //error = pid.PIDIncremental(error);//增量式PID控制器，参数在/src/Control.h Init函数中调节
    error = pid.PIDPositional(error);//位置式PID控制器
    //error *= 25.0;
    twist.angular.z = 90 + error;//将误差换算成角度并发布
    twist.linear.x = speed;//速度
    if(abs(error)<=10)
        twist.linear.x += 10;
    if(twist.angular.z > 180)
        twist.angular.z = 180;
    if(twist.angular.z < 0)
        twist.angular.z = 0;

    twist.angular.y = 0;
    twist.angular.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    pub_.publish(twist);
    ROS_INFO("error:%.2f",error);
}
    

int main(int argc, char *argv[])
{
    pid.Init();
    ros::init(argc,argv,"laser_go");
    PubAndSub PAS;
    ros::spin();
    return 0;
}
