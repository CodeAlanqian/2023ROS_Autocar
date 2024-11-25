#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

#define freq 1440//每转一圈雷达扫描次数
#define speed 1825

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
public:
    PubAndSub()
    {
        pub_ = n_.advertise<geometry_msgs::Twist>("/car/cmd_vel",5);
        sub_ = n_.subscribe("/scan",5,&PubAndSub::callback,this);
    }
    void callback(const sensor_msgs::LaserScan::ConstPtr &laser);
};


void PubAndSub::callback(const sensor_msgs::LaserScan::ConstPtr &laser)
{
    int i,j=0;
    geometry_msgs::Twist twist;
    Point_polar pp[30] = {0,0};
    Point_rectangular pr[30] = {0,0};

    for (i = 1;i < freq;i++)
    {
        if(laser->ranges[i-1] - laser->ranges[i] >= 2.0)
            if(laser->ranges[i] < 2.5)
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
            ROS_INFO("found bucket point:(%.2f,%.2f)\n",pr[i].x,pr[i].y);
        }
    }
}
    

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"laser_go");
    PubAndSub PAS;
    ros::spin();
    return 0;
}
