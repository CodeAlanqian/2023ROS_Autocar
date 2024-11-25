#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include "Control.h"

#define freq 1440   //每转一圈雷达扫描次数
#define speed 1800
#define pi_2 1.57

typedef struct      //极坐标系下的点
{   
    double range;
    double theta;
}Point_polar;

typedef struct      //直角坐标系下的点
{
    double x;
    double y;
}Point_rectangular;

Control::PID pid;   //PID控制器

void cal_point(double theta,double range,double &tmp_x,double & tmp_y) //极坐标转化为直角坐标
{
     tmp_x = range*sin(theta);
     tmp_y = range*cos(theta);

}

class PubAndSub
{
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
public:
    PubAndSub()
    {
        pub_ = n_.advertise<geometry_msgs::Twist>("car/cmd_vel",5);     
        sub_ = n_.subscribe("/scan",5,&PubAndSub::callback,this);
    }
    void callback(const sensor_msgs::LaserScan::ConstPtr &laser);
};

static float low_pass_val;

void PubAndSub::callback(const sensor_msgs::LaserScan::ConstPtr &laser)
{
    int i=0, j=0, k=0;
    double range = 0,error = 0,theta = 0, tmp_x = 0, tmp_y = 0, angle = 0;
    geometry_msgs::Twist twist;
    Point_rectangular red_p[30] = {0,0};    //红色锥桶数组
    Point_rectangular blue_p[30] = {0,0};   //蓝色锥桶数组
    Point_rectangular* p=red_p;             //用于计算误差时对准红蓝锥桶
    for (i = 1;i < freq;i++)
    {
        if(laser->ranges[i-1] - laser->ranges[i] >= 2.0)
         {
            theta = i * laser->angle_increment + laser->angle_min;
            cal_point(theta, laser->ranges[i], tmp_x, tmp_y);
            if((tmp_x <=1.5&&tmp_x>=-2) && tmp_y < 2&& tmp_y>-0.1 && tmp_y!= 0 )  
            {
                
                    if(tmp_x>0)                  
                    {
                      if(laser->ranges[i]<2)
                     {                          
                      red_p[j].x = tmp_x;
                      red_p[j].y = tmp_y;
                      j++;      //红桶数组元素个数
                     }
                    }
                    else
                    {
                     blue_p[k].x = tmp_x;
                     blue_p[k].y = tmp_y;
                     k++;       //蓝桶数组元素个数
                    }
            }
            
        } 
    }

    if(k >= j)  //初筛分组蓝桶较多
    {
        int m = 100;        
        for(int i = 1; i < k; i++)
        {
            if(sqrt(pow((blue_p[i-1].x-blue_p[i].x),2)+pow((blue_p[i-1].y-blue_p[i].y),2)) > 1.4)	
            {
                //距离超过1.4米
                m = i;	
                break;
            }
        }
        if(m != 100)	//存在误分组情况
        {
            for(int i = j - 1; i >= 0; i--)     
            {
                red_p[i + k - m] = red_p[i];	
            }
            for(int i = 0; i < k - m; i++) 
            {
                red_p[i] = blue_p[m + i];
            }
            j += (k - m);	
            k = m;		
        }
    }   

  
    else  //初筛分组红桶较多
    {
        int m = 100;       
        for(int i = j-2; i >= 0; i--)
        {
            if(sqrt(pow((red_p[i].x-red_p[i+1].x),2)+pow((red_p[i].y-red_p[i+1].y),2)) > 1.4)	
            {
                m = i;	
            }
        }
        if(m != 100)	//存在误分组情况
        {
            for(int i = 0; i <  m + 1;  i++)
            {
                blue_p[k + i] = red_p[i];
            }		

            for(int i = 0; i < j - m - 1;  i++) 
            {
                red_p[i] = red_p[i + m + 1];
            }
            j -= (m + 1);	//更新后红桶个数
            k += (m + 1);	//更新后蓝桶个数
        }
    }

    //取等量红蓝桶用于计算误差
    if (j>k)   
    {
        for(i=0;i<(j-k);i++)
            p+=1;             
        j=k;

    }
    else
    {
       k=j;
    }
    
    //误差累计与PID输出
    double gamma=0.8;   
    for(i = k-1;i >=0;i--)
    {
       error=gamma*error+((blue_p[i].x+p->x)); 
       p+=1;       
    }
    angle = pid.PIDPositional(error*10);
    float temp = angle; 
    twist.angular.z = 85 + angle;
    twist.linear.x = speed;
    if(abs(angle)<=10)
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
}
    

int main(int argc, char *argv[])
{
    pid.Init();
    ros::init(argc,argv,"laser_go");
    PubAndSub PAS;
    ros::spin();
    return 0;
}



