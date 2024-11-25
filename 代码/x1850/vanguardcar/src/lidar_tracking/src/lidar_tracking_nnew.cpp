#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include "Control.h"
#include <string>
#define freq 1440
#define pi_2 1.57

int speed = 1500;       //???
// double last_pidoutput = 0;
//??????????????????�????????????????????????????????
double rate_p[10] = { 0.8, 2.2, 0.8, 0, 0, 0, 0, 0, 0, 0 };
double max_right_dis = 1.5;
double max_left_dis = 1.5;

int middle_angle = 90;


int bucket_threshold = 5;
int vaild_bucket_threshold = 2;

typedef struct 
{   
    double range;
    double theta;
}Point_polar;

typedef struct
{
    double x;
    double y;
}Point_rectangular;
Control::PID pid;

void cal_point(double theta,double range,double &tmp_x,double & tmp_y)
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
    geometry_msgs::Twist twist;
    std::string topic;

public:

    PubAndSub()
    {
        std::string ttopic = "laser_control";
        ros::NodeHandle pn("~");
        pn.param("speed", speed, 1500);
        pn.param("kp", pid.kp, 0.88);
        pn.param("ki", pid.ki, 0.0);
        pn.param("kd", pid.kd, 1.2);
        pn.param("max_right_dis", max_right_dis, 1.5);
        pn.param("max_left_dis", max_left_dis, 1.5);
        pn.param("topic", topic, ttopic);

        pn.param("rate1", rate_p[0], 0.8);
        pn.param("rate2", rate_p[1], 2.2);
        pn.param("rate3", rate_p[2], 0.8);
        pn.param("middle_angle", middle_angle, 90);

        pn.param("bucket_threshold", bucket_threshold, 5);
        pn.param("vaild_bucket_threshold", vaild_bucket_threshold, 2);


        
        // pub_ = n_.advertise<geometry_msgs::Twist>("laser_control",5);
        // pub_ = n_.advertise<geometry_msgs::Twist>("car/cmd_vel",5);
        pub_ = n_.advertise<geometry_msgs::Twist>(topic,5);
        sub_ = n_.subscribe("/scan",5,&PubAndSub::callback,this);

    }
    void callback(const sensor_msgs::LaserScan::ConstPtr &laser);
};

void PubAndSub::callback(const sensor_msgs::LaserScan::ConstPtr &laser)
{
    int i=0,j=0,k=0,flag_jk=0;
    double range = 0,error = 0,theta=0,tmp_x=0,tmp_y=0,angle=0,rate_sum = 0,error_sum = 0;
    Point_rectangular red_p[30] = {0,0}; 
    Point_rectangular blue_p[30] = {0,0}; 
    /*int flag_jk=0;*/
    for (i = 1;i < freq - bucket_threshold;i++)
    {
        if (laser->ranges[i - 1] - laser->ranges[i] >= 2.0 && laser->ranges[i] < 2.0 && laser->ranges[i] != 0 && laser->ranges[i - 1] - laser->ranges[i+1] >= 2.0)
        {
            int continue_ranges = 0;
            int continue_dectect = 0;
            //距离突然减小，并且较小距离小于2米且不为0
            for(int idx = 1; idx < bucket_threshold; idx++)   //10为连续检测的数据个数，可调整；
            {
                if(abs(laser->ranges[i] - laser->ranges[i+idx]) < 0.2)
                {
                    continue_ranges++;
                    if(continue_ranges >= vaild_bucket_threshold)    //6为连续检测的阈值，可调整；
                    {
                        continue_dectect = 1;
                        break;
                    }
                }
            }
            if(continue_dectect == 1)
            {
                theta=i*laser->angle_increment + laser->angle_min;
                cal_point(theta,laser->ranges[i],tmp_x,tmp_y);
                
                if((tmp_x <=max_left_dis && tmp_x>=-max_right_dis) && tmp_y < 2&& tmp_y>-0.2 && tmp_y!= 0 )  
                {
                    
                        if(tmp_x>0)                  
                        {
                        if(laser->ranges[i]<2)
                        {                          
                        red_p[j].x = tmp_x;
                        red_p[j].y = tmp_y;
                          ROS_INFO("_____found red bucket point:(%.2f,%.2f)\n",red_p[j].x,red_p[j].y);
                        j++;
                        }
                        }
                        else
                        {
                        blue_p[k].x = tmp_x;
                        blue_p[k].y = tmp_y;
                         ROS_INFO("______found blue bucket point:(%.2f,%.2f)\n",blue_p[k].x,blue_p[k].y);
                        k++;
                        }
                }
            }
        } 
    }
    // ROS_INFO("j=%d,k=%d\n",j,k);
    // if(k+j>6)
    // {
    //     // ROS_INFO("1");
    //     if (j - k >= 2)
    //     {
    //         flag_jk = 1;
    //     }
    //     else if(k - j >= 2)
    //     {
    //         flag_jk = 2;
    //     }
    // }
    
    ROS_INFO("j=%d,k=%d\n",j,k);

    if(k >= j) 
    {
        int m = 100;
        for(int i = 1; i < k; i++)
        {
            if(sqrt(pow((blue_p[i-1].x-blue_p[i].x),2)+pow((blue_p[i-1].y-blue_p[i].y),2)) > 1.4)	
            {
                //ROS_INFO("1");
                m = i;	
                break;
            }
        }
        if(m != 100)	
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

  
    else
    {
        int m = 100;
        
        for(int i = j-2; i >= 0; i--)
        {
            if(sqrt(pow((red_p[i].x-red_p[i+1].x),2)+pow((red_p[i].y-red_p[i+1].y),2)) > 1.4)
            {
                m = i;	
            }
        }
        if(m != 100)	
        {
            for(int i = 0; i <  m + 1;  i++)
            {
                    blue_p[k + i] = red_p[i];
            }		

            for(int i = 0; i < j - m - 1;  i++) 
            {
                red_p[i] = red_p[i + m + 1];
            }
            j -= (m+1);	
            k += (m + 1);	
        }
        
    }
    // ROS_INFO("j=%d,k=%d\n",j,k);

    /*if (j>k)   
    {
        for(i=0;i<(j-k);i++)
            p+=1;             
        j=k;

    }
    else
    {
       k=j;
    }*/

    //锥桶数据标准化：等量且小于4个
    if(j>3)
    {
        //红桶大于3个，舍弃最远的也就是最小号元素
        for(int i = 0; i < 3; i++)
        {
            red_p[i] = red_p[i+1];
        }
        j = 3;  //调整后，红桶有3个
    }
    if(k>3)
    {
        //蓝桶大于3个，舍弃最远的也就是最大号元素
        k = 3;  //调整后，蓝桶有3个
    }
    if(j>k && k!=0)
    {
        //红桶比蓝桶多且蓝桶不为零的情况
        if(k == 1)
        {
            //蓝桶只有一个，那只用两对锥桶计算误差
            if(j > 2)
            {
                for(int i = 0; i < 2; i++)
                {
                    red_p[i] = red_p[i+1];
                    j = 2;
                }
            }
            blue_p[1] = blue_p[0];  //复制一份蓝桶
            k = 2;
        }
        else if(k == 2)
        {
            blue_p[2] = blue_p[1];
            k = 3;
        }
    }
    else if(k > j && j!= 0)
    {
        if(j == 1)
        {
            k = 2;
            red_p[1] = red_p[0];
            j = 2;
        }
        else if(j == 2)
        {
            red_p[2] = red_p[1];
            red_p[1] = red_p[0];
            j = 3;
        }
    }

    //???????????????????????????????
    Point_rectangular* point_to_red = &red_p[j-1];
    //???????????????????????????????????��?
    int error_count = j < k ? j : k;

    //??????
    if (error_count)
    {
        for (i = 0; i < error_count; i++)   
        {
            //??????error_count?��?????????????
            error_sum += (blue_p[i].x + point_to_red->x) * rate_p[i];   //?????+=??????*??????
            rate_sum += rate_p[i];                                      //?????+=??????
            point_to_red--;                                             //?????????????????????
        }
        //??? = ?????/????? * ???????
        error = error_sum / rate_sum * 53.33;
    }
    //????PID????
    angle  = pid.PIDPositional(error);
    // last_pidoutput = angle;
    // switch(flag_jk)
    // {
    //     case 1: angle += 8; //ROS_INFO("1");
    //             break;
    //     case 2: angle-= 8; //ROS_INFO("2");
    //             break;
    //     default: break;
    // }
    //?????? = ?????? + PID????
    twist.angular.z = middle_angle + angle;
    // if(j + k <= 2)
    // {
    //     twist.angular.z = last_pidoutput;
    // }
    twist.linear.x = speed;

    if (abs(angle) >= 20)
    {
        twist.linear.x -= 25;
    }
        
    if (twist.angular.z > 180)
        twist.angular.z = 180;
    if (twist.angular.z < 0)
        twist.angular.z = 0;
    pub_.publish(twist);
    // ROS_INFO("angle:%.2f",angle);
}
    

int main(int argc, char *argv[])
{
    pid.Init();
    ros::init(argc,argv,"laser_go");
    PubAndSub PAS;
    ros::spin();
    return 0;
}
