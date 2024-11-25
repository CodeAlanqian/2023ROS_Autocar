#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include "Control.h"

#define freq 1440
#define pi_2 1.57
int speed=1880;
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

void cal_point(double theta, double range, double& tmp_x, double& tmp_y)
{
    tmp_x = range * sin(theta);
    tmp_y = range * cos(theta);
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
            // pub_ = n_.advertise<geometry_msgs::Twist>("laser_control", 5);
            sub_ = n_.subscribe("/scan", 5, &PubAndSub::callback, this);
            // n_.param("speed", speed, 1800);
        }
        void callback(const sensor_msgs::LaserScan::ConstPtr& laser);
};

static float low_pass_val;

void PubAndSub::callback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
    char negetiveNum = 0, positiveNum = 0;
    int i = 0, j = 0, k = 0;
    double range = 0, error = 0, negetiveSum = 0, positiveSum = 0, theta = 0, tmp_x = 0, tmp_y = 0, angle = 0;
    int flag_jk = 0;
    geometry_msgs::Twist twist;
    Point_rectangular red_p[30] = { 0,0 };
    Point_rectangular blue_p[30] = { 0,0 };
    Point_rectangular* p = red_p;
    ROS_INFO("start:-------------------------------------\n");
    for (i = 1;i < freq /3; i++)
    {
        // if (laser->ranges[i - 1] - laser->ranges[i] >= 2.0 && laser->ranges[i] < 2.0 && laser->ranges[i] != 0 && laser->ranges[i - 1] - laser->ranges[i+1] >= 2.0)
        // {
        //     int continue_ranges = 0;
        //     int continue_dectect = 0;
        //     //距离突然减小，并且较小距离小于2米且不为0
        //     for(int idx = 1; idx < 10; idx++)   //10为连续检测的数据个数，可调整；
        //     {
        //         if(abs(laser->ranges[i] - laser->ranges[i+idx]) < 0.2)
        //         {
        //             continue_ranges++;
        //             if(continue_ranges >= 6)    //6为连续检测的阈值，可调整；
        //             {
        //                 continue_dectect = 1;
        //                 break;
        //             }
        //         }
        //     }
        //     if(continue_dectect == 1)
        //     {
        //         theta = (i+2) * laser->angle_increment + laser->angle_min;
        //         cal_point(theta, laser->ranges[i+2], tmp_x, tmp_y);
                
        //         if ((tmp_x <= 1.5 && tmp_x >= -1.5) && tmp_y < 2 && tmp_y>-0.2 && tmp_y != 0)
        //         {

        //             if (tmp_x > 0)
        //             {
        //                 if (laser->ranges[i] < 2)
        //                 {
        //                     red_p[j].x = tmp_x;
        //                     red_p[j].y = tmp_y;
        //                       ROS_INFO("found red bucket point:(%.2f,%.2f)\n",red_p[j].x,red_p[j].y);
        //                     j++;
        //                 }
        //             }
        //             else
        //             {
        //                 blue_p[k].x = tmp_x;
        //                 blue_p[k].y = tmp_y;
        //                  ROS_INFO("found blue bucket point:(%.2f,%.2f)\n",blue_p[k].x,blue_p[k].y);
        //                 k++;
        //             }
        //         }
        //     }
            ROS_INFO("%.2f",laser->ranges[i]);
        //}
        
    }
    // ROS_INFO("j=%d,k=%d\n",j,k);
    ROS_INFO("end:-------------------------------------\n");
    /*
    if (k >= j)
    {
        int m = 100;
        for (int i = 1; i < k; i++)
        {
            if (sqrt(pow((blue_p[i - 1].x - blue_p[i].x), 2) + pow((blue_p[i - 1].y - blue_p[i].y), 2)) > 1.4)
            {
                //ROS_INFO("1");
                m = i;
                //if (k - j >= 2)
                //{
                    flag_jk = 1;    //车身朝向左边界
                //}
                break;
            }
        }
        if (m != 100)
        {
            for (int i = j - 1; i >= 0; i--)
            {
                red_p[i + k - m] = red_p[i];
            }
            for (int i = 0; i < k - m; i++)
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

        for (int i = j - 2; i >= 0; i--)
        {
            if (sqrt(pow((red_p[i].x - red_p[i + 1].x), 2) + pow((red_p[i].y - red_p[i + 1].y), 2)) > 1.4)
            {
                m = i;
                //if (j - k >= 2)
                //{
                     flag_jk = 2;    //车身朝向右边界
                //}
                break;
            }
        }
        if (m != 100)
        {
            for (int i = 0; i < m + 1; i++)
            {
                blue_p[k + i] = red_p[i];
            }

            for (int i = 0; i < j - m - 1; i++)
            {
                red_p[i] = red_p[i + m + 1];
            }
            j -= (m + 1);
            k += (m + 1);
        }

    }
    // ROS_INFO("j=%d,k=%d\n",j,k);

    if (j > k)
    {
        for (i = 0; i < (j - k); i++)
            p += 1;
        j = k;

    }
    else
    {
        k = j;
    }

    double gamma = 0.8;

    //ROS_INFO("j=%d,k=%d\n",j,k);

    for (i = k - 1; i >= 0; i--)
    {
        error = gamma * error + ((blue_p[i].x + p->x));

        //    ROS_INFO("blue bucket point:(%.2f,%.2f)\n",blue_p[i].x,blue_p[i].y);
        //    ROS_INFO("red bucket point:(%.2f,%.2f)\n",p->x,p->y);
        //    ROS_INFO("error:%.2f\n",((blue_p[i].x+p->x)));
        p += 1;

    }
    */

    //error = pid.PIDIncremental(error);
    angle = pid.PIDPositional(error * 10);

    float temp = angle;
    // angle = 0.6*angle + 0.4*low_pass_val;
    low_pass_val = angle;
    //error *= 25.0;
    twist.angular.z = 85 + angle;
    twist.linear.x = speed;//???
    if (abs(angle) <= 10)
        twist.linear.x += 10;
    if (twist.angular.z > 180)
        twist.angular.z = 180;
    if (twist.angular.z < 0)
        twist.angular.z = 0;

   

    pub_.publish(twist);
    // ROS_INFO("angle:%.2f",angle);
}


int main(int argc, char* argv[])
{

    pid.Init();
    ros::init(argc, argv, "laser_go");
    PubAndSub PAS;
    ros::spin();
    return 0;
}