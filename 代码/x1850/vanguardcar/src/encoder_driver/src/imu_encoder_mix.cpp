#include "ros/ros.h"
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

#define PI 3.14159
using namespace std;
using namespace message_filters;
using namespace sensor_msgs;
ros::Time current_time,last_time;
double x = 0.0;
double y = 0.0;
double s = 0.0;
double th = 0.0;
double vth = 0.0;
double th_init = 0.0;
static double theta_first= 0.0;
static bool flag = true;
ros::Publisher encoder_imu_pub;


//从车身姿态获取航向角
double getYawFromPose(const sensor_msgs::Imu::ConstPtr &carPose) {
  //odom=*odom_cb;
  //geometry_msgs::Pose carPose = odom.pose.pose;
  sensor_msgs::Imu imu=(*carPose);
  float x = imu.orientation.x;
  float y = imu.orientation.y;
  float z = imu.orientation.z;
  float w = imu.orientation.w;

  double tmp, yaw;
  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 quaternion(q);
  quaternion.getRPY(tmp, tmp, yaw);

  return yaw;
}
double getrollFromPose(const sensor_msgs::Imu::ConstPtr &carPose) {
  //odom=*odom_cb;
  //geometry_msgs::Pose carPose = odom.pose.pose;
  sensor_msgs::Imu imu=(*carPose);
  float x = imu.orientation.x;
  float y = imu.orientation.y;
  float z = imu.orientation.z;
  float w = imu.orientation.w;

  double roll,pitch, yaw;
  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 quaternion(q);
  quaternion.getRPY(roll, pitch, yaw);

  return roll;
}
double getpitchFromPose(const sensor_msgs::Imu::ConstPtr &carPose) {
  //odom=*odom_cb;
  //geometry_msgs::Pose carPose = odom.pose.pose;
  sensor_msgs::Imu imu=(*carPose);
  float x = imu.orientation.x;
  float y = imu.orientation.y;
  float z = imu.orientation.z;
  float w = imu.orientation.w;

  double roll,pitch, yaw;
  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 quaternion(q);
  quaternion.getRPY(roll, pitch, yaw);

  return pitch;
}
void callback(const sensor_msgs::Imu::ConstPtr& imu_data,const nav_msgs::Odometry::ConstPtr& speed)
{
        if(flag==true)
        {
      	theta_first=getYawFromPose(imu_data);
        flag=false;
        }

        if(flag==false)
        {
        double v=(*speed).twist.twist.linear.x;
        double theta=getYawFromPose(imu_data);
		double roll=getrollFromPose(imu_data);
		double pitch=getpitchFromPose(imu_data);
		theta = theta - theta_first;

        th_init = theta_first*180/PI;
        th = theta*180/PI;//转换成角度

        // if(th_init > 0) {if(th < -180) th += 360;}
        // if(th_init < 0) {if(th > 180 ) th -= 360;}
      	double vx=v*cos(th/180*PI);
      	double vy=v*sin(th/180*PI);
      	current_time=ros::Time::now();
      	double dt=(current_time-last_time).toSec();
        double delta_x = vx* dt;
        double delta_y = vy* dt;
        double delta_th = vth * dt;//0
		// ROS_INFO("v    :%.4f",v);
 		//ROS_INFO("theta:%.4f",theta*180/PI);
        // ROS_INFO("theta:%.4f",th);


      	x+=delta_x;
      	y+=delta_y;
		s = sqrt(x*x+y*y);
		// ROS_INFO("lenth:%.1f",s*100);
        //th += delta_th;
      	//定义odom发布需要的转换戳和位置

	    // tf::TransformBroadcaster odom_broadcaster;
	    // geometry_msgs::TransformStamped odom_trans;
      	// odom_trans.header.stamp=current_time;
      	// odom_trans.header.frame_id="odom";
      	// odom_trans.child_frame_id="base_footprint";

      	// odom_trans.transform.translation.x = x;
      	// odom_trans.transform.translation.y = y;
      	// odom_trans.transform.translation.z = 0.0;
      	// odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta);
      	// //将角度转换成旋转矩阵

      	// odom_broadcaster.sendTransform(odom_trans);

      	//然后发送里程计信息  发布者odom_pub
      	nav_msgs::Odometry odom;
     	odom.header.stamp=current_time;
      	odom.header.frame_id="odom";
     	odom.child_frame_id="base_footprint";
      	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,theta);

      	odom.pose.pose.position.x = x;
      	odom.pose.pose.position.y = y;
      	odom.pose.pose.position.z = 0.0;
      	odom.pose.pose.orientation = odom_quat;

     	odom.twist.twist.linear.x = vx;
      	odom.twist.twist.linear.y = vy;
      	odom.twist.twist.angular.z = 0;

      	encoder_imu_pub.publish(odom);

        last_time=current_time;
      }
       // ros::spinOnce();

}

int main(int argc,char** argv)
{
	//启动节点订阅消息
	ros::init(argc,argv,"encoder_imu_mix");
	ros::NodeHandle nh;
	message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh,"/imu_data",100);
	message_filters::Subscriber<nav_msgs::Odometry> encoder_sub(nh,"/encoder",20);
	ROS_INFO("-------");
	//创建发布后融合的消息的节点
	encoder_imu_pub = nh.advertise<nav_msgs::Odometry>("/encoder_imu_odom",10);
	//定义同步消息储存器  精确时间戳 大概时间戳
	//typedef sync_policies::ApproximateTime<nav_msgs::Odometry,nav_msgs::Odometry> MySyncPolicy;
	typedef sync_policies::ApproximateTime<sensor_msgs::Imu,nav_msgs::Odometry> MySyncPolicy;

	//typedef sync_policies::ExactTime<sensor_msgs::Imu,nav_msgs::Odometry> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),imu_sub,encoder_sub);
	sync.registerCallback(boost::bind(&callback,_1,_2));
	ros::spin();
	return 0;
}
	//回调函数的创建 callback

// biaoding 