//serial_imu.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <stdio.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <sensor_msgs/Imu.h>
#include <serial_imu/Imu_0x91_msg.h>
#include <serial_imu/Imu_0x62_msg.h>
#include <serial_imu/Imu_data_package.h>
#include <signal.h>

#ifdef __cplusplus 
extern "C"{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "packet.h"
#include "imu_data_decode.h"

#define IMU_SERIAL   "/dev/imu"  //更改为imu
#define BAUD         (115200)
#define GRA_ACC      (9.8)
#define DEG_TO_RAD   (0.01745329)
#define BUF_SIZE     1024

int imu_data_decode_init(void);
typedef void (*on_data_received_event)(packet_t *ptr);
void packet_decode_init(packet_t *pkt, on_data_received_event rx_handler);
uint32_t packet_decode(uint8_t);
void publish_0x91_data(id0x91_t *data, serial_imu::Imu_0x91_msg *data_imu);
void publish_imu_data(id0x91_t *data, sensor_msgs::Imu *imu_data);
void publish_0x62_data(id0x62_t *data, serial_imu::Imu_0x62_msg *data_imu);


#ifdef __cplusplus
}
#endif

static int frame_rate;

static uint8_t buf[2048];

void timer(int sig)
{
	if(SIGALRM == sig)
	{
		frame_rate = frame_count;
		frame_count = 0;
		alarm(1);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "serial_imu");
	ros::NodeHandle n;

	ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("/imu_data", 20);
	ros::Publisher Imu_0x91_pub = n.advertise<serial_imu::Imu_0x91_msg>("/imu_0x91_package", 10);
	ros::Publisher Imu_0x62_pub = n.advertise<serial_imu::Imu_0x62_msg>("/imu_0x62_package", 10);

	serial::Serial sp;

	serial::Timeout to = serial::Timeout::simpleTimeout(100);

	sp.setPort(IMU_SERIAL);

	sp.setBaudrate(BAUD);

	sp.setTimeout(to);
	
	
	imu_data_decode_init();
	signal(SIGALRM,timer);

	try
	{
		sp.open();
	}
	catch(serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port.");
		return -1;
	}
    
	if(sp.isOpen())
	{
		ROS_INFO_STREAM("/dev/imu is opened.");
	}
	else
	{
		return -1;
	}
	
	alarm(1);
	
	ros::Rate loop_rate(500);
	sensor_msgs::Imu imu_data;
	serial_imu::Imu_0x91_msg imu_0x91_msg;
	serial_imu::Imu_0x62_msg imu_0x62_msg;

	while(ros::ok())
	{
		size_t num = sp.available();
		if(num!=0)
		{
			uint8_t buffer[BUF_SIZE]; 
	
			if(num > BUF_SIZE)
				num = BUF_SIZE;
			
			num = sp.read(buffer, num);
			if(num > 0)
			{
				for(int i = 0; i < num; i++)
					packet_decode(buffer[i]);
				
				if (bitmap & 0xff)
				{

					imu_data.header.stamp = ros::Time::now();
					imu_data.header.frame_id = "IMU_link";

					imu_0x91_msg.header.stamp = ros::Time::now();
					imu_0x91_msg.header.frame_id = "base_0x91_link";

					imu_0x62_msg.header.stamp = ros::Time::now();
					imu_0x62_msg.header.frame_id = "base_0x62_link";

					if(id0x62.tag != KItemGWSOL)
					{
						publish_0x91_data(&id0x91, &imu_0x91_msg);
						Imu_0x91_pub.publish(imu_0x91_msg);

						publish_imu_data(&id0x91, &imu_data);
						IMU_pub.publish(imu_data);
					}
					else
					{
						 
						publish_0x62_data(&id0x62, &imu_0x62_msg);
						
						Imu_0x62_pub.publish(imu_0x62_msg);

						 
					}
				}
			}
		}
		loop_rate.sleep();
	}
    
	sp.close();
 
	return 0;
}


//void publish_0x91_data(id0x91_t *data, serial_imu::Imu_0x91_msg *data_imu)
void memcpy_imu_data_package(id0x91_t *data, serial_imu::Imu_data_package *data_imu)
{
	data_imu->tag = data->tag;
	data_imu->bitmap = bitmap;
	if(bitmap & BIT_VALID_ID)
		data_imu->id = data->id;

	if(bitmap & BIT_VALID_TIME)
		data_imu->time = data->time;

	data_imu->frame_rate = frame_rate;

	//data_imu->frame_rate = crc_error_count;
	if(bitmap & BIT_VALID_ACC)
	{
		data_imu->acc_x = data->acc[0];
		data_imu->acc_y = data->acc[1];
		data_imu->acc_z = data->acc[2];
	}

	if(bitmap & BIT_VALID_GYR)
	{
		data_imu->gyr_x = data->gyr[0];
		data_imu->gyr_y = data->gyr[1];
		data_imu->gyr_z = data->gyr[2];
	}

	if(bitmap & BIT_VALID_MAG)
	{
		data_imu->mag_x = data->mag[0];
		data_imu->mag_y = data->mag[1];
		data_imu->mag_z = data->mag[2];
	}

	if(bitmap & BIT_VALID_EUL)
	{
		data_imu->eul_r = data->eul[0];
		data_imu->eul_p = data->eul[1];
		data_imu->eul_y = data->eul[2];
	}

	if(bitmap & BIT_VALID_QUAT)
	{
		data_imu->quat_w = data->quat[0];
		data_imu->quat_x = data->quat[1];
		data_imu->quat_y = data->quat[2];
		data_imu->quat_z = data->quat[3];
	}
}

void publish_imu_data(id0x91_t *data, sensor_msgs::Imu *imu_data)
{	
	imu_data->orientation.x = data->quat[1];
	imu_data->orientation.y = data->quat[2];
	imu_data->orientation.z = data->quat[3];
	imu_data->orientation.w = data->quat[0];
	imu_data->angular_velocity.x = data->gyr[0] * DEG_TO_RAD;
	imu_data->angular_velocity.y = data->gyr[1] * DEG_TO_RAD;
	imu_data->angular_velocity.z = data->gyr[2] * DEG_TO_RAD;
	imu_data->linear_acceleration.x = data->acc[0] * GRA_ACC;
	imu_data->linear_acceleration.y = data->acc[1] * GRA_ACC;
	imu_data->linear_acceleration.z = data->acc[2] * GRA_ACC;
}

void publish_0x91_data(id0x91_t *data, serial_imu::Imu_0x91_msg *data_imu)
{
	memcpy_imu_data_package(data,&(data_imu->imu_data));
}

void publish_0x62_data(id0x62_t *data, serial_imu::Imu_0x62_msg *data_imu)
{
	/*  */
	data_imu->tag = data->tag;
	data_imu->gw_id = data->gw_id;
	data_imu->node_num = data->n;

	for (int i = 0; i < data_imu->node_num; i++)
		memcpy_imu_data_package(&(data->id0x91[i]), &(data_imu->node_data[i]));
		
	
}
