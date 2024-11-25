//subscriber 0x62 data package

#include <ros/ros.h>
#include <iostream>
#include <unistd.h>
#include <serial_imu/Imu_0x62_msg.h>
#include "imu_data_decode.h"

void imu_0x62_callback(const serial_imu::Imu_0x62_msg imu_0x62_msg);

int main(int argc, char **argv)
{

	ros::init(argc, argv, "sub_0x62");
	
	ros::NodeHandle n;

	ros::Subscriber imu_0x62_sub = n.subscribe("/imu_0x62_package", 10,imu_0x62_callback);

	ros::spin();

}

void imu_0x62_callback(const serial_imu::Imu_0x62_msg imu_0x62_msg)
{
	int i = 0;


	printf("\033c");
	
	printf(" Device GWID: %6d\n", imu_0x62_msg.gw_id);
	printf(" Device number:%6d\n", imu_0x62_msg.node_num);
	if(imu_0x62_msg.node_num == 1)
	{
		i = 2;
		goto ONE_NODE;	
	}

	for (i = 0; i < imu_0x62_msg.node_num; i += 2)
	{
		putchar(10);
#if 0
		if(imu_0x62_msg.node_data[i].bitmap & BIT_VALID_ID)
			printf("     Devie ID:%6d\n",imu_0x62_msg.node_data[i].id);

		if(imu_0x62_msg.node_data[i].bitmap & BIT_VALID_PRS)
			printf("     Prs(hPa): %6f\n", imu_0x62_msg.node_data[i].prs);


		if(imu_0x62_msg.node_data[i].bitmap & BIT_VALID_TIME)
			printf("    Run times: %d days  %d:%d:%d:%d\n",imu_0x62_msg.node_data[i].time / 86400000, imu_0x62_msg.node_data[i].time / 3600000 % 24, imu_0x62_msg.node_data[i].time / 60000 % 60, imu_0x62_msg.node_data[i].time / 1000 % 60, imu_0x62_msg.node_data[i].time % 1000);

		printf("   Frame Rate:  %4dHz\r\n", imu_0x62_msg.node_data[i].frame_rate);
		if(imu_0x62_msg.node_data[i].bitmap & BIT_VALID_ACC)
			printf("       Acc(G):%8.3f %8.3f %8.3f\r\n", imu_0x62_msg.node_data[i].acc_x, imu_0x62_msg.node_data[i].acc_y, imu_0x62_msg.node_data[i].acc_z);

		if(imu_0x62_msg.node_data[i].bitmap & BIT_VALID_GYR)
			printf("   Gyr(deg/s):%8.2f %8.2f %8.2f\r\n", imu_0x62_msg.node_data[i].gyr_x, imu_0x62_msg.node_data[i].gyr_y, imu_0x62_msg.node_data[i].gyr_z);

		if(imu_0x62_msg.node_data[i].bitmap & BIT_VALID_MAG) 
			printf("      Mag(uT):%8.2f %8.2f %8.2f\r\n", imu_0x62_msg.node_data[i].mag_x, imu_0x62_msg.node_data[i].mag_y, imu_0x62_msg.node_data[i].mag_z);

		if(imu_0x62_msg.node_data[i].bitmap & BIT_VALID_EUL)
			printf("   Eul(R P Y):%8.2f %8.2f %8.2f\r\n", imu_0x62_msg.node_data[i].eul_r, imu_0x62_msg.node_data[i].eul_p, imu_0x62_msg.node_data[i].eul_y);

		if(imu_0x62_msg.node_data[i].bitmap & BIT_VALID_QUAT)
			printf("Quat(W X Y Z):%8.3f %8.3f %8.3f %8.3f\r\n", imu_0x62_msg.node_data[i].quat_w, imu_0x62_msg.node_data[i].quat_x, imu_0x62_msg.node_data[i].quat_y, imu_0x62_msg.node_data[i].quat_z);
#endif
	}

ONE_NODE:
	if(i - 1 == imu_0x62_msg.node_num)
	{
		putchar(10);
		i -= 2;
#if 1 

		if(imu_0x62_msg.node_data[i].bitmap & BIT_VALID_ID)
			printf("     Devie ID:%6d\n",imu_0x62_msg.node_data[i].id);
		if(imu_0x62_msg.node_data[i].bitmap & BIT_VALID_PRS)
			printf("     Prs(hPa): %6f\n", imu_0x62_msg.node_data[i].prs);


		if(imu_0x62_msg.node_data[i].bitmap & BIT_VALID_TIME)
			printf("    Run times: %d days  %d:%d:%d:%d\n",imu_0x62_msg.node_data[i].time / 86400000, imu_0x62_msg.node_data[i].time / 3600000 % 24, imu_0x62_msg.node_data[i].time / 60000 % 60, imu_0x62_msg.node_data[i].time / 1000 % 60, imu_0x62_msg.node_data[i].time % 1000);

		printf("   Frame Rate:  %4dHz\r\n", imu_0x62_msg.node_data[i].frame_rate);
		if(imu_0x62_msg.node_data[i].bitmap & BIT_VALID_ACC)
			printf("       Acc(G):%8.3f %8.3f %8.3f\r\n", imu_0x62_msg.node_data[i].acc_x, imu_0x62_msg.node_data[i].acc_y, imu_0x62_msg.node_data[i].acc_z);

		if(imu_0x62_msg.node_data[i].bitmap & BIT_VALID_GYR)
			printf("   Gyr(deg/s):%8.2f %8.2f %8.2f\r\n", imu_0x62_msg.node_data[i].gyr_x, imu_0x62_msg.node_data[i].gyr_y, imu_0x62_msg.node_data[i].gyr_z);

		if(imu_0x62_msg.node_data[i].bitmap & BIT_VALID_MAG) 
			printf("      Mag(uT):%8.2f %8.2f %8.2f\r\n", imu_0x62_msg.node_data[i].mag_x, imu_0x62_msg.node_data[i].mag_y, imu_0x62_msg.node_data[i].mag_z);

		if(imu_0x62_msg.node_data[i].bitmap & BIT_VALID_EUL)
			printf("   Eul(R P Y):%8.2f %8.2f %8.2f\r\n", imu_0x62_msg.node_data[i].eul_r, imu_0x62_msg.node_data[i].eul_p, imu_0x62_msg.node_data[i].eul_y);

		if(imu_0x62_msg.node_data[i].bitmap & BIT_VALID_QUAT)
			printf("Quat(W X Y Z):%8.3f %8.3f %8.3f %8.3f\r\n", imu_0x62_msg.node_data[i].quat_w, imu_0x62_msg.node_data[i].quat_x, imu_0x62_msg.node_data[i].quat_y, imu_0x62_msg.node_data[i].quat_z);

#endif
	}

}
