#include <string.h>
#include <stdio.h>

#include "packet.h"
#include "imu_data_decode.h"

static packet_t RxPkt; /* used for data receive */
/*
 **采用结构体来保存数据
 **将标志位都集中到一个32位的变量上，用位来表示
 **在复制数据时，在用户程序中直接调用一个memcpy函数
 */

uint8_t bitmap;
id0x91_t id0x91;  /* HI226 HI229 CH100 CH110 HI221 protocol packet */
id0x62_t id0x62;  /* HI221 Dongle protocol packet */

static int stream2int16(int *dest,uint8_t *src)
{
	dest[0] = (int16_t)(src[0] | src[1] << 8);
	dest[1] = (int16_t)(src[2] | src[3] << 8);
	dest[2] = (int16_t)(src[4] | src[5] << 8);	
	return 0;
}   


/*  callback function of  when recv a data frame successfully */
static void on_data_received(packet_t *pkt)
{
	int temp[3] = {0};
	int offset = 0;
	uint8_t *p = pkt->buf;

	if(pkt->type != 0xA5)
	{
		return;
	}

	bitmap = 0;
	while(offset < pkt->payload_len)
	{
		switch(p[offset])
		{
		case kItemID:
			bitmap |= BIT_VALID_ID;
			id0x91.id = p[1];
			offset += 2;
			break;

		case kItemAccRaw:
			bitmap |= BIT_VALID_ACC;
			stream2int16(temp, p + offset + 1);
			id0x91.acc[0] = (float)temp[0] / 1000;
			id0x91.acc[1] = (float)temp[1] / 1000;
			id0x91.acc[2] = (float)temp[2] / 1000;
			offset += 7;
			break;

		case kItemGyrRaw:
		case kItemGyrRaw_yunjing:
			bitmap |= BIT_VALID_GYR;
			stream2int16(temp, p + offset + 1);
			id0x91.gyr[0] = (float)temp[0] / 10;
			id0x91.gyr[1] = (float)temp[1] / 10;
			id0x91.gyr[2] = (float)temp[2] / 10;
			offset += 7;
			break;

		case kItemMagRaw:
			bitmap |= BIT_VALID_MAG;
			stream2int16(temp, p + offset + 1);
			id0x91.mag[0] = (float)temp[0] / 10;
			id0x91.mag[1] = (float)temp[1] / 10;
			id0x91.mag[2] = (float)temp[2] / 10;
			offset += 7;
			break;

		case kItemRotationEul:
			bitmap |= BIT_VALID_EUL;
			stream2int16(temp, p + offset + 1);
			id0x91.eul[1] = (float)temp[0] / 100;
			id0x91.eul[0] = (float)temp[1] / 100;
			id0x91.eul[2] = (float)temp[2] / 10;
			offset += 7;
			break;

		case kItemRotationQuat:
			bitmap |= BIT_VALID_QUAT;
			memcpy(id0x91.quat, p + offset + 1, sizeof( id0x91.quat));
			offset += 17;
			break;

		case kItemPressure:
			offset += 5;
			break; 

		case KItemIMUSOL:
			bitmap = BIT_VALID_ALL;
			memcpy((void *)&id0x91, p, sizeof(id0x91_t));
			offset += sizeof(id0x91_t);
			break;

		case KItemGWSOL:
			memcpy((void *)&id0x62, p, 8);
			offset += 8;
			for (int i = 0; i < id0x62.n; i++)
			{
				bitmap = BIT_VALID_ALL;
				bitmap &= ~BIT_VALID_TIME;
 				memcpy((void *)&id0x62.id0x91[i], p + offset, sizeof(id0x91_t));

				offset += sizeof(id0x91_t);
			}
			break;

		default:
			/* offset ==> 0 2 9 16 23 30 47 52 76  */
			offset++;	
		}
	}
}


int imu_data_decode_init(void)
{
    packet_decode_init(&RxPkt, on_data_received);
    return 0;
}



