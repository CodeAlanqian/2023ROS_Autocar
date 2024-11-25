#include <string.h>
#include <stdio.h>

#include "packet.h"


#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))
#endif

#ifndef CH_OK
#define CH_OK   (0)
#endif

#ifndef CH_ERR
#define CH_ERR  (1)
#endif

uint32_t crc_error_count;

uint32_t frame_count;

static void crc16_update(uint16_t *currect_crc, const uint8_t *src, uint32_t len)
{
    uint32_t crc = *currect_crc;
    uint32_t j;
    for (j=0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    } 
    *currect_crc = crc;
}


enum status
{
    kStatus_Idle,
    kStatus_Cmd,
    kStatus_LenLow,
    kStatus_LenHigh,
    kStatus_CRCLow,
    kStatus_CRCHigh,
    kStatus_Data,
};

/* function pointer */
static on_data_received_event event_handler;
static packet_t *RxPkt;


 /**
 * @brief  初始化姿态解码模块
 * @note   完成初始化一个引脚配置
 * @param  pkt 接收包指针
 * @param  接收成功回调函数
 * @code

 *      void OnDataReceived(Packet_t *pkt)
 *      {
 *          pkt->buf 为数据 pkt->payload_len 为接收到的字节长度 
 *      }
 *
 *      Packet_t pkt;
 *      Packet_DecodeInit(&pkt, OnDataReceived);
 * @endcode
 * @retval None
 */
void packet_decode_init(packet_t *pkt, on_data_received_event func)
{
    event_handler = func;
    memset(pkt, 0, sizeof(packet_t));
    RxPkt = pkt;
}

 /**
 * @brief  接收IMU数据
 * @note   在串口接收中断中调用此函数
 * @param  c 串口数据
 * @retval CH_OK
 */


uint32_t packet_decode(uint8_t c)
{
    static uint16_t CRCReceived = 0;            /* CRC value received from a frame */
    static uint16_t CRCCalculated = 0;          /* CRC value caluated from a frame */
    static uint8_t status = kStatus_Idle;       /* state machine */
    static uint8_t crc_header[4] = {0x5A, 0xA5, 0x00, 0x00};

    switch(status)
    {
        case kStatus_Idle:
            if(c == 0x5A)
				status = kStatus_Cmd;
			break;

        case kStatus_Cmd:
            RxPkt->type = c;
			if(RxPkt->type == 0xA5)
				status = kStatus_LenLow;
            break;

        case kStatus_LenLow:
            RxPkt->payload_len = c;
            crc_header[2] = c;
            status = kStatus_LenHigh;
            break;

        case kStatus_LenHigh:
            RxPkt->payload_len |= (c<<8);
            crc_header[3] = c;
            status = kStatus_CRCLow;
            break;

        case kStatus_CRCLow:
            CRCReceived = c;
            status = kStatus_CRCHigh;
            break;

        case kStatus_CRCHigh:
            CRCReceived |= (c<<8);
            RxPkt->ofs = 0;
            CRCCalculated = 0;
            status = kStatus_Data;
            break;

        case kStatus_Data:
	
            RxPkt->buf[RxPkt->ofs++] = c;
		
            if(RxPkt->type == 0xA7 && RxPkt->ofs >= 8)
            {
                RxPkt->payload_len = 8;
                event_handler(RxPkt);
                status = kStatus_Idle;
            }
            if(RxPkt->ofs >= MAX_PACKET_LEN)
            {
                status = kStatus_Idle;
                return CH_ERR;   
            }

            if(RxPkt->ofs >= RxPkt->payload_len && RxPkt->type == 0xA5)
            {
                /* calculate CRC */
                crc16_update(&CRCCalculated, crc_header, 4);
                crc16_update(&CRCCalculated, RxPkt->buf, RxPkt->ofs);
                
                /* CRC match */
                if(CRCCalculated == CRCReceived)
                {
					frame_count++;
                    event_handler(RxPkt);
                }
				else
					crc_error_count++;

                status = kStatus_Idle;
            }
            break;
			
        default:
            status = kStatus_Idle;
            break;
    }
    return CH_OK;
}

