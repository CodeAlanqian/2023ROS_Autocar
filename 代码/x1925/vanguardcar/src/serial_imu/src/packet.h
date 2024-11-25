#ifndef __PACKET_H__
#define __PACKET_H__

#include <stdint.h>
#include <stdbool.h>

#define MAX_PACKET_LEN          (2048)
extern uint32_t frame_count;
extern uint32_t crc_error_count;

typedef struct
{
    uint32_t ofs;
    uint8_t buf[MAX_PACKET_LEN];    /* total frame buffer */
    uint16_t payload_len;           
    uint16_t len;                   /* total frame len */
    uint8_t type;
}packet_t;


/* packet Rx API */
typedef void (*on_data_received_event)(packet_t *pkt);
void packet_decode_init(packet_t *pkt, on_data_received_event rx_handler);
uint32_t packet_decode(uint8_t c);


#endif

