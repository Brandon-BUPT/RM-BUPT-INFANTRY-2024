//
// Created by bismarckkk on 2024/2/17.
//

#include "ui_interface.h"
#include "CRC8_CRC16.h"
#include <string.h>

uint8_t seq = 0;

void print_message(const uint8_t *message, const int length) {
    for (int i = 0; i < length; i++) {
        printf("%02x ", message[i]);
    }
    printf("\n\n");
}



#define DEFINE_FRAME_PROC(num, id)                          \
void ui_proc_ ## num##_frame(ui_ ## num##_frame_t *msg) {   \
    msg->header.SOF = 0xA5;                                 \
    msg->header.length = 6 + 15 * num;                      \
    msg->header.seq = 0;                                \
    msg->header.crc8 = get_CRC8_check_sum((uint8_t*)msg,4,0xff);        \
    msg->header.cmd_id = 0x0301;                            \
    msg->header.sub_id = id;                                \
    msg->header.send_id = UI_SELF_ID;                       \
    msg->header.recv_id = UI_SELF_ID + 256;                 \
    msg->crc16 = get_CRC16_check_sum((uint8_t*)msg, 13 + 15 * num,0xffff); \
}

DEFINE_FRAME_PROC(1, 0x0101)
DEFINE_FRAME_PROC(2, 0x0102)
DEFINE_FRAME_PROC(5, 0x0103)
DEFINE_FRAME_PROC(7, 0x0104)

void ui_proc_string_frame(ui_string_frame_t *msg) {
    msg->header.SOF = 0xA5;
    msg->header.length = 51;
    msg->header.seq = 0;
    msg->header.crc8 = get_CRC8_check_sum((uint8_t *) msg, 4,0xff);
    msg->header.cmd_id = 0x0301;
    msg->header.sub_id = 0x0110;
    msg->header.send_id = UI_SELF_ID;
    msg->header.recv_id = UI_SELF_ID + 256;
    msg->option.str_length = strlen(msg->option.string);
    msg->crc16 = get_CRC16_check_sum((uint8_t *) msg, 58,0xffff);
}
