// AUTO-GENERATED CODE: DO NOT EDIT!!!


#include "msppg.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static byte CRC8(byte * data, int n) {

    byte crc = 0x00;
    int k;

    for (k=0; k<n; ++k) {

        crc ^= data[k];
    }

    return crc;
}

byte msp_message_start(msp_message_t * msg) {

    msg->pos = 0;
    return msp_message_get_next(msg);
}

bool msp_message_has_next(msp_message_t * msg) {

    return msg->pos <= msg->len;
}

byte msp_message_get_next(msp_message_t * msg) {

    return msg->bytes[msg->pos++];
}

void msp_parser_init(msp_parser_t * parser) {

    parser->state = 0;
}

void msp_parser_parse(msp_parser_t * parser, byte b) {

    switch (parser->state) {

        case 0:               // sync char 1
            if (b == 36) { // $
                parser->state++;
            }
            break;        

        case 1:               // sync char 2
            if (b == 77) { // M
                parser->state++;
            }
            else {            // restart and try again
                parser->state = 0;
            }
            break;

        case 2:               // direction (should be >)
            if (b == 62) { // >
                parser->message_direction = 1;
            }
            else {            // <
                parser->message_direction = 0;
            }
            parser->state++;
            break;

        case 3:
            parser->message_length_expected = b;
            parser->message_checksum = b;
            // setup arraybuffer
            parser->message_length_received = 0;
            parser->state++;
            break;

        case 4:
            parser->message_id = b;
            parser->message_checksum ^= b;
            if (parser->message_length_expected > 0) {
                // process payload
                parser->state++;
            }
            else {
                // no payload
                parser->state += 2;
            }
            break;

        case 5: // payload
            parser->message_buffer[parser->message_length_received] = b;
            parser->message_checksum ^= b;
            parser->message_length_received++;
            if (parser->message_length_received >= parser->message_length_expected) {
                parser->state++;
            }
            break;

        case 6:
            parser->state = 0;
            if (parser->message_checksum == b) {
                // message received, process
                switch (parser->message_id) {
                
                    case 109: {

                        int altitude;
                        memcpy(&altitude,  &parser->message_buffer[0], sizeof(int));

                        short vario;
                        memcpy(&vario,  &parser->message_buffer[4], sizeof(short));

                        parser->handler_for_ALTITUDE(altitude, vario);
                        } break;

                    case 127: {

                        short back;
                        memcpy(&back,  &parser->message_buffer[0], sizeof(short));

                        short front;
                        memcpy(&front,  &parser->message_buffer[2], sizeof(short));

                        short left;
                        memcpy(&left,  &parser->message_buffer[4], sizeof(short));

                        short right;
                        memcpy(&right,  &parser->message_buffer[6], sizeof(short));

                        parser->handler_for_SONARS(back, front, left, right);
                        } break;

                    case 108: {

                        short roll;
                        memcpy(&roll,  &parser->message_buffer[0], sizeof(short));

                        short pitch;
                        memcpy(&pitch,  &parser->message_buffer[2], sizeof(short));

                        short yaw;
                        memcpy(&yaw,  &parser->message_buffer[4], sizeof(short));

                        parser->handler_for_ATTITUDE(roll, pitch, yaw);
                        } break;

                    case 105: {

                        short c1;
                        memcpy(&c1,  &parser->message_buffer[0], sizeof(short));

                        short c2;
                        memcpy(&c2,  &parser->message_buffer[2], sizeof(short));

                        short c3;
                        memcpy(&c3,  &parser->message_buffer[4], sizeof(short));

                        short c4;
                        memcpy(&c4,  &parser->message_buffer[6], sizeof(short));

                        short c5;
                        memcpy(&c5,  &parser->message_buffer[8], sizeof(short));

                        short c6;
                        memcpy(&c6,  &parser->message_buffer[10], sizeof(short));

                        short c7;
                        memcpy(&c7,  &parser->message_buffer[12], sizeof(short));

                        short c8;
                        memcpy(&c8,  &parser->message_buffer[14], sizeof(short));

                        parser->handler_for_RC(c1, c2, c3, c4, c5, c6, c7, c8);
                        } break;

                }
            }

            break;

        default:
            break;
    }
}

void msp_set_ALTITUDE_handler(msp_parser_t * parser, void (*handler)(int altitude, short vario)) {

    parser->handler_for_ALTITUDE = handler;
}

msp_message_t serialize_ALTITUDE_Request() {

    msp_message_t msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 60;
    msg.bytes[3] = 0;
    msg.bytes[4] = 109;
    msg.bytes[5] = 109;

    msg.len = 6;

    return msg;
}

msp_message_t msp_serialize_ALTITUDE(int altitude, short vario) {

    msp_message_t msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 6;
    msg.bytes[4] = 109;

    memcpy(&msg.bytes[5], &altitude, sizeof(int));
    memcpy(&msg.bytes[9], &vario, sizeof(short));

    msg.bytes[11] = CRC8(&msg.bytes[3], 8);

    msg.len = 12;

    return msg;
}

void msp_set_SONARS_handler(msp_parser_t * parser, void (*handler)(short back, short front, short left, short right)) {

    parser->handler_for_SONARS = handler;
}

msp_message_t serialize_SONARS_Request() {

    msp_message_t msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 60;
    msg.bytes[3] = 0;
    msg.bytes[4] = 127;
    msg.bytes[5] = 127;

    msg.len = 6;

    return msg;
}

msp_message_t msp_serialize_SONARS(short back, short front, short left, short right) {

    msp_message_t msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 8;
    msg.bytes[4] = 127;

    memcpy(&msg.bytes[5], &back, sizeof(short));
    memcpy(&msg.bytes[7], &front, sizeof(short));
    memcpy(&msg.bytes[9], &left, sizeof(short));
    memcpy(&msg.bytes[11], &right, sizeof(short));

    msg.bytes[13] = CRC8(&msg.bytes[3], 10);

    msg.len = 14;

    return msg;
}

msp_message_t msp_serialize_SET_MOTOR(short m1, short m2, short m3, short m4) {

    msp_message_t msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 8;
    msg.bytes[4] = 214;

    memcpy(&msg.bytes[5], &m1, sizeof(short));
    memcpy(&msg.bytes[7], &m2, sizeof(short));
    memcpy(&msg.bytes[9], &m3, sizeof(short));
    memcpy(&msg.bytes[11], &m4, sizeof(short));

    msg.bytes[13] = CRC8(&msg.bytes[3], 10);

    msg.len = 14;

    return msg;
}

void msp_set_ATTITUDE_handler(msp_parser_t * parser, void (*handler)(short roll, short pitch, short yaw)) {

    parser->handler_for_ATTITUDE = handler;
}

msp_message_t serialize_ATTITUDE_Request() {

    msp_message_t msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 60;
    msg.bytes[3] = 0;
    msg.bytes[4] = 108;
    msg.bytes[5] = 108;

    msg.len = 6;

    return msg;
}

msp_message_t msp_serialize_ATTITUDE(short roll, short pitch, short yaw) {

    msp_message_t msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 6;
    msg.bytes[4] = 108;

    memcpy(&msg.bytes[5], &roll, sizeof(short));
    memcpy(&msg.bytes[7], &pitch, sizeof(short));
    memcpy(&msg.bytes[9], &yaw, sizeof(short));

    msg.bytes[11] = CRC8(&msg.bytes[3], 8);

    msg.len = 12;

    return msg;
}

void msp_set_RC_handler(msp_parser_t * parser, void (*handler)(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8)) {

    parser->handler_for_RC = handler;
}

msp_message_t serialize_RC_Request() {

    msp_message_t msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 60;
    msg.bytes[3] = 0;
    msg.bytes[4] = 105;
    msg.bytes[5] = 105;

    msg.len = 6;

    return msg;
}

msp_message_t msp_serialize_RC(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8) {

    msp_message_t msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 16;
    msg.bytes[4] = 105;

    memcpy(&msg.bytes[5], &c1, sizeof(short));
    memcpy(&msg.bytes[7], &c2, sizeof(short));
    memcpy(&msg.bytes[9], &c3, sizeof(short));
    memcpy(&msg.bytes[11], &c4, sizeof(short));
    memcpy(&msg.bytes[13], &c5, sizeof(short));
    memcpy(&msg.bytes[15], &c6, sizeof(short));
    memcpy(&msg.bytes[17], &c7, sizeof(short));
    memcpy(&msg.bytes[19], &c8, sizeof(short));

    msg.bytes[21] = CRC8(&msg.bytes[3], 18);

    msg.len = 22;

    return msg;
}

msp_message_t msp_serialize_SET_RAW_RC(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8) {

    msp_message_t msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 16;
    msg.bytes[4] = 200;

    memcpy(&msg.bytes[5], &c1, sizeof(short));
    memcpy(&msg.bytes[7], &c2, sizeof(short));
    memcpy(&msg.bytes[9], &c3, sizeof(short));
    memcpy(&msg.bytes[11], &c4, sizeof(short));
    memcpy(&msg.bytes[13], &c5, sizeof(short));
    memcpy(&msg.bytes[15], &c6, sizeof(short));
    memcpy(&msg.bytes[17], &c7, sizeof(short));
    memcpy(&msg.bytes[19], &c8, sizeof(short));

    msg.bytes[21] = CRC8(&msg.bytes[3], 18);

    msg.len = 22;

    return msg;
}

msp_message_t msp_serialize_SET_HEAD(short head) {

    msp_message_t msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 2;
    msg.bytes[4] = 205;

    memcpy(&msg.bytes[5], &head, sizeof(short));

    msg.bytes[7] = CRC8(&msg.bytes[3], 4);

    msg.len = 8;

    return msg;
}

