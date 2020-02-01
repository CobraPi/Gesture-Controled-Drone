/* AUTO-GENERATED CODE: DO NOT EDIT!!!*/

#define MAXBUF 256

typedef unsigned char byte;
typedef unsigned char bool;

typedef struct {

    byte bytes[MAXBUF];
    int pos;
    int len;

} msp_message_t;

typedef struct {

    int state;
    byte message_direction;
    byte message_id;
    byte message_length_expected;
    byte message_length_received;
    byte message_buffer[MAXBUF];
    byte message_checksum;

    void (*handler_for_ALTITUDE)(int altitude, short vario);
    void (*handler_for_SONARS)(short back, short front, short left, short right);
    void (*handler_for_ATTITUDE)(short roll, short pitch, short yaw);
    void (*handler_for_RC)(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8);

} msp_parser_t;

byte msp_message_start(msp_message_t * msg);

bool msp_message_has_next(msp_message_t * msg);

byte msp_message_get_next(msp_message_t * msg);

void msp_parser_init(msp_parser_t * parser);

void msp_parser_parse(msp_parser_t * parser, byte b);
msp_message_t msp_serialize_ALTITUDE(int altitude, short vario);

msp_message_t msp_serialize_ALTITUDE_request();

void msp_set_ALTITUDE_handler(msp_parser_t * parser, void (*handler)(int altitude, short vario));

msp_message_t msp_serialize_SONARS(short back, short front, short left, short right);

msp_message_t msp_serialize_SONARS_request();

void msp_set_SONARS_handler(msp_parser_t * parser, void (*handler)(short back, short front, short left, short right));

msp_message_t msp_serialize_SET_MOTOR(short m1, short m2, short m3, short m4);

msp_message_t msp_serialize_ATTITUDE(short roll, short pitch, short yaw);

msp_message_t msp_serialize_ATTITUDE_request();

void msp_set_ATTITUDE_handler(msp_parser_t * parser, void (*handler)(short roll, short pitch, short yaw));

msp_message_t msp_serialize_RC(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8);

msp_message_t msp_serialize_RC_request();

void msp_set_RC_handler(msp_parser_t * parser, void (*handler)(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8));

msp_message_t msp_serialize_SET_RAW_RC(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8);

msp_message_t msp_serialize_SET_HEAD(short head);

