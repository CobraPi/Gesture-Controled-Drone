#ifndef MESSAGES_H
#define MESSAGES_H

typedef enum {
    IncomingMessageEcho = 0x00,

    IncomingMessageBind = 0x01,
    IncomingMessageUnbind = 0x02,
    IncomingMessageReconnect = 0x03,

    IncomingMessageOperatingMode = 0x04,
    IncomingMessageControlSource = 0x05,
    IncomingMessageControlPayload = 0x06,

    IncomingMessageCalibrateAccelerometer = 0x07,
    IncomingMessageRequestAccelerometerData = 0x08,
    IncomingMessagePayloadCorrect = 0x09,
    IncomingMessagePayloadFailure = 0x0A
} IncomingMessageType;

typedef enum {
    OperatingModeFly = 0x00,
    OperatingModeDrive = 0x0F
} OperatingModeType;

typedef enum {
    ControlSourceJoystick = 0x00,
    ControlSourceAccelerometer = 0x01,
    ControlSourceSerial = 0x02
} ControlSourceType;


typedef enum {
    OutgoingMessageBindFailure = 0x00,
    OutgoingMessageBindSuccess = 0x01,

    OutgoingMessageUnbindFailure = 0x02,
    OutgoingMessageUnbindSuccess = 0x03,

    OutgoingMessageReconnectFailure = 0x04,
    OutgoingMessageReconnectSuccess = 0x05,

    OutgoingMessageInitializationFailure = 0x06,

    OutgoingMessagePayloadReady = 0x07,

    OutgoingMessagePing = 254,
    OutgoingMessageDebug = 255,
} OutgoingMessageType;

typedef enum {
    InitializationFailureAccelerometer = 0x00,
    InitializationFailureJoysticks = 0x01
} InitializationFailureType;

#endif // MESSAGES_H
