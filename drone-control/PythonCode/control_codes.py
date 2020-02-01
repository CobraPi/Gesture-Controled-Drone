class IncomingMessageType:
    BindFailure = 0
    BindSuccess = 1

    UnbindFailure = 2
    UnbindSuccess = 3

    ReconnectFailure = 4
    ReconnectSuccess = 5
    InitializationFailure = 6

    PayloadReady = 7

    Ping = 254
    Debug = 255


class OutgoingMessageType:
    Echo = 0

    Bind = 1
    Unbind = 2
    Reconnect = 3

    OperatingMode = 4
    ControlSource = 5
    ControlPayload = 6

    CalibrateAccelerometer = 7
    RequestAccelerometerData = 8
    PayloadCorrect = 9
    PayloadFailure = 10


class OperatingModeType:
    Fly = 0
    Drive = 15


class ControlSourceType:
    Joystick = 0
    Accelerometer = 1
    Serial = 2


class InitializationFailureType:
    AccelerometerFailure = 0
    JoystickFailure = 1

