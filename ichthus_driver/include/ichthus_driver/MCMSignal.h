#include <stdint.h>

#define DEFAULT_BUS 0x01
#define SUBSYS_1 0
#define SUBSYS_2 1
#define SUBSYS_1_BUS_1 0x01
#define SUBSYS_2_BUS_1 0x81

#define NONE        0x00
#define BRAKE_ID    0x00
#define ACCEL_ID    0x20
#define STEER_ID    0x40

#define OVERRIDE_ACK 0x41
#define COMMAND_ID 0x160
#define CONTROL_ID 0x60

#define SUBSYS_MASK 10000000

// Defined for Fault & Override Ack
#define ACK_KEY 0xA5
#define ACK_COND 2  // Needed for Ack
#define OVERRIDE_MSG_TIMEOUT 2 //Override MSG Timeout Period 2sec

typedef struct ACK_MSG{
    int subsys_id;
    char key[4];
}ACK_MSG;

union float_hex_convert{
    unsigned int hex;
    char data[4];
    float val;
};

enum DeviceType{
    KIACAN,
    MCM,
    DEFAULT
};

enum MCM_MESSAGE_TYPE{
    CONTROL_RESPONSE,
    FAULT,
    OVERRIDE,
    OVERRIDE_RESPONSE
};

enum MCM_GENERAL_STATE{
    CONTROL_DISABELD,
    CONTROL_ENABLED,
    OVERRIDED,
    FAULT_
};

typedef struct MCM_STATE{
    bool control_State;
    bool Brake_Control_State;
    bool Accel_Control_State;
    bool Steer_Control_State;
    bool OVERRIDE;
    int FAULT;
}MCM_STATE;

//STRUCTS FOR CAN DATA Frames
namespace CanMessage
{

typedef struct MCM_DATA{
    MCM_MESSAGE_TYPE type;
    unsigned int subsys_id;
    char hex_id;
    float float_data;
    bool bool_data;
    char fault_msg[4];
    char override_msg[4];
}MCM_DATA;


typedef struct WHL_SPD{
    float FL;
    float FR;
    float RL;
    float RR;
}WHL_SPD;

} //namespace Can Message
