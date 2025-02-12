#ifndef __DCP__
#define __DCP__

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

enum e_Flags {
    FLAG_Instant        = 0b0,
    FLAG_Assynchronous  = 0b1
};

typedef struct{
    uint8_t addr;
    union{
        enum e_Flags e_flags;
        unsigned int flags; //TODO change var size?
    } flags;
    bool isController;
    enum {SLOW = 0, FAST1, FAST2, ULTRA} speed;
}DCP_MODE;

bool DCPInit(const unsigned int busPin, const DCP_MODE mode);

struct DCP_Message_L3_t{
    uint8_t SOH;
    uint8_t IDS;
    uint8_t IDD;
    uint8_t COD;
    uint8_t data[6];
    uint8_t PAD;
    uint8_t CRC;
};

struct DCP_Message_Generic_t{
    uint8_t addr;
    uint8_t payload[];
};

struct DCP_Message_t {
    //by definition, the type is either: SYNC, L3, or the number of bytes to be sent.
    enum __attribute__((__packed__)) DCP_Message_type_e {MESSAGE_SYNC = 0u, MESSAGE_L3 = 1u} type;
    union {
        struct DCP_Message_L3_t L3;
        struct DCP_Message_Generic_t generic;
    };
};

//this is done so we can access the message as a continuous byte array
//in this way, it is possible to send everything in one go
typedef union {
    struct DCP_Message_t * const message;
    uint8_t * data;
} DCP_Data_t;

bool SendMessage(const DCP_Data_t message);
struct DCP_Message_t* ReadMessage();

#endif

