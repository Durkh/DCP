#ifndef __DCP__
#define __DCP__

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

enum e_Flags {
    FLAG_Instant        = 0u,
    FLAG_Assynchronous  = 1u
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

typedef struct{
    enum DCP_Message_type_e {MESSAGE_SYNC = 0u, MESSAGE_L3, MESSAGE_GENERIC} type;
    size_t size;
    uint8_t* payload;
}DCP_Message_t;

bool SendMessage(const DCP_Message_t* message, const uint8_t addr);
uint8_t ReadByte();
DCP_Message_t* ReadMessage();

#endif

