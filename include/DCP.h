#ifndef __DCP__
#define __DCP__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

enum e_Flags {
    FLAG_Instant        = 0b0,
    FLAG_Assynchronous  = 0b1
};

typedef struct DCP_MODE{
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
    uint8_t SOH;        //header
    uint8_t IDS;        //source ID
    uint8_t IDD;        //destinaiton ID
    uint8_t COD;        //instruction code
    uint8_t data[6];    //data
    uint8_t PAD;        //padding
    uint8_t CRC;        //CRC
};

struct DCP_Message_Generic_t{
    uint8_t addr;
    uint8_t payload[];
};

struct DCP_Message_t {
    uint8_t type;
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

#ifdef __cplusplus
}
#endif

#endif

