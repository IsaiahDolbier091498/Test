#include <stdint.h>
#ifndef RUNCAM_H
#define RUNCAM_H

typedef struct
{
    uint8_t header; // 0xCC
    uint8_t commandID; 
    uint8_t actionID;
    uint8_t crc8;
} RunCamCommand;

extern void startRecording();

#endif