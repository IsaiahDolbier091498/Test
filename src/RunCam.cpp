#include <stdint.h> // for uint8_t
#include <stddef.h> // for size_t
#include "RunCam.h"
#include "Arduino.h"
//https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol

const uint8_t startRecordingPacket[] = {0xCC, 0x01, 0x03};
const uint8_t stopRecordingPacket[] = {0xCC, 0x01, 0x04};

//CRC checksum implementation provided
uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (int i = 0; i < 8; ++i) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }

    return crc;
}

uint8_t processCrc8(const uint8_t* data, size_t length)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++)
    {
        crc = crc8_dvb_s2(crc, (unsigned char)data[i]);
    }
    return crc;
}

void startRecording()
{
    RunCamCommand startRecording = {0xCC, 0x01, 0x03, processCrc8(startRecordingPacket, 3)};
    Serial1.write((const uint8_t*)&startRecording, sizeof(startRecording));
}

void stopRecording()
{
    RunCamCommand stopRecording = {0xCC, 0x01, 0x04, processCrc8(stopRecordingPacket, 3)};
    Serial1.write((const uint8_t*)&stopRecording, sizeof(stopRecording));
}