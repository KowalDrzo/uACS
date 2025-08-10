#ifndef FRAME_H
#define FRAME_H

#include <Arduino.h>

struct Frame
{
    uint32_t time_ms;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    uint8_t angle1;
    uint8_t angle2;
    float acc_x;
    float acc_y;
    float acc_z;
    float alt;
};

#endif
