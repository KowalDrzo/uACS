#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>

#include "frame.h"

#define FRAMES_NUM 20

extern float initialPressure;
extern float initialTemperature;

void initFs();

void appendDataFile(Frame *frame);

void saveTask(void *pvParameter);

void clearDataFile();

void readDataFile();


#endif
