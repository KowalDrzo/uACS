#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>

#include "frame.h"

extern float initialPressure;
extern float initialTemperature;

void initFs();

void appendDataFile(Frame frame);

void clearDataFile();

void readDataFile();


#endif
