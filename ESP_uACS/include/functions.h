#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP085.h>

#include "frame.h"

#define FRAMES_NUM 100

extern float initialPressure;
extern float initialTemperature;
extern float baroPressure;
extern Adafruit_BMP085 bmp;

void initFs();

void appendDataFile(Frame *frame);

void saveTask(void *pvParameter);

void clearDataFile();

void readDataFile();

void baroTask(void *pvParameter);

#endif
