#include "functions.h"

float initialPressure = 1;
float initialTemperature = -100;
float baroPressure = 101325;

Adafruit_BMP085 bmp;
File file;

void initFs() {
    SPIFFS.begin(true);
    Serial.print("Free KB: ");
    Serial.println((SPIFFS.totalBytes() - SPIFFS.usedBytes()) / 1024);
}

Frame framesCopy[FRAMES_NUM];

void appendDataFile(Frame *frames) {

    //file = SPIFFS.open("/FlightData.apg", "a");

    //char tempDataAscii[200];
    for (int8_t j = 0; j < FRAMES_NUM; j ++) {
        framesCopy[j] = frames[j];
    }

    //file.close();
    Serial.println("Data save");
    xTaskCreate(saveTask, "Save Task", 32768, NULL, 2, NULL);
}

void saveTask(void *pvParameter) {

    file = SPIFFS.open("/FlightData.apg", "a");

    char tempDataAscii[200];
    for (int8_t j = 0; j < FRAMES_NUM; j ++) {
        sprintf(tempDataAscii, "%d;%d;%d;%d;%d;%d;%f;%f;%f;%f\n",
                framesCopy[j].time_ms, (int)framesCopy[j].gyro_x, (int)framesCopy[j].gyro_y,
                (int)framesCopy[j].gyro_z, framesCopy[j].angle1, framesCopy[j].angle2,
                framesCopy[j].acc_x, framesCopy[j].acc_y, framesCopy[j].acc_z, framesCopy[j].alt);
        file.write((uint8_t*) tempDataAscii, strlen(tempDataAscii));
    }

    file.close();

    vTaskDelete(NULL);
}

void clearDataFile() {

    file = SPIFFS.open("/FlightData.apg", "w");
    file.close();
    Serial.println(SPIFFS.remove("/FlightData.apg"));
    Serial.println("CLEARED");
}

void readDataFile() {

    file = SPIFFS.open("/FlightData.apg", "r");

    while (file.available()) {
        String fileContent = file.readStringUntil('\n');
        Serial.println(fileContent);
    }

    file.close();
}

void baroTask(void *pvParameter) {

    while (1) {
        baroPressure = bmp.readPressure();
        vTaskDelay(1);
    }
}