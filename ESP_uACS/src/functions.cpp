#include "functions.h"

float initialPressure = 1;
float initialTemperature = -100;
File file;

void initFs() {
    SPIFFS.begin(true);
}

Frame framesCopy[FRAMES_NUM];

void appendDataFile(Frame *frames) {

    file = SPIFFS.open("/FlightData.apg", "a");

    char tempDataAscii[200];
    for (int8_t j = 0; j < FRAMES_NUM; j ++) {
        framesCopy[j] = frames[j];
    }

    file.close();

    xTaskCreate(saveTask, "Save Task", 32768, NULL, 2, NULL);
}

void saveTask(void *pvParameter) {

    file = SPIFFS.open("/FlightData.apg", "a");

    char tempDataAscii[200];
    for (int8_t j = 0; j < FRAMES_NUM; j ++) {
        sprintf(tempDataAscii, "%d;%f;%f;%f;%d;%d;%f;%f;%f;%f\n",
                framesCopy[j].time_ms, framesCopy[j].gyro_x, framesCopy[j].gyro_y,
                framesCopy[j].gyro_z, framesCopy[j].angle1, framesCopy[j].angle2,
                framesCopy[j].acc_x, framesCopy[j].acc_y, framesCopy[j].acc_z, framesCopy[j].alt);
        file.write((uint8_t*) tempDataAscii, strlen(tempDataAscii));
    }

    file.close();

    vTaskDelete(NULL);
}

void clearDataFile() {

    file = SPIFFS.open("/FlightData.apg", "w");
    file.close();
    Serial.println("CLEARED");
}

void readDataFile() {

    file = SPIFFS.open("/FlightData.apg", "r");

    while (file.available()) {
        String fileContent = file.readString();
        Serial.print(fileContent);
    }

    file.close();
}
