#include "functions.h"

float initialPressure = 1;
float initialTemperature = -100;
File file;

void initFs() {
    SPIFFS.begin(true);
}

void appendDataFile(Frame frame) {

    file = SPIFFS.open("/FlightData.apg", "a");

    char tempDataAscii[200];
    sprintf(tempDataAscii, "%d;%f;%f;%f;%d;%d;%f;%f;%f;%f\n",
            frame.time_ms, frame.gyro_x, frame.gyro_y, frame.gyro_z, frame.angle1, frame.angle2,
            frame.acc_x, frame.acc_y, frame.acc_z, frame.alt);
    file.write((uint8_t*) tempDataAscii, strlen(tempDataAscii));

    file.close();
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
