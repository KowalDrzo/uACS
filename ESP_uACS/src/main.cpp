#include <I2Cdev.h>
#include <ITG3200.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <ESP32Servo.h>

#include "pinout.h"
#include "pid.h"
#include "functions.h"
#include "filter.h"

ITG3200 gyro;
int16_t gx, gy, gz;
float alt;
sensors_event_t event;
Adafruit_ADXL345_Unified accel;

Servo servo1, servo2;
Frame frames[FRAMES_NUM];

PID pid(0.035, 0.002, 0.001);
Filter filterX(0.2);
Filter filterY(0.2);
Filter filterZ(0.2);
Filter filterAlt(0.2);

uint32_t timer;

void setup() {

    Serial.begin(115200);
    delay(3000);

    Wire.begin(SDA_PIN, SCL_PIN, 400000);
    bmp.begin(3);
    Serial.println(bmp.readPressure());
    gyro.initialize();
    gyro.testConnection();
    accel.begin();
    accel.setRange(ADXL345_RANGE_16_G);

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    servo1.attach(SERVO_1_PIN);
    servo2.attach(SERVO_2_PIN);

    servo1.write(90);
    servo2.write(90);

    initFs();

    initialTemperature = bmp.readTemperature();
    initialPressure = bmp.readPressure();
    xTaskCreate(baroTask, "Baro Task", 32768, NULL, 2, NULL);

    timer = millis();
}

int8_t j = 0;

void loop() {

    if (millis() - timer >= 50) {

        timer = millis();
        frames[j].time_ms = timer;

        float tableX[3];
        float tableY[3];
        float tableZ[3];
        float tableAlt[3];

        for (int8_t i = 0; i < 3; i++) {
            gyro.getRotation(&gx, &gy, &gz);
            tableX[i] = gx;
            tableY[i] = gy;
            tableZ[i] = gz;
            tableAlt[i] = (initialTemperature+273.15)/0.0065*(1.0 - pow(baroPressure/initialPressure, 0.1903));
        }

        // Filtrowanie:
        gx = filterX.doFiltering3Vals(tableX);
        gy = filterY.doFiltering3Vals(tableY);
        gz = filterZ.doFiltering3Vals(tableZ);
        alt = filterAlt.doFiltering3Vals(tableAlt);

        // Skalowanie:
        gx /= 14.375;
        gy /= 14.375;
        gz /= 14.375;

        frames[j].gyro_x = gx;
        frames[j].gyro_y = gy;
        frames[j].gyro_z = gz;
        frames[j].alt = alt;

        if (gy > -100 && gy < 100) gy = 0;

        float setpoint = 0.0;  // Wartość zadana
        float actual = gy;     // Wartość aktualna
        float dt = 0.05;         // Krok czasowy (np. 100ms)

        float error = setpoint - actual;
        float control = pid.compute(error, dt);

        uint8_t angle = 90 + control;
        if (angle > 105) angle = 105;
        else if (angle < 75) angle = 75;
        //Serial.println(angle);

        servo1.write(angle);
        servo2.write(angle);

        frames[j].angle1 = angle;
        frames[j].angle2 = angle;

        accel.getEvent(&event);
        frames[j].acc_x = event.acceleration.x;
        frames[j].acc_y = event.acceleration.y;
        frames[j].acc_z = event.acceleration.z;

        if (j >= FRAMES_NUM - 1) appendDataFile(frames);

        j++;
        if (j >= FRAMES_NUM) j = 0;
    }

    if (!digitalRead(BUTTON_PIN)) {
        int8_t i;
        for (i = 0; i < 10; i++) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            if (digitalRead(BUTTON_PIN)) break;
        }

        if (i < 9) readDataFile();
        else clearDataFile();
    }

    vTaskDelay(3 / portTICK_PERIOD_MS);
}
