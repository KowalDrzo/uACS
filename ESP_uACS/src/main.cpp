#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP085.h>
#include <I2Cdev.h>
#include <ITG3200.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <ESP32Servo.h>

#include "pinout.h"
#include "pid.h"
#include "functions.h"
#include "filter.h"

Adafruit_BMP085 bmp;
ITG3200 gyro;
int16_t gx, gy, gz;
float press;
sensors_event_t event;
Adafruit_ADXL345_Unified accel;

Servo servo1, servo2;
Frame frame;

PID pid(0.035, 0.002, 0.001);
Filter filterX(0.2);
Filter filterY(0.2);
Filter filterZ(0.2);
Filter filterPress(0.2);

uint32_t timer;

void setup() {

    Serial.begin(115200);
    delay(3000);

    Serial.println("Dupa1");
    Wire.begin(SDA_PIN, SCL_PIN);
    Serial.println("Dupa2");
    bmp.begin(3);
    Serial.println("Dupa7");
    Serial.println(bmp.readPressure());
    Serial.println("Dupa3");
    gyro.initialize();
    Serial.println("Dupa4");
    gyro.testConnection();
    Serial.println("Dupa6");
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

    timer = millis();
}

void loop() {

    if (millis() - timer >= 50) {

        timer = millis();
        frame.time_ms = timer;

        float tableX[3];
        float tableY[3];
        float tableZ[3];
        float tablePress[3];

        for (int8_t i = 0; i < 3; i++) {
            gyro.getRotation(&gx, &gy, &gz);
            tableX[i] = gx;
            tableY[i] = gy;
            tableZ[i] = gz;
            tablePress[i] = 101325;//bmp.readPressure();
        }

        // Filtrowanie:
        gx = filterX.doFiltering3Vals(tableX);
        gy = filterY.doFiltering3Vals(tableY);
        gz = filterZ.doFiltering3Vals(tableZ);
        press = filterPress.doFiltering3Vals(tablePress);

        // Skalowanie:
        gx /= 14.375;
        gy /= 14.375;
        gz /= 14.375;

        frame.gyro_x = gx;
        frame.gyro_y = gy;
        frame.gyro_z = gz;
        frame.alt = (initialTemperature+273.15)/0.0065*(1.0 - pow(press/initialPressure, 0.1903));

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

        frame.angle1 = angle;
        frame.angle2 = angle;

        accel.getEvent(&event);
        frame.acc_x = event.acceleration.x;
        frame.acc_y = event.acceleration.y;
        frame.acc_z = event.acceleration.z;

        uint32_t timer_save = millis();
        appendDataFile(frame);
        Serial.println(millis() - timer_save);
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

    vTaskDelay(10 / portTICK_PERIOD_MS);
}
