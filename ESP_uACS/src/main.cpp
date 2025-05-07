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

Adafruit_BMP085 bmp;
ITG3200 gyro;
int16_t gx, gy, gz;
sensors_event_t event;
Adafruit_ADXL345_Unified accel;

Servo servo1, servo2;

PID pid(0.2, 0.001, 0.001);

/*float Setpoint = 0, Input, Output;
float Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);*/

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

    servo1.attach(SERVO_1_PIN);
    servo2.attach(SERVO_2_PIN);

    servo1.write(90);
    servo2.write(90);

    //myPID.SetMode(AUTOMATIC);
}

void loop() {

    //Serial.println(bmp.readPressure());

    gyro.getRotation(&gx, &gy, &gz);
    if (gy > -100 && gy < 100) gy = 0;
    //Serial.printf("%0.2f;%0.2f;%0.2f\n", gx / 14.375, gy / 14.375, gz / 14.375);
    /*Input = gy / 14.375;
    myPID.Compute();
    Serial.println(Output);
    servo1.write(90 + (Output - 127) / 25);
    servo2.write(90 + (Output - 127) / 25);*/

    float setpoint = 0.0;  // Wartość zadana
    float actual = gy / 14.375;     // Wartość aktualna
    float dt = 0.05;         // Krok czasowy (np. 100ms)

    float error = setpoint - actual;
    float control = pid.compute(error, dt);

    int angle = 90 + control;
    if (angle > 105) angle = 105;
    else if (angle < 75) angle = 75;
    Serial.println(angle);

    servo1.write(angle);
    servo2.write(angle);

    //accel.getEvent(&event);
    //Serial.printf("%0.2f;%0.2f;%0.2f\n", event.acceleration.x, event.acceleration.y, event.acceleration.z);

    delay(50);
}
