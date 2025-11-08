#include <ICM42688.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP5xx.h>

#include <ESP32Servo.h>

#include "pinout.h"
#include "pid.h"
#include "functions.h"
#include "filter.h"

ICM42688 imu(Wire, 0x69);
Adafruit_BMP5xx bmp;

int16_t gx, gy, gz;
float alt;

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

    int status = imu.begin();
    if (status < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while(1) {}
    }

    imu.setAccelFS(ICM42688::gpm16);
    imu.setGyroFS(ICM42688::dps2000);
    imu.setAccelODR(ICM42688::odr2k);
    imu.setGyroODR(ICM42688::odr2k);

    if(!bmp.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire)) {
        Serial.println("BMP Error");
        while(1) {}
    }

    bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
    bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP5XX_ODR_10_HZ);
    bmp.enablePressure(true);

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    servo1.attach(SERVO_1_PIN);
    servo2.attach(SERVO_2_PIN);

    servo1.write(90);
    servo2.write(90);

    initFs();

    bmp.performReading();
    initialTemperature = bmp.temperature;
    initialPressure = bmp.pressure;

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

            imu.getAGT();

            tableX[i] = imu.gyrX();
            tableY[i] = imu.gyrY();
            tableZ[i] = imu.gyrZ();
            baroPressure = bmp.readPressure();

            tableAlt[i] = (initialTemperature+273.15)/0.0065*(1.0 - pow(baroPressure/initialPressure, 0.1903));
        }

        frames[j].acc_x = imu.accX();
        frames[j].acc_y = imu.accY();
        frames[j].acc_z = imu.accZ();

        // Filtrowanie:
        gx = filterX.doFiltering3Vals(tableX);
        gy = filterY.doFiltering3Vals(tableY);
        gz = filterZ.doFiltering3Vals(tableZ);
        alt = filterAlt.doFiltering3Vals(tableAlt);

        // Skalowanie: TODO
        gx /= 14.375;
        gy /= 14.375;
        gz /= 14.375;

        frames[j].gyro_x = gx;
        frames[j].gyro_y = gy;
        frames[j].gyro_z = gz;
        frames[j].alt = alt;

        //if (gy > -100 && gy < 100) gy = 0; TODO

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

        //if (j >= FRAMES_NUM - 1) appendDataFile(frames); TODO

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

    vTaskDelay(1 / portTICK_PERIOD_MS);
}
