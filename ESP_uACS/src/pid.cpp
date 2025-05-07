#include "pid.h"

float PID::compute(float error, float dt) {
    integral += error * dt;
    float derivative = (error - prevError) / dt;
    prevError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;
    return output;
}

void PID::reset() {
    prevError = 0.0;
    integral = 0.0;
}
