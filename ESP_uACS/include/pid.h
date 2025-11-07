#ifndef PID_H
#define PID_H

class PID {

    float Kp;      // Wzmocnienie proporcjonalne
    float Ki;      // Wzmocnienie całkujące
    float Kd;      // Wzmocnienie różniczkujące
    float prevError;     // Poprzedni błąd
    float integral;      // Suma całki

public:

    PID(float kp, float ki, float kd)
        : Kp(kp), Ki(ki), Kd(kd), prevError(0.0), integral(0.0) {}

    float compute(float error, float dt);
    void reset();
};

#endif
