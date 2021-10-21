//
// Created by fouri on 2021/10/21.
//

#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <JetsonGPIO.h>
#include <string>
#include <chrono>
#include <string>
#include <thread>
#include <iostream>

class PID {
private:
    // ****** PIN LOCATIONS ****** //
    int PWM_pin;
    int standby_pin;
    int direction_pin1;
    int direction_pin2;
    int hall_effect_pin;
    // ****** SPEED VARIABLES ****** //
    float dutyCycle;
    float targetRPM;
    float currentRPM;
    // ****** RPM VARIABLES ****** //
    std::chrono::time_point<std::chrono::high_resolution_clock> rotEnd = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock> rotStart = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds rotDuration;
    // ****** MISC ****** //
    GPIO::PWM* motor_pwm;
    bool setup_complete;
public:
    PID();

    ~PID();

    void setupPWM(bool c);

    void ChangeDutyCycle(float d);

    void rampDown(GPIO::PWM* pwm, int s);

    void setDirection(std::string s);

    void start(float d);

    float getDutyCycle();

    void rpm_interrupt_handler();

    void setPWM_pin(int t) {
        PWM_pin = t;
    }

    void setStandby_pin(int t) {
        standby_pin = t;
    }

    void setDirection1_pin(int t) {
        direction_pin1 = t;
    }

    void setDirection2_pin(int t) {
        direction_pin2 = t;
    }

    void setHall_pin(int t) {
        hall_effect_pin = t;
    }
};


#endif //MOTOR_PID_H
