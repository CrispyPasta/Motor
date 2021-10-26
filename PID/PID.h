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
#include <deque>
#include <iomanip>
#include <fstream>

class PID {
public:
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
    std::chrono::time_point<std::chrono::steady_clock> rotationEnd = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> rotationStart = std::chrono::steady_clock::now();
    std::chrono::microseconds rotationDuration;
    std::deque<float> rpmHistory;
    std::deque<float> errorHistory;
    std::deque<float> rotationTimeHistory;
    // ****** MISC ****** //
    GPIO::PWM* motor_pwm;
    int historySize;

    PID();

    ~PID();

    void setupPins(bool c);

    void ChangeDutyCycle(float d);

    void rampDown(int s);

    void setDirection(std::string s);

    void start(float d);

    float getDutyCycle();

    void rpm_interrupt_handler();

    float pidControl(float targetrpm, float Pk, float Pi, float Pd);

    void printVectors();

    void dumpText();

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
