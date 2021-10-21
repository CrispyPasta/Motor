//
// Created by fouri on 2021/10/21.
//

#include "PID.h"
using namespace std;
using namespace GPIO;

PID::PID() {
    PWM_pin = 33;           //set the pins to their default locations
    standby_pin = 19;
    direction_pin1 = 21;
    direction_pin2 = 23;
    hall_effect_pin = 18;
    dutyCycle = 0.0;
    targetRPM = 0.0;
    currentRPM = 0.0;
    setup_complete = false;
    this->setupPWM(true);
    motor_pwm = new GPIO::PWM(PWM_pin, 1000);
}


void PID::setupPWM(bool clockwise) {
    try{
        GPIO::setmode(GPIO::BOARD);

        GPIO::setup(PWM_pin, OUT);
        GPIO::setup(standby_pin, OUT);
        GPIO::setup(direction_pin1, OUT);
        GPIO::setup(direction_pin2, OUT);

        if (clockwise){
            GPIO::output(direction_pin1, GPIO::HIGH);                   //set direction to clockwise
            GPIO::output(direction_pin2, GPIO::LOW);
        } else {
            GPIO::output(direction_pin1, GPIO::LOW);                   //set direction to counter-clockwise
            GPIO::output(direction_pin2, GPIO::HIGH);
        }
        GPIO::output(standby_pin, GPIO::HIGH);
        setup_complete = true;
    } catch (...) {
        string e = "Error encountered in setupPWM() function.";
        throw e;
    }
}


void PID::ChangeDutyCycle(float d) {
    if (setup_complete) {
        if ((d <= 100.0) && (d >= 0.0)){
            motor_pwm->ChangeDutyCycle(d);
            dutyCycle = d;
        } else {
            string e = "ERROR: Invalid dutyCycle passed to PID::ChangeDutyCycle.";
            throw e;
        }
    }
}


void PID::rampDown(GPIO::PWM* pwm, int s) {
    int pauseLength = (s * 1000) / dutyCycle;

    for (int a = dutyCycle; a > 0; --a){
        this->ChangeDutyCycle(a);
        std::this_thread::sleep_for(std::chrono::milliseconds(pauseLength));
    }
}


void PID::setDirection(string s) {
    if (s == "cw"){ 
        GPIO::output(direction_pin1, GPIO::HIGH);                   //set direction to clockwise
        GPIO::output(direction_pin2, GPIO::LOW);
    } else if (s == "ccw") {
        GPIO::output(direction_pin1, GPIO::LOW);                   //set direction to counter-clockwise
        GPIO::output(direction_pin2, GPIO::HIGH);
    } else {
        string e = "ERROR: Invalid string given to setDirection function.\n";
        throw e;
    }
}


/**
 * Starts the PWM at the given duty cycle.
 */
void PID::start(float d) {
    motor_pwm->start(d);
}


float PID::getDutyCycle() {
    return this->dutyCycle;
}


void PID::rpm_interrupt_handler() {
    
}


/**
 * Cleans up all the pins used by the motor and the hall effect sensor so there's no garbage left over
 * on the pins after the program runs.
 */
PID::~PID() {
    GPIO::cleanup(this->PWM_pin);
    GPIO::cleanup(this->direction_pin1);
    GPIO::cleanup(this->direction_pin2);
    GPIO::cleanup(this->standby_pin);
    GPIO::cleanup(this->hall_effect_pin);
}
