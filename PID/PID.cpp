//
// Created by fouri on 2021/10/21.
//

#include "PID.h"
using namespace std;
using namespace chrono;
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
    this->setupPins(true);
    motor_pwm = new GPIO::PWM(PWM_pin, 1000);

    historySize = 15;
    // errorHistory.reserve(historySize);
    // rotationTimeHistory.reserve(historySize);
}


void PID::setupPins(bool clockwise) {
    try{
        GPIO::setmode(GPIO::BOARD);

        GPIO::setup(PWM_pin, GPIO::OUT);
        GPIO::setup(standby_pin, GPIO::OUT);
        GPIO::setup(direction_pin1, GPIO::OUT);
        GPIO::setup(direction_pin2, GPIO::OUT);

        if (clockwise){
            GPIO::output(direction_pin1, GPIO::HIGH);                   //set direction to clockwise
            GPIO::output(direction_pin2, GPIO::LOW);
        } else {
            GPIO::output(direction_pin1, GPIO::LOW);                   //set direction to counter-clockwise
            GPIO::output(direction_pin2, GPIO::HIGH);
        }
        GPIO::output(standby_pin, GPIO::HIGH);

        GPIO::setup(hall_effect_pin, GPIO::IN);         //RPM section
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


void PID::rampDown(int s) {
    int pauseLength = (s * 1000) / dutyCycle;

    for (int a = dutyCycle; a > 0; --a){
        this->ChangeDutyCycle(a);
        std::this_thread::sleep_for(std::chrono::milliseconds(pauseLength));
    }

    this->ChangeDutyCycle(0);
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
    cout << "RPM interrupt handler.\n";
    rotationEnd = steady_clock::now();
    rotationDuration = duration_cast<milliseconds>(rotationEnd - rotationStart);
    rotationStart = steady_clock::now();

    cout << "Rotation duration: " << rotationDuration.count() << " milliseconds\n";
    currentRPM = 60000.0 / rotationDuration.count();
    cout << "RPM: " << to_string(currentRPM) << '\n';

    errorHistory.push_back(currentRPM);
    rotationTimeHistory.push_back(rotationDuration.count());

    // if (errorHistory.size() == historySize) {   //if the vector is full, remove the oldest element
    //     errorHistory.erase(errorHistory.begin());
    //     rotationTimeHistory.erase(rotationTimeHistory.begin());
    // }

    printVectors();
}

void PID::printVectors() {
    for (int a = 0; a < errorHistory.size(); a++) {
        cout << "RPM: " << setprecision(3) << errorHistory[a];
        cout << ". Time: " << setprecision(3) <<  rotationTimeHistory[a] << '\n';
    }
    cout << '\n';
}

void PID::dumpText() {
    ofstream outFile;
    outFile.open("./motorData.txt", std::ios_base::app);
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    outFile << "==============================";
    outFile << "\nDate/Time: ";
    outFile << put_time(&tm, "%d-%m-%Y %H-%M-%S") << endl;
    outFile << "==============================\n";

    for (int a = 0; a < errorHistory.size(); a++){
        outFile << "RPM: " << setprecision(5) << setw(8) << errorHistory[a] << '\t';
        outFile << "Δt: " << setprecision(3) << rotationTimeHistory[a] << '\n';
    }

    outFile << '\n';

    outFile.close();
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
