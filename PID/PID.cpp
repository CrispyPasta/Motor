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

    this->setupPins(true);
    motor_pwm = new GPIO::PWM(PWM_pin, 1000);

    historySize = 1000;
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
    } catch (...) {
        string e = "Error encountered in setupPWM() function.";
        throw e;
    }
}


void PID::ChangeDutyCycle(float d) {
    if ((d <= 100.0) && (d >= 0.0)){
        motor_pwm->ChangeDutyCycle(d);
        dutyCycle = d;
        if (dutyCycle < 5) {
            currentRPM = 0;
        }
    } else {
        string e = "ERROR: Invalid dutyCycle passed to PID::ChangeDutyCycle.";
        throw e;
    }
}


void PID::rampDown(int s) { 
    if (dutyCycle == 0) {
        return;
    }
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
    // cout << "RPM\n";
    rotationEnd = steady_clock::now();
    rotationDuration = duration_cast<microseconds>(rotationEnd - rotationStart);
    rotationStart = steady_clock::now();

    // cout << "Rotation duration: " << (rotationDuration.count() / 1000.0) << " milliseconds\n";
    currentRPM = 60000000.0 / rotationDuration.count();
    // cout << "RPM: " << to_string(currentRPM) << '\n';

    rpmHistory.push_back(currentRPM);
    // errorHistory.push_back(targetRPM - currentRPM);
    // cout << "Error: " << to_string(errorHistory.back()) << '\n';
    rotationTimeHistory.push_back(rotationDuration.count());

    if (rpmHistory.size() > historySize) {   //if the vector is full, remove the oldest element
        rpmHistory.pop_front();
        // errorHistory.pop_front();
        rotationTimeHistory.pop_front();
    }
    // cout << "RPM\n";
}


void PID::updateError(bool first) {
    errorEnd = steady_clock::now();
    errorDuration = duration_cast<milliseconds>(errorEnd - errorStart);
    errorStart = steady_clock::now();

    errorHistory.push_back(targetRPM - currentRPM);
    if (first) {
        errorTimeHistory.push_back(20);
    } else {
        errorTimeHistory.push_back(errorDuration.count());
    }

    if (errorHistory.size() > historySize) {
        errorHistory.pop_front();
        errorTimeHistory.pop_front();
    }
}


float PID::pidControl(float t, float Pk, float Ti, float Td) {
    // cout << "PID CONTROL FUNCTION\n";
    targetRPM = t;      //update the target RPM for the next time the rpm interrupt is called

    if (errorHistory.size() > 2) {
        float proportional = errorHistory.back();          //use latest error
        cout << "Last error: " << proportional << '\n';


        float integral = 0;
        for (int a = 0; a < errorHistory.size(); a++) { //calculate discrete integral over the duration in seconds (multiply milliseconds by 1000)
            integral += errorHistory[a] * (errorTimeHistory[a]);      
        }
        integral = (1.0 / Ti) * integral;
        // cout << "PID CONTROL integral set: " << integral << '\n';

        float errorChange = (errorHistory.back() - errorHistory[errorHistory.size() -2]);
        float derivative = (errorChange / errorTimeHistory.back());     //change in error divided by the time between error calculations (in ms). 
        derivative = Td * derivative;
        // cout << "PID CONTROL derivative set: " << derivative << '\n';

        float output = Pk * (proportional + integral + derivative);
        // cout << "PID CONTROL Output:" << output << '\n';

        //correct any invalid values for duty cycle here 
        if (output > 100) {
            output = 100.0;
        } 

        if (output < 0) {
            output = 0.0;
        }
        
        cout << "PID CONTROL Output:" << output << "\n\n";
        return output;
    }

    if (currentRPM < targetRPM) {
        // cout << (dutyCycle +10) << '\n';
        return (dutyCycle + 10);
    } 
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
    // ********** RPM Data txt file ***********
    outFile.open("./rpmData.txt", std::ios_base::app);
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    outFile << "==============================";
    outFile << "\nDate/Time: ";
    outFile << put_time(&tm, "%d-%m-%Y %H-%M-%S") << endl;
    outFile << "==============================\n";

    for (int a = 0; a < rpmHistory.size(); a++){
        outFile << "RPM: " << setprecision(5) << setw(8) << rpmHistory[a] << '\t';
        outFile << "??t (ms): " << setprecision(6) << (rotationTimeHistory[a] / 1000) << '\n';
    }
    outFile << '\n';
    outFile.close();
    // ********** RPM Data txt file ***********

    // ********** Error Data txt file ***********
    outFile.open("./errorData.txt", std::ios_base::app);
    outFile << "==============================";
    outFile << "\nDate/Time: ";
    outFile << put_time(&tm, "%d-%m-%Y %H-%M-%S") << endl;
    outFile << "==============================\n";

    for (int a = 0; a < errorHistory.size(); a++){
        outFile << "E(t): " << setprecision(5) << setw(8) << errorHistory[a] << '\t';
        outFile << "??t: " << setprecision(5) << errorTimeHistory[a] << '\n';
    }
    outFile << '\n';
    outFile.close();
    // ********** Error Data txt file ***********

    // ********** RPM Data csv file ***********
    outFile.open("./rpmData.csv");
    outFile << "RPM,deltaT\n";
    outFile << "0.0,0.0\n";
    for (int a = 0; a < rpmHistory.size(); a++){
        outFile << setprecision(6) << rpmHistory[a] << ',';
        outFile << setprecision(6) << rotationTimeHistory[a] << '\n';
    }
    outFile << '\n';
    outFile.close();
    // ********** RPM Data csv file ***********
}


/**
 * Cleans up all the pins used by the motor and the hall effect sensor so there's no garbage left over
 * on the pins after the program runs.
 */
PID::~PID() {
    try {
        cout << "PID destructor.\n";
        GPIO::cleanup(this->PWM_pin);
        GPIO::cleanup(this->direction_pin1);
        GPIO::cleanup(this->direction_pin2);
        GPIO::cleanup(this->standby_pin);
        GPIO::cleanup(this->hall_effect_pin);
    } catch (...) {
        cout << "Error encountered in PID destructor.\n";
    }
}
