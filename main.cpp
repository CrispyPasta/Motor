#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <signal.h>
#include <fstream>
#include "./PID/PID.h"
#include <JetsonGPIO.h>

using namespace GPIO;
using namespace std;
using namespace chrono;

bool done = false;
PID* pid_ptr = nullptr;
int rotations = 0;

bool PIDmode = false;

void delayMs(int ms) { 
    this_thread::sleep_for(chrono::milliseconds(ms)); 
}

void signalHandler(int s){
    done = true;
}

void blink(int channel)
{
    pid_ptr->rpm_interrupt_handler();
}

void readParams(float &Pk, float &Ti, float &Td, int &pidDelay, float &targetRPM) {
    ifstream infile;
    infile.open("./PIDparams.txt");
    string value;
    getline(infile, value);
    Pk = stof(value);

    getline(infile, value);
    Ti = stof(value);

    getline(infile, value);
    Td = stof(value);

    getline(infile, value);
    targetRPM = stof(value);

    getline(infile, value);
    pidDelay = stoi(value);

    getline(infile, value);
    int temp = stoi(value);
    if (temp == 1) {
        PIDmode = true;
    } else if (temp == 0) {
        PIDmode = false;
    }
}

int main(int numArgs, char* args[]) {
    cout << "Hello, World!" << endl;
    GPIO::setwarnings(false);
    GPIO::cleanup();
    float targetRPM = 0;
    float Pk;
    float Ti;
    float Td;
    int pidDelay;

    PIDmode = false;
    readParams(Pk, Ti, Td, pidDelay, targetRPM);
    cout << "arguments set\n";

    //when CTRL+C is pressed, signalHandler will be invoked.
    try {
        signal(SIGINT, signalHandler);
        pid_ptr = new PID();
        pid_ptr->targetRPM = targetRPM;
        
        // double dutyCycle = 10;
        double dutyCycle = targetRPM / 50.0;
        pid_ptr->start(dutyCycle);
        GPIO::setup(18, GPIO::IN);
        GPIO::setup(11, GPIO::OUT, GPIO::LOW);
        GPIO::add_event_detect(18, GPIO::Edge::FALLING, blink, 10);

        int increment = 5;

        int runs = 0;
        // while(!done){
        //     if (runs == 0) {
        //         delayMs(1200);      //one long delay to let the motor stabilize first
        //         cout << "\n\nStartup done \n\n";
        //     }
        //     if (PIDmode) {
        //         delayMs(pidDelay);
        //         pid_ptr->updateError(runs == 1);
        //         dutyCycle = pid_ptr->pidControl(targetRPM, Pk, Ti, Td);
        //         pid_ptr->ChangeDutyCycle(dutyCycle);
        //         if (runs == 600) {
        //             done = true;
        //         }
        //     } else {
        //         delayMs(1500);
        //         dutyCycle += increment;
        //         cout << dutyCycle << '\n';
        //         if (dutyCycle >= 75){
        //             increment = -5;
        //         }
        //         if (dutyCycle <= 10){
        //             increment = 5;
        //         }
        //         pid_ptr->ChangeDutyCycle(dutyCycle);
        //         pid_ptr->updateError(runs == 1);
        //         // done = true;
        //         // if (pid_ptr->currentRPM >= 4500) {
        //         //     cout << "\n\n\n\n 4500RPM reached at " << dutyCycle << "\n\n\n\n";
        //         // }
        //     }
        //     runs++;
        // }

        while(!done) {
            if (runs < 2) {
                delayMs(1000);      //one long delay to let the motor stabilize first
            } else {
                // dutyCycle += 10;
                pid_ptr->ChangeDutyCycle(dutyCycle);
                delayMs(1000);
                done = true;
            }        
            runs++;
        }

        pid_ptr->rampDown(3);
        pid_ptr->dumpText();

    } catch (...) {
        cout << "Error encountered in loop, ramping down.\n";
        pid_ptr->rampDown(3);
        pid_ptr->dumpText();
    }

    return 0;
}
