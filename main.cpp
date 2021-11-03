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

void readParams(float &Pk, float &Ti, float &Td, int &pidDelay) {
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
    pidDelay = stoi(value);
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

    // if (numArgs == 5) {
    //     PIDmode = true;
    //     targetRPM = atof(args[1]);
    //     Pk = atof(args[2]);
    //     Ti = atof(args[3]);
    //     Td = atof(args[4]);
    //     cout << "arguments set\n";
    // }

    PIDmode = false;
    targetRPM = 2000;
    // Pk = 0.102;
    // Ti = 89.419;
    // Td = 55.887;
    readParams(Pk, Ti, Td, pidDelay);
    cout << "arguments set\n";

    //when CTRL+C is pressed, signalHandler will be invoked.
    signal(SIGINT, signalHandler);
    pid_ptr = new PID();
    pid_ptr->targetRPM = targetRPM;
    
    double dutyCycle = 20;
    pid_ptr->start(dutyCycle);
    GPIO::setup(18, GPIO::IN);
    GPIO::setup(11, GPIO::OUT, GPIO::LOW);
    GPIO::add_event_detect(18, GPIO::Edge::FALLING, blink, 10);

    int increment = 5;

    int runs = 0;
    while(!done){
        if (runs == 0) {
            delayMs(2000);      //one long delay to let the motor stabilize first
        }
        if (PIDmode) {
            delayMs(pidDelay);
            pid_ptr->updateError();
            dutyCycle = pid_ptr->pidControl(targetRPM, Pk, Ti, Td);
            pid_ptr->ChangeDutyCycle(dutyCycle);
        } else {
            delayMs(2000);
            dutyCycle += increment;
            cout << dutyCycle << '\n';
            if (dutyCycle >= 50){
                increment = -5;
            }
            if (dutyCycle <= 10){
                increment = 5;
            }
            pid_ptr->ChangeDutyCycle(dutyCycle);
            pid_ptr->updateError();
            // done = true;
            // if (pid_ptr->currentRPM >= 4500) {
            //     cout << "\n\n\n\n 4500RPM reached at " << dutyCycle << "\n\n\n\n";
            // }
        }

        
    }

    pid_ptr->rampDown(2);
    pid_ptr->dumpText();

    return 0;
}
