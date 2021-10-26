#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <signal.h>
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

int main(int numArgs, char* args[]) {
    cout << "Hello, World!" << endl;
    GPIO::setwarnings(false);
    float targetRPM = 0;
    float Pk;
    float Pi;
    float Pd;

    if (numArgs == 5) {
        PIDmode = true;
        targetRPM = atof(args[1]);
        Pk = atof(args[2]);
        Pi = atof(args[3]);
        Pd = atof(args[4]);
        cout << "arguments set\n";
    }

    //when CTRL+C is pressed, signalHandler will be invoked.
    signal(SIGINT, signalHandler);
    pid_ptr = new PID();
    pid_ptr->targetRPM = targetRPM;
    
    double dutyCycle = 10;
    pid_ptr->start(dutyCycle);
    GPIO::setup(18, GPIO::IN);
    GPIO::setup(11, GPIO::OUT, GPIO::LOW);
    GPIO::add_event_detect(18, GPIO::Edge::FALLING, blink, 10);

    int increment = 5;

    while(!done){
        delayMs(750);
        if (PIDmode) {
            dutyCycle = pid_ptr->pidControl(targetRPM, Pk, Pi, Pd);
        } else {
            dutyCycle += increment;
        }

        pid_ptr->ChangeDutyCycle(dutyCycle);
        if (dutyCycle >= 100){
            increment = -5;
        }
        if (dutyCycle <= 0){
            increment = 5;
        }
        // if (pid_ptr->currentRPM >= 4500) {
        //     cout << "\n\n\n\n 4500RPM reached at " << dutyCycle << "\n\n\n\n";
        // }
    }

    pid_ptr->rampDown(4);
    pid_ptr->dumpText();

    return 0;
}
