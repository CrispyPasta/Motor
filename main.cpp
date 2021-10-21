#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <signal.h>
#include "./PID/PID.h"
#include <JetsonGPIO.h>

using namespace GPIO;
using namespace std;

//define motor control pins
int PWM_pin = 33;
int standby_pin = 19;
int direction_control1 = 21;
int direction_control2 = 23;

bool done = false;

void signalHandler(int s){
    done = true;
}

int main() {
    cout << "Hello, World!" << endl;
    //when CTRL+C is pressed, signalHandler will be invoked.
    signal(SIGINT, signalHandler);

    // GPIO::setmode(GPIO::BOARD);
    // GPIO::PWM* my_pwm = setupPWM(true);     //get pins ready
    // pid_object.ChangeDutyCycle(dutyCycle);
    // my_pwm->start(dutyCycle);      //start out at 50%
    
    PID pid_object;
    double dutyCycle = 10;
    pid_object.start(dutyCycle);

    int increment = 5;

    while(!done){
        std::this_thread::sleep_for(std::chrono::milliseconds(750));
        if (dutyCycle >= 50){
            increment = -5;
        }
        if (dutyCycle <= 0){
            increment = 5;
        }
        dutyCycle += increment;
        // my_pwm->ChangeDutyCycle(dutyCycle);
        pid_object.ChangeDutyCycle(dutyCycle);
        cout << " Duty Cycle = " << setprecision(3) << (pid_object.getDutyCycle()) << '\n';
    }

    // cleanupPins();

    return 0;
}
