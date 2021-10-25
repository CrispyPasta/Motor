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

inline void delayMs(int ms) { this_thread::sleep_for(chrono::milliseconds(ms)); }


//define motor control pins
int PWM_pin = 33;
int standby_pin = 19;
int direction_control1 = 21;
int direction_control2 = 23;

bool done = false;

PID* pid_ptr = nullptr;

int rotations = 0;
auto rotEnd = steady_clock::now();
auto rotStart = steady_clock::now();
auto rotDuration = duration_cast<milliseconds>(rotEnd-rotStart);

void signalHandler(int s){
    done = true;
}

void blink(int channel)
{
    cout << "Rotations: " << to_string(++rotations) << '\n';
    rotEnd = steady_clock::now();
    rotDuration = duration_cast<milliseconds>(rotEnd - rotStart);
    rotStart = steady_clock::now();
    cout << "Rotation duration: " << rotDuration.count() << " milliseconds\n";
    float rpm = 60000.0 / rotDuration.count();
    cout << "RPM: " << to_string(rpm) << '\n';
}

int main() {
    cout << "Hello, World!" << endl;
    //when CTRL+C is pressed, signalHandler will be invoked.
    signal(SIGINT, signalHandler);
    pid_ptr = new PID();

    // GPIO::setmode(GPIO::BOARD);
    // GPIO::PWM* my_pwm = setupPWM(true);     //get pins ready
    // pid_object.ChangeDutyCycle(dutyCycle);
    // my_pwm->start(dutyCycle);      //start out at 50%
    
    double dutyCycle = 25;
    pid_ptr->start(dutyCycle);
    GPIO::setup(18, GPIO::IN);
    GPIO::setup(11, GPIO::OUT, GPIO::LOW);
    GPIO::add_event_detect(18, GPIO::Edge::FALLING, blink, 10);

    int increment = 5;

    while(!done){
        // delayMs(2000);
        // GPIO::output(11, GPIO::HIGH);
        delayMs(750);
        // GPIO::output(11, GPIO::LOW);
        if (dutyCycle >= 50){
            increment = -5;
        }
        if (dutyCycle <= 0){
            increment = 5;
        }
        dutyCycle += increment;
        // my_pwm->ChangeDutyCycle(dutyCycle);
        pid_ptr->ChangeDutyCycle(dutyCycle);
        // cout << " Duty Cycle = " << setprecision(3) << (pid_ptr->getDutyCycle()) << '\n';
    }

    // GPIO::remove_event_detect(10);

    // cleanupPins();
    GPIO::cleanup();

    return 0;
}
