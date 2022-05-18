#include "Motor.h"
#include "RC.h"
#include "mbed.h"

class Wheel 
{ 
public: 
    Timer Wheeltimer;
    Wheel(PinName pwm, PinName fwd, PinName rev, PinName EncA, PinName EncB);

    void SetSpeed(double VitesseVoulue);

    void FilterSpeed();

    void UpdateSpeed();

    float GetSpeed();

private:
    Motor *_motor;
    RC *_rc;
    Timer *_tm;

    double W_Setpoint;
    double W_Input;
    double W_Output;
    double W_VitesseVoulue;
    float W_Vitesse;
    float W_cmd;

    float W_Tq;  
    float  W_to2;
    float W_to;
    double  W_vReelMotor;
    double W_vit_PB;
    float W_vit_last;
    double W_in_last;
    double W_count;
    
};
// #endif