#include "Motor.h"
#include "RC.h"
#include "mbed.h"

class Wheel
{
public:
    
    Wheel(PinName pwm, PinName fwd, PinName rev, PinName EncA, PinName EncB);

    void SetSpeed(double VitesseVoulue);

    float FilterSpeed(float);

    void UpdateSpeed(float dt);

private:
    Motor *_motor;
    RC *_rc;

    double W_Setpoint[2];
    double W_Input_RC;
    double W_Output_RC;
    double W_VitesseVoulue;
    float W_Vitesse;
    float W_cmd;

    float W_Tq;
    double  W_vReelMotor;
    float mes_filter_last;
    double W_count;

};
// #endif