#ifndef WHEEL_H
#define WHEEL_H

#include "Motor.h"
#include "RC.h"
#include "mbed.h"

class Wheel
{
public:
    
    Wheel(PinName pwm, PinName fwd, PinName rev, PinName EncA, PinName EncB);

    void SetSpeedRPM(double VitesseVoulue);

    float FilterSpeed(float);

    void StartRegule();
    void StopRegule();

    void SetAcceleration(float);
    void Acceleration();
    void SetFreinage(float);
    void Freinage();

    void UpdateSpeed();

    void SetPWM(float);

    float GetCommande();

    float GetSpeed();

    void Stop();

private:
    Motor *_motor;
    RC *_rc;
    Ticker *_t;

    double W_Setpoint[2];
    double W_Input_RC;
    double W_Output_RC;
    double W_VitesseVoulue;
    float W_Vitesse;
    float W_cmd;
    float _cmd;

    float W_Tq;
    double W_vReelMotor;
    float mes_filter_last;
    double W_count;

    bool ReguleActivated;

    int _k;
    float _to_acc;
    float _to_fre;

};
#endif