#ifndef WHEEL_H
#define WHEEL_H

#include "Motor.h"
#include "RC.h"
#include "mbed.h"

class Wheel
{
public:
    
    Wheel(PinName pwm, PinName fwd, PinName rev, PinName EncA, PinName EncB);

    void SetSpeed(float VitesseVoulue);
    float getSpeed(){ return _motor->getSpeed(); }

    float FilterSpeed(float);

    void StartRegule();
    void StopRegule();

    void UpdateSpeed();

    void SetPWM(float);

    void setAcceleration(float to);
    void setBraking(float to);



    float getCommadePWM(){ return W_cmd; }

    float getFilteredMeasurement(){ return filteredMeasurement;}
    float getResultatLissage(){ return W_Setpoint[1]; }

    float getPreCommande(){ return _rc->getPreCommande(); }
    float getError(){ return _rc->getError(); }
    float getRC(){ return _rc->getRC(); }
    float getRC1s(){return _rc->getRC1s(); }

private:
    Motor *_motor;
    RC *_rc;
    Ticker *_t;

    float _toAcceleration, _toBraking;

    float W_Setpoint[2];
    float W_SetpointRC;
    float W_Input_RC;
    float W_Output_RC;
    float W_VitesseVoulue;
    float W_Vitesse;
    float W_cmd;
    float _cmd;

    float W_Tq;
    float  W_vReelMotor;
    float filteredMeasurement;
    float mes_filter_last;
    float W_count;

    bool ReguleActivated;

};
#endif