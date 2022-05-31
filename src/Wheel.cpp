#include "Wheel.h"
#include "Motor.h"
#include "RC.h"
#include "mbed.h"
#include <cstdio>

Wheel::Wheel(PinName pwm, PinName fwd, PinName rev, PinName EncA, PinName EncB)
{
    _motor = new Motor(pwm, fwd, rev, EncA, EncB);       //pwm, fwd, rev, EncA, EncB
    _rc = new RC(&W_Input_RC, &W_Output_RC, &W_SetpointRC);
    
   
    int dir = -1;                           // sens de direction du moteur par rapport a l'encodeur
    int enco = 12;                          // nombre de tick sur l'encodeur
    float ratio = 34.014;                   // rapport de réduction
    _motor->MotorSetup(dir, enco, ratio);

    _rc->SetMode(AUTOMATIC);
    float a0=805;              //coefficient de la fonction de transfert du moteur
    float a1=82.38;              //coefficient de la fonction de transfert du moteur
    float a2=1;                            //coefficient de la fonction de transfert du moteur
    float b0=1.629*pow(10,5);          //coefficient de la fonction de transfert du moteur
    _rc->ParaMotor(a0, a1, a2, b0);
    _rc->setParamRC(0.2);

    W_Tq = 0.01;
    W_vReelMotor = 0;
    W_VitesseVoulue = 0;
    ReguleActivated = false;

    setAcceleration(100);
    setBraking(100);

    _t = new Ticker;
}

void Wheel::SetSpeed(float VitesseVoulue){
    W_VitesseVoulue = VitesseVoulue;
}

void Wheel::setAcceleration(float to){
    to = to > 100 ? 100 : to < 0 ? 0 : to;
    _toAcceleration = (1 - to/100.0);
}

void Wheel::setBraking(float to){
    to = to > 100 ? 100 : to < 0 ? 0 : to;
    _toBraking = (1 - to/100.0);
}

float Wheel::FilterSpeed(float mesures){
    float  to2 = 0.01;
    float mesures_filter = ((mesures*W_Tq)+(mes_filter_last*to2))/(to2+W_Tq);
    mes_filter_last = mesures_filter;
    W_count = 0;
    return mesures_filter;
}

void Wheel::StartRegule(){
    ReguleActivated = true;
    _t->attach(callback(this, &Wheel::UpdateSpeed), W_Tq);
}

void Wheel::StopRegule(){
    ReguleActivated = false;
    _t->detach();
}

//                                                                  RC Regulator
//                     Smoothing               _______________________________________________
//                   -----------              |  -----------                     -----------  |          -----------
// W_VitesseVoulue  |     1     | W_Setpoint  | |     1     |         +         | r1*s + r0 | | W_cmd   |           | mes
// ---------------->|-----------|-------------->|-----------|------>O---------->|-----------|---------->|   MOTOR   |-------o----->
//                  | to*s + 1  |             | | r1*s + r0 |       ^ -         | c1*s + c0 | |         |           |       |
//                   -----------              |  -----------        |            -----------  |          -----------        |
//                                            |                     |                         |                             |
//                                            |                     |                         |              -----------    |
//                                            |                     |                         |  mes_filter |           |   |
//                                            |                     ----------------------------------------|   FILTER  |<---
//                                            |_______________________________________________|             |           |
//                                                                                                           -----------

         
//          |     ^  __ |
//          |     | /   |
// ---------| -- --- -> |
//          | __/ |     |
//          |___________|



void Wheel::UpdateSpeed(){
    int k=1;
    float to = 0.1;

    // Get measurement
    float mes = _motor->getSpeed();
    // Filtering of measurement
    // float mes_filter = mes;
    float mes_filter = FilterSpeed(mes);
    
    filteredMeasurement = mes_filter;

    //  Command smoothing (acceleration or braking)
    if (W_VitesseVoulue == 0)
        W_Setpoint[k] = ((W_VitesseVoulue*W_Tq)+(W_Setpoint[k-1]*_toBraking))/(_toBraking+W_Tq);
    else
        W_Setpoint[k] = ((W_VitesseVoulue*W_Tq)+(W_Setpoint[k-1]*_toAcceleration))/(_toAcceleration+W_Tq);
    

    if (W_Setpoint[k] < 20 && W_Setpoint[k] > -20) {
    W_SetpointRC = 0;
    }
    else {
    W_SetpointRC =  W_Setpoint[1];
    }

    W_Input_RC = mes_filter;                                        // mise à jour de la vitesse réele
    W_cmd = _rc->computeCommande(W_Tq, mes_filter);
    W_cmd = W_cmd > 1 ? 1 : W_cmd < -1 ? -1 : W_cmd;

    // Minimal motor speed treatement
    if (abs(W_Setpoint[k]) < 20){
        _rc->reset();
        W_cmd = 0;
    }

    _motor->speed(W_cmd);                                          // donner la commande au moteur

    // printf("%f %f %f %.3f \r\n", W_Setpoint[k], mes_filter, W_cmd, dt);

    W_Setpoint[k-1] = W_Setpoint[k];
}

void Wheel::SetPWM(float pwm){
    if (ReguleActivated) return;
    pwm = pwm > 1 ? 1 : (pwm < -1 ? -1 : pwm);
    _motor->speed(pwm);    
}