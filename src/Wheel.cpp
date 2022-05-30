#include "Wheel.h"
#include "Motor.h"
#include "RC.h"
#include "mbed.h"
#include <cstdio>

Wheel::Wheel(PinName pwm, PinName fwd, PinName rev, PinName EncA, PinName EncB)
{
    _motor = new Motor(pwm, fwd, rev, EncA, EncB);       //pwm, fwd, rev, EncA, EncB
    _rc = new RC(&W_Input_RC, &W_Output_RC, &W_Setpoint[1]);
    
   
    int dir = -1;                           //sens de direction du moteur par rapport a l'encodeur
    int enco = 12;                          // nombre de tick sur l'encodeur
    float ratio = 34.014;                   //rapport de réduction
    _motor->MotorSetup(dir, enco, ratio);

    _rc->SetMode(AUTOMATIC);
    double a0=6.487*pow(10,5);              //coefficient de la fonction de transfert du moteur
    double a1=5.322*pow(10,5);              //coefficient de la fonction de transfert du moteur
    double a2=1;                            //coefficient de la fonction de transfert du moteur
    double b0=2.466*pow(10,7)*100;          //coefficient de la fonction de transfert du moteur
    _rc->SetParaMotor(a0, a1, a2, b0);

    W_Tq = 0.01;
    W_vReelMotor = 0;
    W_VitesseVoulue = 0;
    ReguleActivated = false;

    _t = new Ticker;
}

void Wheel::SetSpeedRPM(double VitesseVoulue){
    W_VitesseVoulue = VitesseVoulue;
}

float Wheel::FilterSpeed(float mesures){
    float  to2 = 0.005;
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

void Wheel::SetAcceleration(float to_acc){
    _to_acc = to_acc;
}
void Wheel::Acceleration(){
    
    W_Setpoint[_k] = ((W_VitesseVoulue*W_Tq)+(W_Setpoint[_k-1]*_to_acc))/(_to_acc+W_Tq); //ammortissement 
}

void Wheel::SetFreinage(float to_fre){
    _to_fre = to_fre;
}
void Wheel::Freinage(){
    W_Setpoint[_k] = ((W_VitesseVoulue*W_Tq)+(W_Setpoint[_k-1]*_to_fre))/(_to_fre+W_Tq); //ammortissement 
}

void Wheel::UpdateSpeed(){
    int k=1;
    // float to = 0.1;
    float mes = _motor->GetSpeed();
    float mes_filter = FilterSpeed(mes);

    if (mes<W_VitesseVoulue){
        Freinage();
    }
    else {
        Acceleration();
    }
    // W_Setpoint[k] = ((W_VitesseVoulue*W_Tq)+(W_Setpoint[k-1]*to))/(to+W_Tq); //ammortissement de notre setpoint
    // if (W_Setpoint[k] > W_VitesseVoulue) {
    //     W_Setpoint[k] = W_VitesseVoulue;
    // }
    
    W_Input_RC = mes_filter;                                       //mise à jour de la vitesse réele
    _rc->Regule(W_Tq);                                               //calcul de la regulation de vitesse
    W_cmd = W_Output_RC;                                           //vise a jour du pwm envoyé au moteur
    W_cmd = W_cmd > 1 ? 1 : (W_cmd < -1 ? -1 : W_cmd);
    _motor->SetSpeedPWM(W_cmd);                                          //donner la commande au moteur

    // printf("%f %f %f %.3f \r\n", W_Setpoint[k], mes_filter, W_cmd, dt);

    W_Setpoint[k-1] = W_Setpoint[k];
}

void Wheel::SetPWM(float pwm){
    if (ReguleActivated) return;
    pwm = pwm > 1 ? 1 : (pwm < -1 ? -1 : pwm);
    _motor->SetSpeedPWM(pwm);    
}

float Wheel::GetCommande(){
    return _rc->GetCommande();
}

float Wheel::GetSpeed(){
    return _motor->GetSpeed();
}

void Wheel::Stop(){
    _motor->Stop();
}