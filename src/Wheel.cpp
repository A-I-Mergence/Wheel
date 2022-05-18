#include "Wheel.h"
#include "Motor.h"
#include "RC.h"
#include "mbed.h"
#include <cstdio>

Wheel::Wheel(PinName pwm, PinName fwd, PinName rev, PinName EncA, PinName EncB)
{
    _motor = new Motor(pwm, fwd, rev, EncA, EncB);       //pwm, fwd, rev, EncA, EncB
    _rc = new RC(&W_Input, &W_Output, &W_Setpoint);
    _tm = new Timer();

    W_Tq = 0.01;
    W_to2 = 0.005;
    W_to = 0.1;
    W_vReelMotor = 0;
}

void Wheel::SetSpeed(double VitesseVoulue){
    W_VitesseVoulue = VitesseVoulue;
}

void Wheel::FilterSpeed(){
    W_vReelMotor = _motor->getSpeed();
    W_vit_PB = ((W_vReelMotor*W_Tq)+(W_vit_last*W_to2))/(W_to2+W_Tq);
    W_vit_last = W_vit_PB;
    W_count = 0;
}

void Wheel::UpdateSpeed(){
    int W_t0 = _tm->read_ms();
    int W_dt = _tm->read_ms() - W_t0;
    // if (W_dt >= W_Tq*1000) {

        W_Setpoint = ((W_VitesseVoulue*W_Tq)+(W_in_last*W_to))/(W_to+W_Tq); //ammortissement de notre setpoint
        if (W_Setpoint > W_VitesseVoulue) {
        W_Setpoint = W_VitesseVoulue;
        }
        // else {W_Setpoint = W_Setpoint;}

        W_Input = W_vit_PB;                                          //mise à jour de la vitesse réele
        _rc->Regule((float)W_dt/1000);                               //calcul de la regulation de vitesse
        W_cmd = W_Output;                                            //vise a jour du pwm envoyé au moteur
        _motor->speed(W_cmd);                                        //donner la commande au moteur
        
        printf("%f %f %f %d \r\n", W_Setpoint, W_vit_PB, W_cmd, W_dt);

        _tm->reset();
        W_t0 = _tm->read_ms();
        W_in_last = W_Setpoint;  
    // }
}
// float GetSpeed(){
//     return _motor->speed(W_cmd); 
// }
