
// Code de l'utilisation des roues en utlisant la réguation RC ainsi que l'acceleration et le freinage

#include "mbed.h"
#include "Motor.h"
#include <cstdio>
#include "RC.h"
#include "Wheel.h"

Serial pc(USBTX, USBRX, 115200);
Wheel wheel(D10, D9, D6, D5, D4);       //pwm, fwd, rev, EncA, EncB

float Tq = 0.01;                       //fréquence d'échantillonnage pour update et afficher la vitesse

bool loopAccepted = false;
Ticker tick;

void acceptLoop(){
    loopAccepted = true;
}

float desiredSpeedRPM = 0;
float actualSpeedPRM = 0;

int main() {
    
    wheel.setAcceleration(40);
    wheel.setBraking(100);

    wheel.StartRegule();

    Timer tim;
    tim.start();
    tim.reset();
    int t0_ms = tim.read_ms();

    tick.attach(&acceptLoop, Tq);

    while (1) {
        int t1_ms = tim.read_ms();
        if (t1_ms - t0_ms < 1000)       desiredSpeedRPM = 0;
        else if (t1_ms - t0_ms < 5000)  desiredSpeedRPM = 50;
        else if (t1_ms - t0_ms < 10000)  desiredSpeedRPM = -100;
        else if (t1_ms - t0_ms < 15000)  desiredSpeedRPM = 30;
        else if (t1_ms - t0_ms < 20000) desiredSpeedRPM = 0;
        else break;

        if (loopAccepted) {
            loopAccepted = false;
            wheel.SetSpeed(desiredSpeedRPM);

            actualSpeedPRM = wheel.getSpeed();
            float actualFilteredMeasurement = wheel.getFilteredMeasurement();
            float actualLissage = wheel.getResultatLissage();

            float actualPreCommande = wheel.getPreCommande();
            float actualError = wheel.getError();
            float actualRC = wheel.getRC();
            float actualRC1S = wheel.getRC1s();

            float actualCommadePWM = wheel.getCommadePWM();


            printf("t: %.2f\t", (t1_ms - t0_ms)/1000.0);
            printf("Vdes: %.2f\t", desiredSpeedRPM);
            printf("Vmes: %.2f\t", actualSpeedPRM);
            printf("VmesF: %.2f\t", actualFilteredMeasurement);
            printf("VdesL: %.2f\t", actualLissage);
            printf("Precom: %.2f\t", actualPreCommande);
            printf("Err: %.2f\t", actualError);
            printf("RC: %.2f\t", actualRC);
            printf("RC1S: %.2f\t", actualRC1S);
            printf("PWM: %.2f\t", actualCommadePWM);
            
            printf("\r\n");
            
        }
    }
}