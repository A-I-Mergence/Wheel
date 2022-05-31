
// Code de l'utilisation des roues sans utliser la réguation RC

#include "mbed.h"
#include "Motor.h"
#include <cstdio>
#include "RC.h"
#include "Wheel.h"

Serial pc(USBTX, USBRX, 115200);
Wheel wheel(D10, D9, D6, D5, D4);       //pwm, fwd, rev, EncA, EncB

float _Tq = 0.01;                       //fréquence d'échantillonnage pour update et afficher la vitesse
Ticker t;
Timer tm;

float CommandePWM = 0.5;

int main() {

    wheel.StopRegule();
    tm.start();
    int t0 = tm.read_ms();

    while (1) {
        int dt = tm.read_ms() - t0;
        if (dt >= _Tq*1000) {
            tm.reset();
            t0 = tm.read_ms();
            // si nous ne sommes pas dans le mode StopRegule la commande suivante ne pourra pas etre executé
            wheel.SetPWM(CommandePWM);
        }       
    }
}