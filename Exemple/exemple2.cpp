
// Code de l'utilisation des roues en utlisant la réguation RC

#include "mbed.h"
#include "Motor.h"
#include <cstdio>
#include "RC.h"
#include "Wheel.h"

Serial pc(USBTX, USBRX, 115200);
Wheel wheel(D3, D9, D6, D5, D4);       //pwm, fwd, rev, EncA, EncB

float _Tq = 0.01;                       //fréquence d'échantillonnage pour update et afficher la vitesse
Ticker t;
Timer tm;

double VitesseVoulue = 30;

int main() {
    
    wheel.StartRegule();
    tm.start();
    int t0 = tm.read_ms();

    while (1) {
        wheel.SetSpeed(VitesseVoulue);
        int dt = tm.read_ms() - t0;
        if (dt >= _Tq*1000) {
            tm.reset();
            t0 = tm.read_ms();
        }       
    }
}