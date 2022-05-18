#include "mbed.h"
#include "Motor.h"
#include <cstdio>
#include "RC.h"
#include "Wheel.h"

Serial pc(USBTX, USBRX, 115200);
Wheel wheel(D10, D9, D6, D5, D4);       //pwm, fwd, rev, EncA, EncB

int _count;                             //declaration du compteur pour l'encodeur
int rev = 12;                           // nombre de tick sur l'encodeur
int dir = -1;                           //sens de direction du moteur par rapport a l'encodeur
float ratio = 34.014;                   //rapport de réduction

int freq_echnatillonnage = 50;          //frequence d'execution de l'asservissement

double a0=6.487*pow(10,5);              //coefficient de la fonction de transfert du moteur
double a1=5.322*pow(10,5);              //coefficient de la fonction de transfert du moteur
double a2=1;                            //coefficient de la fonction de transfert du moteur
double b0=2.466*pow(10,7)*100;          //coefficient de la fonction de transfert du moteur

double vReelMotor = 0;                  //vitesse réelle de notre moteur

float _Tq = 0.01;                       //fréquence d'échantillonnage pour update et afficher la vitesse
Ticker t;
Timer tm;

int main() {
    int stop = 1;                       //variable pour faire démarer la boucle while
    wheel.SetSpeed(30);
    printf("Temps / Vitesse\r\n");
    tm.start();
    int t0 = tm.read_ms();
    while (stop) {    
        int dt = tm.read_ms() - t0;
        if (dt >= _Tq*1000) {
            wheel.UpdateSpeed();
        }       
    }
}