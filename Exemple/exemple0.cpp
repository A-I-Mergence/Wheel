#include "mbed.h"
#include "Motor.h"
#include <cstdio>
#include "RC.h"

Serial pc(USBTX, USBRX, 115200);
Motor motor(D10, D9, D6, D5, D4);       //pwm, fwd, rev, EncA, EncB

int _count;                             //declaration du compteur pour l'encodeur
int rev = 12;                           // nombre de tick sur l'encodeur
int dir = -1;                           //sens de direction du moteur par rapport a l'encodeur
float ratio = 34.014;                   //rapport de réduction

float vitesse ;                         //vitesse de ma roue
float vit_PB;                           //vitesse filtrée de ma roue
float vit_last;                         //vitesse a l'instant t-1
float to2 = 0.005;                      //valeur pour le calcule de la vitesse filtrée

int freq_echnatillonnage = 50;          //frequence d'execution de l'asservissement

//Données pour le PID et le RC
double Input = vitesse;                 //valeur d'entrée du regulateur
double Output = 0;                      //valeur de sortie du régulateur
double Setpoint = 0;                    //valeur a atteindre (commande)

double Kp = 0.1217;                     //coefficient proportionnel
double Ki = 0.0435;                     //coefficient integral
double Kd = 0;                          //coefficient dérivé

double a0=6.487*pow(10,5);              //coefficient de la fonction de transfert du moteur
double a1=5.322*pow(10,5);              //coefficient de la fonction de transfert du moteur
double a2=1;                            //coefficient de la fonction de transfert du moteur
double b0=2.466*pow(10,7)*100;          //coefficient de la fonction de transfert du moteur

double vReelMotor = 0;                  //vitesse réelle de notre moteur

RC myRC(&Input, &Output ,&Setpoint);

float _Tq = 0.01;                       //fréquence d'échantillonnage pour update et afficher la vitesse
Ticker t;
bool ready = false;                     //variable pour print la vitesse du moteur
Timer tm;

void updateSpeed(){
    vReelMotor = motor.getSpeed();
    vit_PB = ((vReelMotor*_Tq)+(vit_last*to2))/(to2+_Tq);
    vit_last = vit_PB;
    _count = 0;                         //on remet le compteur a 0 a chaque fois qu'on a update la vitesse
    ready = true;
}

int main() {
    float cmd = 0;                      //commande du moteur
    int stop = 1;                       //variable pour faire démarer la boucle while
    float to = 0.1;                     //valeur pour le calcule du filtre du setpoint
    int vitesse_moteur=50;              //valeur voulue de notre moteur
    double in_last;                     //valeur a t-1 du setpoint
    int compt_chang = 0;                //compteur pour changer la vitesse voulue du moteur
   
    // myPID.SetMode(AUTOMATIC);
    myRC.SetMode(AUTOMATIC);
    myRC.SetParaMotor(a0, a1, a2, b0);

    printf("Temps / Vitesse\r\n");
    motor.speed(0); 
    
    t.attach(callback(&updateSpeed), _Tq);
    tm.start();
    int t0 = tm.read_ms();
    while (stop) {    
        int dt = tm.read_ms() - t0;
        if (dt >= _Tq*1000) {

            Setpoint = ((vitesse_moteur*_Tq)+(in_last*to))/(to+_Tq); //ammortissemnt de notre setpoint
            if (Setpoint > vitesse_moteur) {
            Setpoint = vitesse_moteur;
            }
            else {Setpoint =Setpoint;}

            Input = vit_PB;                                          //mise à jour de la vitesse réele
            myRC.Regule((float)dt/1000);                             //calcul de la regulation de vitesse
            cmd = Output;                                            //vise a jour du pwm envoyé au moteur
            motor.speed(cmd);                                        //donner la commande au moteur
            
            printf("%f %f %f %d \r\n", Setpoint, vit_PB, cmd, dt);

            tm.reset();
            compt_chang++;                                           //incrementation pour changement de vitsse
            t0 = tm.read_ms();

            if (compt_chang == 300) {                                //changement de la vitesse
            vitesse_moteur=90;
            }
            in_last = Setpoint;                                      //mise a jour de setpoint a t-1
        }       
    }
}