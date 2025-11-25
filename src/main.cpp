/*
Projet: Arduino Lasagne
Equipe: Iron Men
Description: Code pour l'arduino du RobUS controller par la manette
Date: 11/13/2025
*/

/* ****************************************************************************
Inclure les librairies de functions que vous voulez utiliser
**************************************************************************** */
#include "main.h"
#include <Arduino.h>
#include "Capteurs.h"
#include "mouvement.h"
unsigned long cooldown=0;
int start=0;
double t_init=0;
double t_main=0;
/* ****************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */
position robot;
void setup()
{
    BoardInit();
    //mpu_init(0);
}
/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
void loop()
{
    if(start==1){
        if((t_main-t_init)<=2){
            vitesseRoues(0.3,0.3);
        }
        if((t_main-t_init)>1 && robot.angle<PI){
            vitesseRoues(0,0.3);
        }
        if(robot.angle>PI&&robot.y>0){
            vitesseRoues(0.3,0.3);
        }
        if(robot.angle>PI&&robot.y<0){
            vitesseRoues(0,0);
        }
        t_main=get_temps();
        if (t_main>=cooldown){
        // Serial.print("acc x: ");
        // Serial.println(mpu_get(0,0));
        // Serial.print("acc y:");
        // Serial.println(mpu_get(1,0));
        // Serial.print("acc z:");
        // Serial.println(mpu_get(2,0));
        // Serial.print("\n");
        // Serial.print("rot z:");
        // Serial.println(mpu_get(2,1));
        //Serial.print("position x: ");
        //Serial.println(get_pos(0));
        //Serial.print("position y: ");
        //Serial.println(get_pos(1));
        // Serial.print("orientation: ");
        // Serial.println(get_or());
        // Serial.print("temps: ");
        // Serial.println(get_temps());
        // Serial.print("\n");

        cooldown+=1;
        }
        actu_angle(robot);
}
    if (ROBUS_IsBumper(0)==true&&start==0){
        start=1;
        initialiserTemps();
        t_main=get_temps();
        t_init=t_main;
    }
    delay(10);
}

