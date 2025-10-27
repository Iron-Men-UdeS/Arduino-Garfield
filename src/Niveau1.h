#ifndef NIVEAU1_H
#define NIVEAU1_H

//Includes 
#include <Arduino.h>
#include <LibRobus.h>
#include "Capteurs.h"
#include "Mouvement.h"
// Defines
#define INTERVALLE 100
#define VITESSE_MOTEUR 0.5
// Variables externe


// //Prototypes de fonctions
int suivreLigne(void);
void rouge();
void changeRobot(int direction);
#endif