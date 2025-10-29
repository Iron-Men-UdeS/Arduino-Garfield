#ifndef NIVEAU1_H
#define NIVEAU1_H

//Includes 
#include <Arduino.h>
#include <LibRobus.h>
#include "Capteurs.h"
#include "Mouvement.h"
#include "Capteurs.h"
// Defines
#define INTERVALLE 100
#define VITESSE_MOTEUR 0.2
#define AVANT 1
#define ARRIERE 2
#define HAUT 3
#define BAS 4
#define AUCUN -1
#define BRAS_DROIT 1
#define BRAS_GAUCHE 0
#define TIME 3000
// Variables externe

// //Prototypes de fonctions
int suivreLigne(void);
void bleu();
void vert();
void bougeBrasDroit(int position);
void bougeBrasGauche(int position);
void bouge2Bras(int posGauche,int posDroit);
void randomDEL();
void rouge();
void changeRobot(int direction);
void rouge2();
void jauneAntoine();
#endif