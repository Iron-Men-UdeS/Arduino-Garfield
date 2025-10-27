#ifndef NIVEAU1_H
#define NIVEAU1_H

//Includes 
#include <Arduino.h>
// Defines
#define INTERVALLE 100
#define VITESSE_MOTEUR 0.5
#define AVANT 1
#define ARRIERE 2
#define HAUT 3
#define BAS 4
#define AUCUN -1
#define DROITE
#define GAUCHE 
// Variables externe


// //Prototypes de fonctions
int suivreLigne(void);
void bleu();
void vert();
void bougeBrasDroit(int position);
void bougeBrasGauche(int position);
void bouge2Bras(int posGauche,int posDroit);
#endif