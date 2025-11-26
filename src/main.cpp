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
#include "Communication.h"
/* ****************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */
uint8_t tabGarfield[2] = {5,6};
uint8_t tabLasagne[4] = {33,33,33,33};
void setup()
{
    initUART1();
    Serial.begin(9600);
}

/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
void loop()
{
    envoieTrame(tabGarfield);
    delay(100);
    if(litUART(tabLasagne, 6))
    {
        Serial.print(tabLasagne[0]);
        Serial.print(tabLasagne[1]);
        Serial.print(tabLasagne[2]);
        Serial.print(tabLasagne[3]);
    }
    // Serial.print(trame[2]);
    // Serial.print(trame[3]);
    // Serial.println(trame[4]);
}

