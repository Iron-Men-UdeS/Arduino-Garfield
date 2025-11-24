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
uint8_t trame[6] = {0,0,0,0,0,0};

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
    litUART(trame, 6);
    Serial.print(trame[0]);
    Serial.print(trame[1]);
    Serial.print(trame[2]);
    Serial.print(trame[3]);
    Serial.println(trame[4]);
}

