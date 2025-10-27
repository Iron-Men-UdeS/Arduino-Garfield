#ifndef CAPTEUR_H
#define CAPTEUR_H
#include <LibRobus.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "Adafruit_TCS34725.h"
#include "LibRobus.h"
// Defines

//Suiveur de ligne
#define CAPTEUR0_GAUCHE A0
#define CAPTEUR0_CENTRE A1
#define CAPTEUR0_DROITE A2
#define CAPTEUR1_GAUCHE A3
#define CAPTEUR1_CENTRE A4
#define CAPTEUR1_DROITE A5

//DELS
#define LED_ROUGE 46
#define LED_VERTE 44
#define LED_JAUNE 47
#define LED_BLEUE 45

//Capteur couleur
#define couleurRouge 0
#define couleurVert 1
#define couleurBleu 2
#define couleurJaune 3

//Capteur d'obstacle
#define PIN_DIST_G 48 
#define PIN_DIST_D 49

//Capteur de sifflet
#define BRUIT_AMBIENT A0
#define SIGNAL_5kHz A1

//Capteur de distance
#define DISTANCEA A2
#define DISTANCEC A3
// Variables


// //Prototypes de fonctions
int calibrerGauche(void);
int calibreCentre(void);
int calibrationDroite(void);
void calibrationTotale(void);
int lireCapteurs(int capteur);
void initCapteurCouleur(void);
int detectCouleur();
void inverseDEL(int pin);
void eteindreToutesLesDELs(void);
bool mur();
bool sifflet_5kHz();
float detecDistance(int pin);
float detecDistanceLin(int pin);
float corr(int pin, float valeurCapteur);
#endif