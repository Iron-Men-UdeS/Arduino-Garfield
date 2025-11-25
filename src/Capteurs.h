#ifndef CAPTEUR_H
#define CAPTEUR_H
#include <LibRobus.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Defines

//Suiveur de ligne
#define CAPTEUR0_GAUCHE A8
#define CAPTEUR0_CENTRE A9
#define CAPTEUR0_DROITE A10
#define CAPTEUR1_GAUCHE A11
#define CAPTEUR1_CENTRE A12
#define CAPTEUR1_DROITE A13

//DELS
#define LED_ROUGE 46
#define LED_VERTE 44
#define LED_JAUNE 47
#define LED_BLEUE 45

//Capteur couleur
#define COULEURROUGE 0
#define COULEURVERT 1
#define COULEURBLEU 2
#define COULEURJAUNE 3

//Capteur d'obstacle
#define PIN_DIST_G 48 
#define PIN_DIST_D 49

//Capteur de sifflet
#define BRUIT_AMBIENT A0
#define SIGNAL_5kHz A1

//Capteur de distance
#define DISTANCEA A2
#define DISTANCEC A3
//accéléromètre
#define POSX_R1 0;
#define POSY_R2 0;
#define ERR_ACCX_R1 0.37;
#define ERR_ACCY_R1 0.2;
#define ERR_ROT_R1 -4;
#define DIST 18.75;
#define CMPT 23.93895
// Variables

// struct suiveur{
//   int pinGauche,pinDroite,pinCentre;
//   int seuilCentre;
//   int seuilDroite;
//   int seuilGauche;
//   int readCentre,readDroite,readGauche;
// };

// //Prototypes de fonctions
int calibrerGauche(void);
int calibreCentre(void);
int calibrationDroite(void);
//void calibrationTotale(void);
int lireCapteurs(int capteur);
void initCapteurCouleur(void);
int detectCouleur();
void inverseDEL(int pin);
void eteindreToutesLesDELs(void);
bool mur();
bool sifflet_5kHz();
float detecDistance(int pin);
//float detecDistanceLin(int pin);
float corrDist(int pin, float valeurCapteur);// a remettre
//float calibreSuiveur(int pin);
//int lireSuiveur(struct suiveur);
void mpu_init(int calibration);
double mpu_lect(int axe, int mode);
void actu_pos();
void actu_pos2();
double get_or();
double get_pos(int axe);
double get_temps();
double mpu_get(int axe, int mode);
void actu_pos3();
void actu_pos4();
void actu_pos5();
void initialiserTemps();
#endif