#ifndef MOUVEMENT_H
#define MOUVEMENT_H

#include <Arduino.h>
#include <LibRobus.h>

//math
#define CIRCON_ROUE_CM 23.939
#define TOUR_COMPLET_ENCO 3200.0
#define CASE 50.0
#define QUART_DE_TOUR 93.04//parfait pour tourner a droite
#define TOUR_COMPLET 378.9//parfait pour tourner a droite
#define DIST_ROUE 19 //distance entre les 2 roues

//PID
#define KP 0.00003
#define KI 0.0
#define KD 0.00002

//Autre
#define GAUCHE false
#define DROITE true


struct vecteur{
  double norme = 0;
  double angle = 0;
  double x = 0;//TODO mettre les valeur d'origine du robot par rapport au coin
  double y = 0;
};
uint32_t distanceEnco(float distanceCM);
uint32_t angleEnco(float angleDeg);
double encoToCM(float distanceEnco);
double calculVitesse(float maxSpeed, uint32_t position, uint32_t positionFinal);
double pid(float error, float &lastError);
bool mur();
void robotSetSpeed(float vitesse, int direction ,float &correction);
void vitesseRoues(float vitesse1, float vitesse2);
void avance(int distanceCM, float vitesse);
void tourne(float angleDeg, float vitesse, bool direction);
void vitesseRoues(float vitesse1, float vitesse2);
double angleArc(int encodeur);
double normeDeplacement(double angle);
double angleDeplacement(double angle);
void positionRobot(vecteur& positionRobot, double gyro);
#endif 


