#ifndef MOUVEMENT_H
#define MOUVEMENT_H

#include <Arduino.h>
#include <LibRobus.h>
//math
#define CIRCON_ROUE_CM 23.939
#define TOUR_COMPLET 3200.0
#define CASE 50.0

//PID
#define KP 1.0
#define KI 0.0
#define KD 0.0

//Autre
#define GAUCHE false
#define DROITE true



uint32_t distanceEnco(float distanceCM);
uint32_t angleEnco(int angleDeg);
double calculVitesse(float maxSpeed, uint32_t position, uint32_t positionFinal);
double pid(float error, float &lastError);
bool mur();
void robotSetSpeed(float vitesse, int direction ,float &correction);

#endif 


