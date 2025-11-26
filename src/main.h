#ifndef MAIN_H
#define MAIN_H


struct position{
  double x = 0;
  double y = 0;
  double angle = 0;
};

//Includes
#include <LibRobus.h> // Essentielle pour utiliser RobUS
#include "Capteurs.h" // Capteurs ajoutes sur RobUS
#include "Mouvement.h"
#include "Niveau1.h"
#endif