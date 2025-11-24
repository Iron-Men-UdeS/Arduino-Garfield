/*
Projet: Arduino Lasagne
Equipe: Iron Men
Description: Code pour l'arduino du RobUS controller par la manette
Date: 11/13/2025
*/

/* ****************************************************************************
Inclure les librairies de functions que vous voulez utiliser
**************************************************************************** */
#include "Capteurs.h"
#include "Mouvement.h"
#include "Niveau1.h"
/* ****************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */

int flagR = 0;
int flagV = 0;
int flagB = 0;
int flagJ = 0;
int flagS = 0;

int flagBumper=0;

int couleur = 0;
int flagRouge = 0;
int flagVert = 0;
int flagBleu = 0;
int flagJaune = 0;

unsigned long clockR = 0;
unsigned long clockV = 0;
unsigned long clockB = 0;
unsigned long clockJ = 0;
unsigned long clockN = 0;

/*******************************************************************************************
 * Auteur : Raphael
 *
 * Renvoie le flag rouge selon la couleur et les cooldown
 *
 * Pas de return mais joue sur la variable globale flagRouge et les clockN et clockR
 ******************************************************************************************/
void malusRouge()
{

  couleur = detectCouleur();
  clockN = millis();

  if (clockN - clockR > 5000)
  {
    flagRouge = 0;
  } // Durée du bonus/malus

  if (couleur == COULEURROUGE & (clockN - clockR > 10000 || clockR == 0))
  { // Cooldown
    flagRouge = 1;
    clockR = millis();
  }
}

/*******************************************************************************************
 * Auteur : Raphael
 *
 * Renvoie le flag vert selon la couleur et les cooldown
 *
 * Pas de return mais joue sur la variable globale flagVert et les clockN et clockV
 ******************************************************************************************/
void bonusVert()
{

  couleur = detectCouleur();
  clockN = millis();

  if (clockN - clockV > 5000)
  {
    flagVert = 0;
  } // Durée du bonus/malus

  if (couleur == COULEURVERT & (clockN - clockV > 10000 || clockV == 0))
  { // Cooldown
    flagVert = 1;
    clockV = millis();
  }
}

/*******************************************************************************************
 * Auteur : Raphael
 *
 * Renvoie le flag jaune selon la couleur et les cooldown
 *
 * Pas de return mais joue sur la variable globale flagJaune et les clockN et clockJ
 ******************************************************************************************/
void bananeJaune()
{

  couleur = detectCouleur();
  clockN = millis();
  Serial.print(couleur);

  if (couleur == COULEURJAUNE & (clockN - clockJ > 7000 || clockJ == 0))
  { // Cooldown
    flagJaune = 1;
    while (flagJaune == 1)
    {
      digitalWrite(LED_JAUNE, LOW);
      tourne(762, 0.4, DROITE);

      digitalWrite(LED_JAUNE, HIGH);
      flagJaune = 0;
      clockJ = millis();
    }
  }
  else if (couleur != COULEURJAUNE)
  {
    flagJaune = 0;
  }
}

/*******************************************************************************************
 * Auteur : Raphael
 *
 * Renvoie le flag bleu selon la couleur et les cooldown
 *
 * Pas de return mais joue sur la variable globale flagBleu et les clockN et clockB
 ******************************************************************************************/
void gelBleu()
{

  couleur = detectCouleur();
  clockN = millis();

  if (clockN - clockB > 5000)
  {
    flagBleu = 0;
  } // Durée du bonus/malus

  if (couleur == COULEURBLEU & (clockN - clockB > 10000 || clockB == 0))
  { // Cooldown
    flagBleu = 1;
    clockB = millis();
  }
}

/*******************************************************************************************
 * Auteur : Raphael
 *
 * Regarde quels flags des bonus/malus sont ON et allume la DEL durant la durée du flag
 *
 * Jaune pas inclus car dans sa fct
 *
 * Pas de return juste à mettre la fct dans le loop
 ******************************************************************************************/
void delBonus()
{
  if (flagBleu == 1)
  {
    digitalWrite(LED_BLEUE, LOW);
  }
  if (flagRouge == 1)
  {
    digitalWrite(LED_ROUGE, LOW);
  }
  if (flagVert == 1)
  {
    digitalWrite(LED_VERTE, LOW);
  }
  if (flagBleu == 0)
  {
    digitalWrite(LED_BLEUE, HIGH);
  }
  if (flagRouge == 0)
  {
    digitalWrite(LED_ROUGE, HIGH);
  }
  if (flagVert == 0)
  {
    digitalWrite(LED_VERTE, HIGH);
  }
}


/*******************************************************************************************
 * Auteur : Raphael
 *
 * Regarde l'état des bumpers
 * 
 * Défini flagBumper à 1 si un bumper est ON
 ******************************************************************************************/
  void flagBumperSet(){
    bool bumpp=false;
    if(ROBUS_IsBumper(0)){bumpp=true;}
  if(ROBUS_IsBumper(1)){bumpp=true;}
  if(ROBUS_IsBumper(2)){bumpp=true;}
  if(ROBUS_IsBumper(3)){bumpp=true;}
    if(bumpp){flagBumper=1;}
    if(!bumpp){flagBumper=0;}
}

void setup()
{

}

/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
void loop()
{
flagBumperSet();
malusRouge();
bonusVert();
gelBleu();
bananeJaune();
delBonus();

}

