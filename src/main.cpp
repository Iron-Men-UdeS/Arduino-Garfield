/*
Projet: Le nom du script
Equipe: Votre numero d'equipe
Auteurs: Les membres auteurs du script
Description: Breve description du script
Date: Derniere date de modification
*/

/* ****************************************************************************
Inclure les librairies de functions que vous voulez utiliser
**************************************************************************** */

#include "main.h"
#define act 1

// suiveur suiveurGauche = {CAPTEUR0_GAUCHE,CAPTEUR0_DROITE,CAPTEUR0_CENTRE,800,800,800,0,0,0};
// suiveur suiveurDroite = {CAPTEUR1_GAUCHE,CAPTEUR1_DROITE,CAPTEUR1_CENTRE,800,800,800,0,0,0};

// float corr;

/* ****************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */
// -> Se fait appeler au debut du programme
// -> Se fait appeler seulement un fois
// -> Generalement on y initilise les varibbles globales
int couleur;
void setup()

void setup()
{
  BoardInit();
  pinMode(LED_ROUGE, OUTPUT);
  pinMode(LED_VERTE, OUTPUT);
  pinMode(LED_JAUNE, OUTPUT);
  pinMode(LED_BLEUE, OUTPUT);
  // initCapteurCouleur();
}
  // pinMode(LED_ROUGE, OUTPUT);
  // pinMode(LED_VERTE, OUTPUT);
  // pinMode(LED_JAUNE, OUTPUT);
  // pinMode(LED_BLEUE, OUTPUT);
  // delay(500);
  // SERVO_Enable(0);
  // SERVO_Enable(1);

  // avance(20, -0.5);
}

/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"

void loop()
// float corr;
void loop()
{
  //############### ACT 1 ###############
  #if act == 1

  int couleur = suivreLigne();
  switch (couleur)
  {
    
    case 0:
    rouge2();
    break;
    
    case 1:
    vert();
    break;
    
    case 2:
    bleu();
    break;

    case 3:
    jauneAntoine();
    break;
    
    default:
    break;
  }
  #endif
//#####################################


//############### ACT 2 ###############
#if act == 2




#endif
//#####################################



}

