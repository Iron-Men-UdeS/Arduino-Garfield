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
int couleur=0;
int flagRouge=0;
int flagVert=0;
int flagBleu=3;
int flagJaune=0;
int etatJeu=4;
uint8_t listeLasagne[4];
uint8_t listeGarfield[2];
unsigned long clockR=0;
unsigned long clockV=0;
unsigned long clockB=0;
unsigned long clockJ=0;
unsigned long clockN=0;
unsigned long debutJeu=0;

//Flags simulant les données du mvmnt
int positionX=20;
int positionY=720;

//Les recu par comm
int positionXRecu=0;
int positionYRecu=0;
int flagBleuRecu=0;
int etatJeuRecu=0;

/*******************************************************************************************
 * Auteur : Alexandre Dionne
 *
 * Initialisation du port UART1 a 115200 bauds
 *
 ******************************************************************************************/
void initUART1(void)
{
    Serial1.begin(115200);
}

/*******************************************************************************************
 * Auteur : Alexandre Dionne
 *
 * Lit une trame sur UART1 et stocke les donnes dans un tableau
 *
 * @param trame (Tableau uint8_t) Addresse du tableau pour la trame recu
 * @param sizeTrame (uint8_t) longueur de la trame a recevoir (+2 pour start et checksum)
 ******************************************************************************************/
void litUART(uint8_t *trame, uint8_t sizeTrame)
{
    int somme = 0;
    uint8_t temporaire[sizeTrame - 1];
    int i;

    if (Serial1.available() >= sizeTrame)
    {
        Serial1.readBytes(temporaire, 1);
        if (temporaire[0] == 0x24)
        {
            Serial1.readBytes(temporaire, sizeTrame - 1);
            for (i = 0; i < sizeTrame - 2; i++)
            {
                somme = somme + temporaire[i];
            }
            if (temporaire[sizeTrame - 2] == somme)
            {
                for (i = 0; i < sizeTrame - 2; i++)
                {
                    trame[i] = temporaire[i];
                }
            }
        }
    }
}

/*******************************************************************************************
 * Auteur : Alexandre Dionne
 *
 * Envoie une trame sur le port UART
 *
 * @param trame (Tableau uint8_t) Trame a evoyer
 ******************************************************************************************/
void envoieTrame(uint8_t *trame)
{
    uint8_t somme;
    for(unsigned int i = 0; i < (sizeof(trame)); i++)
    {
        somme = somme + trame[i];
    }
    Serial1.write(0x24);
    Serial1.write(trame, sizeof(trame));
    Serial1.write(somme);
}

/*******************************************************************************************
 * Auteur : Raphael
 *
 * Définit la variable etatJeu
 * 
 * 0=Jeu pas débuté
 * 1=Débuté
 * 2=Débuté mais bumper ON
 * 3=Terminé car deux bumper ON
******************************************************************************************/
void setEtatJeu(){
if((positionXRecu!=0 || positionYRecu!=0) && flagBumper==0 && debutJeu==0 ){etatJeu=1; debutJeu=millis();}
if((positionXRecu!=0 || positionYRecu!=0) && flagBumper==0){etatJeu=1;}
if((positionXRecu!=0 || positionYRecu!=0) && flagBumper==1){etatJeu=2;}
if((positionXRecu!=0 || positionYRecu!=0) && flagBumper==1 && etatJeuRecu==2){etatJeu=3;}
if(millis()-debutJeu>60000){etatJeu=3;}
}

/*******************************************************************************************
 * Auteur : Raphael
 *
 * Crée une liste avec les variables qu'on va communiquer
 * 
 * @return Tableau [x,y,gel,état du jeu]
******************************************************************************************/
void creationListe(){
    listeGarfield[0] = flagBleu;
    listeGarfield[1] = etatJeu;
    
}


/*******************************************************************************************
 * Auteur : Raphael
 *
 * Définit les variables avec la liste reçu
 * 
 * @return distanceXRecu
 * @return distanceYRecu
 * @return flagBleuRecu
 * @return etatJeuRecu
 * 
******************************************************************************************/
void receptionListe(){
positionXRecu=listeLasagne[0];
positionYRecu=listeLasagne[1];
flagBleuRecu=listeLasagne[2];
etatJeuRecu=listeLasagne[3];
}
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
  if (flagBleuRecu==1){digitalWrite(LED_BLEUE,LOW);delay(5000);digitalWrite(LED_BLEUE,HIGH);}
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
  if (flagRouge == 1)
  {
    digitalWrite(LED_ROUGE, LOW);
  }
  if (flagVert == 1)
  {
    digitalWrite(LED_VERTE, LOW);
  }
  if (flagRouge == 0)
  {
    digitalWrite(LED_ROUGE, HIGH);
  }
  if (flagVert == 0)
  {
    digitalWrite(LED_VERTE, HIGH);
  }
  if(etatJeu==3){digitalWrite(LED_BLEUE,LOW);digitalWrite(LED_ROUGE,LOW);digitalWrite(LED_VERTE,LOW);digitalWrite(LED_JAUNE,LOW);while(true){delay(10);}}
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
  BoardInit();
  initUART1();
    pinMode(LED_ROUGE, OUTPUT);
    pinMode(LED_VERTE, OUTPUT);
    pinMode(LED_JAUNE, OUTPUT);
    pinMode(LED_BLEUE, OUTPUT);
    digitalWrite(LED_JAUNE, HIGH);
    digitalWrite(LED_VERTE, HIGH);
    digitalWrite(LED_BLEUE, HIGH);
    digitalWrite(LED_ROUGE, HIGH);
    
}

/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
void loop()
{
 
litUART(listeLasagne,6);
receptionListe();
//flagBumperSet();
//malusRouge();
//bonusVert();
//gelBleu();
//bananeJaune();
//setEtatJeu(); //Doit etre avant delbonus()
//delBonus();
creationListe();
envoieTrame(listeGarfield);
Serial.print(flagBleuRecu);
Serial.print(etatJeuRecu);
Serial.print(positionXRecu);
Serial.print(positionYRecu);
}

