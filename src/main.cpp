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
#include "main.h"
/* ****************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */
#define vMaxNormal 0.5
#define vMaxVert 0.7
#define vMaxRouge 0.3
#define DIST 18.75
#define CMPT 23.93895

float angleInitial=0;
int flagR = 0;
int flagV = 0;
int flagB = 0;
int flagJ = 0;
int flagS = 0;
float vMax=0;
int lastTime=0;
unsigned long tempsBumpp=0;
int flagBumper=0;
int couleur=0;
int flagRouge=0;
int flagVert=0;
int flagBleu=0;
int flagJaune=0;
int etatJeu=0;
uint8_t listeLasagne[4];
uint8_t listeGarfield[2];
unsigned long clockR=0;
unsigned long clockV=0;
unsigned long clockB=0;
unsigned long clockJ=0;
unsigned long clockN=0;
unsigned long debutJeu=0;

float anglePrecedent=90;

position robot;

//Flags simulant les données du mvmnt
double positionX=0;
double positionY=0;


float tempsBleu=-10000;

//Les recu par comm
int positionXRecu=-0;
int positionYRecu=0;
int flagBleuRecu=0;
int etatJeuRecu=0;

double vitang=0;
double dist=DIST;
double temps= 0;
double temps_prec= 0;
double vit1=0;
double vit2=0;
 
double cx=0;
double cy=0;
double dep1=0;
double dep2=0;

int encoder_prec1=0;
int encoder_prec2=0;



void actu_angle(position& pos){
  temps=((double) millis())/1000;
  dep1= ((double) ENCODER_ReadReset(0))*CMPT/3200;
  dep2= ((double) ENCODER_ReadReset(1))*CMPT/3200;
  if(temps==temps_prec){vit1=0;vit2=0;}
  else{vit1=dep1/(temps-temps_prec);
  vit2=dep2/(temps-temps_prec);}
  vitang=(vit1-vit2)/dist;
  if ((vit1==vit2)||(dep1==0&&dep2==0)){
    pos.x+=dep1*cos(pos.angle+(PI/2));
    pos.y+=dep1*sin(pos.angle+(PI/2));
    temps_prec=temps;
    encoder_prec1=ENCODER_Read(0);
    encoder_prec2=ENCODER_Read(1);
  }
  else{
      double r= (dist*(vit2+vit1))/(2*(vit2-vit1));
      cx= pos.x - (r*cos(pos.angle));
      cy= pos.y - (r*sin(pos.angle));
    //   Serial.println("vitang");
    //   Serial.println(vitang);
    //   Serial.println("dep1");
    //   Serial.println(dep1);
    //   Serial.println("dep2");
    //   Serial.println(dep2);
    //   Serial.println("vit1");
    //   Serial.println(vit1);
    //   Serial.println("vit2");
    //   Serial.println(vit2);
    // //   Serial.println("rayon :");
    // //   Serial.println(r);
    //   Serial.println("angle:");
    //   Serial.println(pos.angle);
    //   Serial.println("centre x:");
    //   Serial.println(cx);
    //   Serial.println("centre y:");
    //   Serial.println(cy);
//           Serial.println("dep1");
//     Serial.println(dep1);
//           Serial.println("dep2");
//     Serial.println(dep2);
//   Serial.println("temps");
//   Serial.println(temps);
//   Serial.println("tempsPrec");
//   Serial.println(temps_prec);
// //     Serial.println("millis");
//   Serial.println(millis());
    pos.angle-=vitang*(temps-temps_prec);  

    pos.x = cx + (r*cos(pos.angle));
    pos.y = cy + (r*sin(pos.angle));
  
positionX=(cx + (r*cos(pos.angle))+16);
positionY=(cy + (r*sin(pos.angle))+20);

if(pos.angle>(2*PI)){pos.angle=pos.angle-(2*PI);}
if(pos.angle<(-2*PI)){pos.angle=pos.angle+(2*PI);}

    temps_prec=temps;
    encoder_prec1=ENCODER_Read(0);
    encoder_prec2=ENCODER_Read(1);
    //Serial.println(pos.x);
    //Serial.println(pos.y);
    }
}

/*******************************************************************************************
 * Auteur : Raphael Bouchard
 *
 * Algo robot autonome, Sprint vers  coordonées de Lasagne
 ******************************************************************************************/
void algoGarfield(){
 float vMax=0;
 bool coteTourne=true;

 if(flagVert==flagRouge){vMax=vMaxNormal;}
 if(flagVert==1 && flagRouge==0){vMax=vMaxVert;}
 if(flagVert==0 && flagRouge==1){vMax=vMaxRouge;}        //Défini la vitesse selon bonus/malus

 float diffX=positionXRecu-positionX;
 float diffY=positionYRecu-positionY;                  //Définit les différences de positions

 float anglePoursuite=atan2(diffY,diffX);
//Serial.println("Angle de poursuite");
//Serial.println(anglePoursuite);


float changementAngle=(robot.angle+(PI/2))-anglePoursuite;  //Trouve combien il faut qu'il tourne
//Serial.println("Angle changment");
//Serial.println(changementAngle);
//Serial.println("Angle now");
//Serial.println(robot.angle+(PI/2));
if(changementAngle>180){changementAngle=((2*PI)-changementAngle)*-1;}// transforme angle>180 à angle négatif

 if(changementAngle<0){bool coteTourne=false;changementAngle=-changementAngle;}
 if(changementAngle>=0){bool coteTourne=true;} //Regarde et tourne dans le coté moins long
if(diffX<30&&diffY<30){MOTOR_SetSpeed(0,0);MOTOR_SetSpeed(1,0);}
if(diffX>30 || diffY>30){MOTOR_SetSpeed(0,vMax);
MOTOR_SetSpeed(1,vMax);}

uint32_t encodeurInitialGauche=abs(ENCODER_Read(0));
uint32_t encodeurInitialDroite=abs(ENCODER_Read(1));

uint32_t tickEncondeur=abs(angleEnco(changementAngle));


if(coteTourne){if(abs(robot.angle-anglePoursuite)>(PI/16)){MOTOR_SetSpeed(1,0);}}// Bloquant
if(!coteTourne){if(abs(robot.angle-anglePoursuite)>(PI/16)){MOTOR_SetSpeed(0,0);}}// Bloquant

if(flagBumper==1){MOTOR_SetSpeed(0,-vMaxRouge);MOTOR_SetSpeed(1,-vMaxRouge);}
}
void tournejusqua(float angle){
  if(!((robot.angle+PI/2)<angle+PI/6 && ((robot.angle+PI/2)>angle-PI/6))){
  if((robot.angle+PI/2)>angle+PI/6){
    vitesseRoues(0.1,-0.1);
  }  
  if((robot.angle+PI/2)<angle-PI/6){
    vitesseRoues(-0.1,0.1);
  }
 
}else { 
if(flagVert==flagRouge){ vMax=vMaxNormal;}
 if(flagVert==1 && flagRouge==0){ vMax=vMaxVert;}
 if(flagVert==0 && flagRouge==1){ vMax=vMaxRouge;} 
 vitesseRoues(vMax, vMax);}
}
float angledepoursuite(float positionXRecu, float positionYRecu, float positionX, float positionY){
    
  
  float diffX=positionXRecu-positionX;
  float diffY=positionYRecu-positionY;                  //Définit les différences de positions



  float anglePoursuite=atan2(diffY,diffX);



   return anglePoursuite;
}

/*******************************************************************************************
 * Auteur : Alexandre Dionne
 *
 * Initialisation du port UART1 a 115200 bauds
 *
 ******************************************************************************************/
void initUART1(void)
{
    Serial2.begin(115200);
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
    if (Serial2.available() >= sizeTrame)
    {
        Serial2.readBytes(temporaire, 1);
        if (temporaire[0] == 0x24)
        {
            Serial2.readBytes(temporaire, sizeTrame - 1);
            Serial.print(temporaire[0]);
            Serial.print(temporaire[1]);
            Serial.print(temporaire[2]);
            Serial.println(temporaire[3]);

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
    for(unsigned int i = 0; i < (sizeof(*trame)); i++)
    {
        somme = somme + trame[i];
    }
    Serial2.write(0x24);
    Serial2.write(trame, sizeof(*trame));
    Serial2.write(somme);
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
if((etatJeuRecu==1) && flagBumper==0 && debutJeu==0 ){etatJeu=1; debutJeu=millis();}
if(etatJeuRecu==2 || etatJeuRecu==3){etatJeu=3;}
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
positionXRecu=(listeLasagne[0]);
(positionYRecu)=(1*listeLasagne[1]);
flagBleuRecu=listeLasagne[2];
etatJeuRecu=listeLasagne[3];
positionX=robot.x;
positionY=robot.y;

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
  //Serial.print(couleur);

  if (couleur == COULEURJAUNE & (clockN - clockJ > 7000 || clockJ == 0))
  { // Cooldown
    flagJaune = 1;
    while (flagJaune == 1)
    {
      digitalWrite(LED_JAUNE, LOW);
      while(robot.angle-angleInitial<(2*PI)){actu_angle;vitesseRoues(0.2,-0.2);}
      vitesseRoues(0,0);
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
  if (flagBleuRecu==1&&millis()-tempsBleu>10000){digitalWrite(LED_BLEUE,LOW);vitesseRoues(0,0);delay(5000);(LED_BLEUE,HIGH);tempsBleu=millis();}
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
  if(etatJeu==3){digitalWrite(LED_BLEUE,LOW);digitalWrite(LED_ROUGE,LOW);digitalWrite(LED_VERTE,LOW);digitalWrite(LED_JAUNE,LOW);while(true){vitesseRoues(0,0);(10);}}
}


/*******************************************************************************************
 * Auteur : Raphael
 *
 * Regarde l'état des bumpers
 * 
 * Défini flagBumper à 1 si un bumper est ON
 ******************************************************************************************/
  void flagBumperSet(){
    if(flagBumper==1&&millis()-tempsBumpp<5000){flagBumper=1;}
    if(flagBumper==0 || millis()-tempsBumpp>5000){bool bumpp=false;
    if(ROBUS_IsBumper(0)){bumpp=true;}
    if(ROBUS_IsBumper(1)){bumpp=true;}
    if(ROBUS_IsBumper(2)){bumpp=true;}
    if(ROBUS_IsBumper(3)){bumpp=true;}
    if(bumpp){flagBumper=1;float tempsBumpp=millis();}
    if(!bumpp){flagBumper=0;}
}}

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
float anglePoursuite=0;
void loop()
{
  while(1)
  {
 if(!etatJeu==0){ anglePoursuite= angledepoursuite(positionXRecu, positionYRecu, positionX, positionY);
  tournejusqua(anglePoursuite);   //Problématique car loop trop longue donc passe trop de temps à tourner et overshoot
  actu_angle(robot); }
  // Serial.println("angle robot");
  // Serial.println(robot.angle);
  // Serial.println("angle poursuite");
  // Serial.println(anglePoursuite);
  // Serial.println("positionX");
  // Serial.println(positionX);
  // Serial.println("positionY");
  // Serial.println(positionY);

litUART(listeLasagne,6);

if(!etatJeu==0){angledepoursuite(positionXRecu, positionYRecu, positionX, positionY);
  tournejusqua(anglePoursuite);   //Problématique car loop trop longue donc passe trop de temps à tourner et overshoot
  actu_angle(robot); }

receptionListe();
flagBumperSet();
malusRouge();
bonusVert();      
gelBleu();
bananeJaune();
setEtatJeu(); //Doit etre avant delbonus()
delBonus();
creationListe();
envoieTrame(listeGarfield);
// Serial.println("positionXRecu");
// Serial.println(positionXRecu);
// Serial.println("positionYRecu");
// Serial.println(positionYRecu);

// Serial.println("positionX");
// Serial.println(positionX);
// Serial.println("positionY");
// Serial.println(positionY);
  }
}

