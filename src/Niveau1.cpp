#include "Niveau1.h"
#include "Capteurs.h"
#include "Mouvement.h"

float correction=0;

/*******************************************************************************************
 * Auteur : Alexandre Dionne
 * 
 * suit la ligne détecter par les capteurs de contraste
 * 
 * @return couleur (integer) valeur de la coueleur lue
 ******************************************************************************************/
int suivreLigne(void)
{
    unsigned long currentTime;
    unsigned long previousTime;

    float i0 = 1; // Indice correcteur moteur gauche
    float i1 = 1; // Indice correcteur moteur droit
    
    int ligne = 0;

    previousTime = millis();
    while(1)
    {
        currentTime = millis();
        if ((currentTime - previousTime) >= INTERVALLE)
        {
            previousTime = currentTime;
            ligne = lireCapteurs(0);
            ligne = (lireCapteurs(1) << 3) + ligne;

            switch (ligne)
            {
            case 0x01: // 000 001  Grosse correction vers la gauche
            case 0x03: // 000 011
                i0 = 0.75;
                i1 = 1;
                break;

            case 0x02: // 000 010  Petite correction vers la gauche
            case 0x06: // 000 110
                i0 = 0.90;
                i1 = 1;
                break;

            case 0x04: // 000 100  Aucune correction 
            case 0x0C: // 001 100
            case 0x08: // 001 000
                i0 = 1;
                i1 = 1;

                break;

            case 0x18: // 011 000  Petite correction vers la droite
            case 0x10: // 010 000
                i0 = 1;
                i1 = 0.90;
                break;

            case 0x30: // 110 000  Grosse correction vers la droite
            case 0x20: // 100 000
                i0 = 1;
                i1 = 0.75;
                break;

            case 0x3F: // 111 111 ligne partout  Arret complet
            case 0x00: // 000 000 pas de ligne
                i0 = 0;
                i1 = 0;
                break;
            }
            Serial.println(i0);
            Serial.println(i1);
            //Avance(VITESSE_MOTEUR*i0, VITESSE_MOTEUR*i1);
        }

        if(detectCouleur() != -1)
        {
          return detectCouleur();  
        }
    }
}

/*******************************************************************************************
 * Auteur : Raphael
 * 
 * Description : Effectue les mouvements sur la croix pour l'acte 2
 *
 * @param posI (int [0 à 5]) Position initiale sur la croix
 *
 * @param posF (int [0 à 5]) Position finale sur la croix
 *
 ******************************************************************************************/
void changePlace(int posI,int posF)
{
  if(posI==1&&posF==2){
    ENCODER_Reset(0);
    ENCODER_Reset(1);
   while(ENCODER_Read(1)<angleEnco(45)&&ENCODER_Read(0)<angleEnco(45)){
    robotSetSpeed(calculVitesse(0.7,0,angleEnco(45)),1,correction);
   }
   robotSetSpeed(0,0,correction);
   ENCODER_Reset(0);
   ENCODER_Reset(1);
   while(ENCODER_Read(1)<distanceEnco(28.28)&&ENCODER_Read(0)<distanceEnco(28.28)){
    robotSetSpeed(-calculVitesse(0.7,0,distanceEnco(28.28)),0,correction);
   }
   robotSetSpeed(0,0,correction);
   ENCODER_Reset(0);
   ENCODER_Reset(1);
   while(ENCODER_Read(1)<angleEnco(45)&&ENCODER_Read(0)<angleEnco(45)){
    robotSetSpeed(calculVitesse(0.7,0,angleEnco(45)),-1,correction);
   }
   robotSetSpeed(0,0,correction);
   ENCODER_Reset(0);
   ENCODER_Reset(1);
  }
  if(posI==2&&posF==3){
    ENCODER_Reset(0);
    ENCODER_Reset(1);
   while(ENCODER_Read(1)<angleEnco(45)&&ENCODER_Read(0)<angleEnco(45)){
    robotSetSpeed(calculVitesse(0.7,0,angleEnco(45)),-1,correction);
   }
   robotSetSpeed(0,0,correction);
   ENCODER_Reset(0);
   ENCODER_Reset(1);
   while(ENCODER_Read(1)<distanceEnco(28.28)&&ENCODER_Read(0)<distanceEnco(28.28)){
    robotSetSpeed(-calculVitesse(0.7,0,distanceEnco(28.28)),0,correction);
   }
   robotSetSpeed(0,0,correction);
   ENCODER_Reset(0);
   ENCODER_Reset(1);
   while(ENCODER_Read(1)<angleEnco(45)&&ENCODER_Read(0)<angleEnco(45)){
    robotSetSpeed(calculVitesse(0.7,0,angleEnco(45)),1,correction);
   }
   robotSetSpeed(0,0,correction);
   ENCODER_Reset(0);
   ENCODER_Reset(1);
  }
  if(posI==3&&posF==4){
    ENCODER_Reset(0);
    ENCODER_Reset(1);
   while(ENCODER_Read(1)<angleEnco(45)&&ENCODER_Read(0)<angleEnco(45)){
    robotSetSpeed(calculVitesse(0.7,0,angleEnco(45)),1,correction);
   }
   robotSetSpeed(0,0,correction);
   ENCODER_Reset(0);
   ENCODER_Reset(1);
   while(ENCODER_Read(1)<distanceEnco(28.28)&&ENCODER_Read(0)<distanceEnco(28.28)){
    robotSetSpeed(calculVitesse(0.7,0,distanceEnco(28.28)),0,correction);
   }
   robotSetSpeed(0,0,correction);
   ENCODER_Reset(0);
   ENCODER_Reset(1);
   while(ENCODER_Read(1)<angleEnco(45)&&ENCODER_Read(0)<angleEnco(45)){
    robotSetSpeed(calculVitesse(0.7,0,angleEnco(45)),-1,correction);
   }
   robotSetSpeed(0,0,correction);
   ENCODER_Reset(0);
   ENCODER_Reset(1);
  }
  if(posI==4&&posF==1){
    ENCODER_Reset(0);
    ENCODER_Reset(1);
   while(ENCODER_Read(1)<angleEnco(45)&&ENCODER_Read(0)<angleEnco(45)){
    robotSetSpeed(calculVitesse(0.7,0,angleEnco(45)),-1,correction);
   }
   robotSetSpeed(0,0,correction);
   ENCODER_Reset(0);
   ENCODER_Reset(1);
   while(ENCODER_Read(1)<distanceEnco(28.28)&&ENCODER_Read(0)<distanceEnco(28.28)){
    robotSetSpeed(calculVitesse(0.7,0,distanceEnco(28.28)),0,correction);
   }
   robotSetSpeed(0,0,correction);
   ENCODER_Reset(0);
   ENCODER_Reset(1);
   while(ENCODER_Read(1)<angleEnco(45)&&ENCODER_Read(0)<angleEnco(45)){
    robotSetSpeed(calculVitesse(0.7,0,angleEnco(45)),1,correction);
   }
   robotSetSpeed(0,0,correction);
   ENCODER_Reset(0);
   ENCODER_Reset(1);
  }
  if(posI==4&&posF==0){
    ENCODER_Reset(0);
    ENCODER_Reset(1);
   while(ENCODER_Read(1)<angleEnco(90)&&ENCODER_Read(0)<angleEnco(90)){
    robotSetSpeed(calculVitesse(0.7,0,angleEnco(90)),-1,correction);
   }
   robotSetSpeed(0,0,correction);
   ENCODER_Reset(0);
   ENCODER_Reset(1);
   while(ENCODER_Read(1)<distanceEnco(20)&&ENCODER_Read(0)<distanceEnco(20)){
    robotSetSpeed(calculVitesse(0.7,0,distanceEnco(20)),0,correction);
   }
   robotSetSpeed(0,0,correction);
   ENCODER_Reset(0);
   ENCODER_Reset(1);
   while(ENCODER_Read(1)<angleEnco(90)&&ENCODER_Read(0)<angleEnco(90)){
    robotSetSpeed(calculVitesse(0.7,0,angleEnco(90)),1,correction);
   }
   robotSetSpeed(0,0,correction);
   ENCODER_Reset(0);
   ENCODER_Reset(1);
  }
  if(posI==0&&posF==1){
    ENCODER_Reset(0);
    ENCODER_Reset(1);
   while(ENCODER_Read(1)<distanceEnco(20)&&ENCODER_Read(0)<distanceEnco(20)){
    robotSetSpeed(calculVitesse(0.7,0,distanceEnco(20)),0,correction);
   robotSetSpeed(0,0,correction);
   ENCODER_Reset(0);
   ENCODER_Reset(1);
   }}
  if(posI==1&&posF==0){
    ENCODER_Reset(0);
    ENCODER_Reset(1);
   while(ENCODER_Read(1)<distanceEnco(20)&&ENCODER_Read(0)<distanceEnco(20)){
    robotSetSpeed(-calculVitesse(0.7,0,distanceEnco(20)),0,correction);
   robotSetSpeed(0,0,correction);
   ENCODER_Reset(0);
   ENCODER_Reset(1);
   }
}
}