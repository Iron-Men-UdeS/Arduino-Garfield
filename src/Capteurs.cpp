
#include "Capteurs.h"

// Suiveur de ligne
int seuilGauche = 875;
int seuilCentre = 875;
int seuilDroite = 875;

// Capteur de couleur test
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_4X); // Définit capteur et caractéristiques d'utilisation

/*******************************************************************************************
 * Auteur : Amine
 * 
 * Determine la valeur seuil du capteur de gauche
 * 
 * @return seuil (integer) millieu entre le blanc et le noir
 ******************************************************************************************/
int calibrerGauche(void)
{
  Serial.println("Place le capteur GAUCHE sur BLANC");
  delay(2000);
  int blanc = analogRead(CAPTEUR0_GAUCHE);
  Serial.print("Valeur blanc = ");
  Serial.println(blanc);

  Serial.println("Place le capteur GAUCHE sur NOIR");
  delay(2000);
  int noir = analogRead(CAPTEUR0_GAUCHE);
  Serial.print("Valeur noir = ");
  Serial.println(noir);

  int seuil = (blanc + noir) / 2;
  Serial.print("Seuil gauche = ");
  Serial.println(seuil);
  return seuil;
}
/*******************************************************************************************
 * Auteur : Amine
 * 
 * Determine la valeur seuil du capteur du centre
 * 
 * @return seuil (integer) millieu entre le blanc et le noir
 ******************************************************************************************/
int calibreCentre(void) 
{
  Serial.println("Place le capteur CENTRE sur BLANC");
  delay(2000);
  int blanc = analogRead(CAPTEUR0_CENTRE);
  Serial.print("Valeur blanc = ");
  Serial.println(blanc);

  Serial.println("Place le capteur CENTRE sur NOIR");
  delay(2000);
  int noir = analogRead(CAPTEUR0_CENTRE);
  Serial.print("Valeur noir = ");
  Serial.println(noir);

  int seuil = (blanc + noir) / 2;
  Serial.print("Seuil centre = ");
  Serial.println(seuil);
  return seuil;
}
/*******************************************************************************************
 * Auteur : Amine
 * 
 * Determine la valeur seuil du capteur de droite
 * 
 * @return seuil (integer) millieu entre le blanc et le noir
 ******************************************************************************************/
int calibrationDroite(void)
{
  Serial.println("Place le capteur DROITE sur BLANC");
  delay(2000);
  int blanc = analogRead(CAPTEUR0_DROITE);
  Serial.print("Valeur blanc = ");
  Serial.println(blanc);

  Serial.println("Place le capteur DROITE sur NOIR");
  delay(2000);
  int noir = analogRead(CAPTEUR0_DROITE);
  Serial.print("Valeur noir = ");
  Serial.println(noir);

  int seuil = (blanc + noir) / 2; // parenthèses corrigées
  Serial.print("Seuil droite = ");
  Serial.println(seuil);
  return seuil;
}
/*******************************************************************************************
 * Auteur : Amine
 * 
 * Stocke les seuils des trois capteurs dans des variables
 * 
 * @return seuil (integer) millieu entre le blanc et le noir
 ******************************************************************************************/
// void calibrationTotale(struct suiveur mySuiveur)
// {
//   suiveurGauche.seuilGauche = calibreSuiveur(suiveurGauche.pinGauche);
//   delay(2500);
//   suiveurGauche.seuilCentre = calibreSuiveur(suiveurGauche.pinCentre);
//   delay(2500);
//   suiveurGauche.seuilDroite = calibreSuiveur(suiveurGauche.pinDroite);
//   delay(2500);

//   suiveurDroite.seuilGauche = calibreSuiveur(suiveurDroite.pinGauche);
//   delay(2500);
//   suiveurDroite.seuilCentre = calibreSuiveur(suiveurDroite.pinCentre);
//   delay(2500);
//   suiveurDroite.seuilDroite = calibreSuiveur(suiveurDroite.pinDroite);
//   delay(2500);
// }

/*******************************************************************************************
 * Auteur : Amine
 * 
 * lit le capteur de contraste pour detecter une ligne
 * 
 * @param capteur (integer) l'id du capteur que l'on souhaite lire
 * 
 * @return resultat (integer) ou la ligne est appercu (0, pas de ligne, 1 ligne) en binaire
 ******************************************************************************************/
int lireCapteurs(int capteur)
{
  int valeurGauche = 0;
  int valeurCentre = 0;
  int valeurDroite = 0;
  int resultat = 0;

  if (capteur == 0)
  {
    valeurGauche = analogRead(CAPTEUR0_GAUCHE);
    valeurCentre = analogRead(CAPTEUR0_CENTRE);
    valeurDroite = analogRead(CAPTEUR0_DROITE);
    // Serial.println(valeurGauche);
    // Serial.println(valeurCentre);
    // Serial.println(valeurDroite);
  }
  else if (capteur == 1)
  {
    valeurGauche = analogRead(CAPTEUR1_GAUCHE);
    valeurCentre = analogRead(CAPTEUR1_CENTRE);
    valeurDroite = analogRead(CAPTEUR1_DROITE);
    // Serial.println(valeurGauche);
    // Serial.println(valeurCentre);
    // Serial.println(valeurDroite);
  }

  resultat = (valeurGauche >= seuilGauche) ? 1 : 0;
  resultat = (((valeurCentre >= seuilCentre) ? 1 : 0) << 1) + resultat;
  resultat = (((valeurDroite >= seuilDroite) ? 1 : 0) << 2) + resultat;

  return resultat;
}

// int lireSuiveur(struct suiveur mySuiveur)
// {
//   mySuiveur.readCentre = analogRead(mySuiveur.pinCentre);
//   mySuiveur.readDroite = analogRead(mySuiveur.pinDroite);
//   mySuiveur.readGauche = analogRead(mySuiveur.pinGauche);

//   int resultat = 0;
//   resultat = (mySuiveur.readGauche >= mySuiveur.seuilGauche) ? 1 : 0;
//   resultat = (((mySuiveur.readCentre >= mySuiveur.seuilCentre) ? 1 : 0) << 1) + resultat;
//   resultat = (((mySuiveur.readDroite >= mySuiveur.seuilDroite) ? 1 : 0) << 2) + resultat;

//   return resultat;
// }

/*******************************************************************************************
 * Auteur : Rapahel
 * 
 * Initialise le capteur de couleur
 ******************************************************************************************/
void initCapteurCouleur(void)
{
  Wire.begin();
  if (tcs.begin()) // S'assure que le capteur est detecte
  {
    tcs.setInterrupt(false);
    delay(100);
  }
}

/*******************************************************************************************
 * Auteur : Rapahel
 * 
 * @return couleur détecté : Rouge = 0, Vert = 1, Bleu = 2, Jaune = 3, aucune couleur = -1
 ******************************************************************************************/
int detectCouleur(void)
{
  uint16_t clear, red, green, blue;

    //tcs.setInterrupt(false);      // turn on LED

    //delay(60);  // takes 50ms to read

    tcs.getRawData(&red, &green, &blue, &clear);

   // tcs.setInterrupt(true);  // turn off LED

  //  Serial.print("C:\t"); Serial.print(clear);
  //  Serial.print("\tR:\t"); Serial.print(red);
  //   Serial.print("\tG:\t"); Serial.print(green);
  //   Serial.print("\tB:\t"); Serial.print(blue);
   // Serial.println();

    // Figure out some basic hex code for visualization
    // uint32_t sum = clear;
    // float r, g, b;
    // r = red; r /= sum;
    // g = green; g /= sum;
    // b = blue; b /= sum;
    // r *= 256; g *= 256; b *= 256;

    
    if(red >= 800 && green >= 900 && (green/(float)blue)>=1.35)
    {
      return COULEURJAUNE;
    }
    if(red >= 675 && red >= blue && (blue/(float)green) >= 0.78)
    {
      return COULEURROUGE;
    }
    if(green >= 700 && (green/(float)red) >= 1.35 && (red/(float)blue) >= 0.8)
    {
       return COULEURVERT;
 }
    if(blue >= 650 && (blue/(float)red)>= 1.2 && (blue/(float)green) >= 0.87)
    {
      return COULEURBLEU;
    }
    
      return -1;
}

/*******************************************************************************************
 * Auteur : Raphael
 *
 * Inverse l'état d'une DEL
 *
 * @param pin (integer) broche de la DEL à inverser
 ******************************************************************************************/
void inverseDEL(int pin)
{
  digitalWrite(pin, !digitalRead(pin)); // Inverse l'etat de la pin
}

/*******************************************************************************************
 * Auteur : Rapahel
 * 
 * Eteint toute les DELs
 ******************************************************************************************/
void eteindreToutesLesDELs(void)
{
  digitalWrite(LED_ROUGE, LOW);
  digitalWrite(LED_VERTE, LOW);
  digitalWrite(LED_JAUNE, LOW);
  digitalWrite(LED_BLEUE, LOW);
}

/*******************************************************************************************
 * Vérifie la détection d'un mur
 *
 * @return (bool) vrai si il y a un mur
 ******************************************************************************************/
bool mur()
{
  if (digitalRead(PIN_DIST_D) == LOW && digitalRead(PIN_DIST_G) == LOW)
  {
    return true;
  }
  else
    return false;
}

/*******************************************************************************************
 * Vérifie le bruit à 5kHz
 *
 * @return (bool) vrai si il y a un bruit a 5kHz
 ******************************************************************************************/
bool sifflet_5kHz()
{
  // TODO verifier le fonctionnement avec le pourcentage d'écart
  float ratio = analogRead(BRUIT_AMBIENT) / analogRead(SIGNAL_5kHz) * 100;

  if (ratio > 150)
  { // ecart est a vérifier dans différent contexte de bruit
    return true;
  }
  else
  {
    return false;
  }
}

/*******************************************************************************************
 * Auteur : Justin
 * lit un capteur de distance, traduit les lectures 
 * en distance (cm) 
 * note: très précis de 10-40 cm, moins précis de 40-80, inutilisable en dessous de 10
 * arguments: la pin du capteur à lire
 * @return la distance en cm
 ******************************************************************************************/
float detecDistance(int pin){
  int voltage = analogRead(pin);
  for (int i=1;i<=7;i++){
    voltage+=analogRead(pin);
  }
  voltage= ((double) voltage)/8;
  return corrDist(pin,(float) (14994*pow(voltage, -1.173)));
}

/*******************************************************************************************
 * Auteur : Justin
 * corrige la valeur du capteur de distancce selon la 
 * fonction invverse obtenue dans les tests d'étalonnage
 * arguments: la pin du capteur dont proviennent les données, la valeur non corrigée (cm)
 * @return la distance, corrigée, en cm
 ******************************************************************************************/
float corrDist (int pin, float valeurCapteur){
  if (pin==DISTANCEA){
    return (valeurCapteur+2.22)/1.272;
  }
  else if(pin==DISTANCEC){
    return (valeurCapteur+4.3587)/1.3894;
  }
  else{
    return 0;
  }
}
