/*##### Librairie #####*/

#include "Capteurs.h"

/*##### Objet #####*/

// Capteur de couleur
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_4X); // Définit capteur et caractéristiques d'utilisation

/*##### Variables #####*/

// Suiveur de ligne
struct suiveur{
  int pinGauche,pinDroite,pinCentre;
  int seuilCentre;
  int seuilDroite;
  int seuilGauche;
  int readCentre,readDroite,readGauche;
};

suiveur suiveurGauche = {CAPTEUR0_GAUCHE,CAPTEUR0_DROITE,CAPTEUR0_CENTRE,800,800,800,0,0,0};
suiveur suiveurDroite = {CAPTEUR1_GAUCHE,CAPTEUR1_DROITE,CAPTEUR1_CENTRE,800,800,800,0,0,0};

/*##### Fonctions #####*/

/*******************************************************************************************
 * Auteur : Justin
 * 
 * lit un capteur de distance, traduit les lectures 
 * en distance (cm) 
 * 
 * note: très précis de 10-40 cm, moins précis de 40-80, inutilisable en dessous de 10
 * 
 * @param pin (integer) la broche à lire
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
 * fonction inverse obtenue dans les tests d'étalonnage
 * arguments: la pin du capteur dont proviennent les données, la valeur non corrigée (cm)
 * @return la distance, corrigée, en cm
 ******************************************************************************************/
float corrDist(int pin, float valeurCapteur){
  if (pin==DISTANCE1){
    return (valeurCapteur+2.22)/1.272;
  }
  else if(pin==DISTANCE2){
    return (valeurCapteur+4.3587)/1.3894;
  }
  else{
    return 0;
  }
}

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
 * Modif : Antoine
 * 
 * Determine la valeur seuil du capteur 
 * 
 * @return seuil (integer) millieu entre le blanc et le noir
 ******************************************************************************************/
float calibreSuiveur(int pin){
  Serial.println("Place le capteur sur BLANC");
  delay(2000);
  int blanc = analogRead(pin);
  Serial.print("Valeur blanc = ");
  Serial.println(blanc);

  Serial.println("Place le capteur sur NOIR");
  delay(2000);
  int noir = analogRead(pin);
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
 * Stocke les seuils des trois capteurs dans des variables
 * 
 * @return seuil (integer) millieu entre le blanc et le noir
 ******************************************************************************************/
void calibrationTotale(struct suiveur mySuiveur)
{
  mySuiveur.seuilGauche = calibreSuiveur(suiveurGauche.pinGauche);
  delay(2500);
  mySuiveur.seuilCentre = calibreSuiveur(suiveurGauche.pinCentre);
  delay(2500);
  mySuiveur.seuilDroite = calibreSuiveur(suiveurGauche.pinDroite);
  delay(2500);
}

/*******************************************************************************************
 * Auteur : Amine
 * 
 * Modif : Antoine
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
  }
  else if (capteur == 1)
  {
    valeurGauche = analogRead(CAPTEUR1_GAUCHE);
    valeurCentre = analogRead(CAPTEUR1_CENTRE);
    valeurDroite = analogRead(CAPTEUR1_DROITE);
  }

//  resultat = (valeurGauche >= seuilGauche) ? 1 : 0;
  //resultat = (((valeurCentre >= seuilCentre) ? 1 : 0) << 1) + resultat;
  //resultat = (((valeurDroite >= seuilDroite) ? 1 : 0) << 2) + resultat;

  return resultat;
}

int lireSuiveur(struct suiveur mySuiveur)
{
  mySuiveur.readCentre = analogRead(mySuiveur.pinCentre);
  mySuiveur.readDroite = analogRead(mySuiveur.pinDroite);
  mySuiveur.readGauche = analogRead(mySuiveur.pinGauche);

  int resultat = 0;
  resultat = (mySuiveur.readGauche >= mySuiveur.seuilGauche) ? 1 : 0;
  resultat = (((mySuiveur.readCentre >= mySuiveur.seuilCentre) ? 1 : 0) << 1) + resultat;
  resultat = (((mySuiveur.readDroite >= mySuiveur.seuilDroite) ? 1 : 0) << 2) + resultat;

  return resultat;
}

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
 * Initialise le capteur de couleur
 ******************************************************************************************/
int detectCouleur(void)
{
  uint16_t r, g, b, c;            // Definit les variables r,g,b et c en 16 bits valeurs positives, comme ce que le capteur renvoie
  tcs.getRawData(&r, &g, &b, &c); // Prends les valeurs des capteurs et les renvoies avec les pointeurs de r,g,b et c
  float total = r + g + b;        // Calcul le total afin de faire des proportions de couleurs
  if (total == 0)
  {
    total = 1;
  } // Afin d'eviter les divisions par 0
  float rouge = (float)r / total; // variable rouge est la proportion de la couleur
  float vert = (float)g / total;  // la meme avec vert, le float entre parenthèses sert à faire en sorte que la division ne soit pas entière
  float bleu = (float)b / total;  // la meme avec bleu
  if (rouge > bleu * 1.3 && rouge > vert * 1.3)
  {                      // Verifie si rouge est dominant sur les autres couleurs par un coefficient de 1.3
    return couleurRouge; // Renvoie 0
  }
  else if (bleu > rouge * 1.3 && bleu > vert * 1.3)
  {
    return couleurBleu;
  }
  else if (vert > rouge * 1.3 && vert > bleu * 1.3)
  {
    return couleurVert;
  }
  else if ((rouge + vert) / 2 > bleu * 1.2)
  { // Pour le jaune, la moitié de rouge et vert combiné avec un plus petit coefficient
    return couleurJaune;
  }
  else
  {
    return -1; // Si aucune couleur dominante, retourne -1
  }
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
  float diff = analogRead(BRUIT_AMBIENT) - analogRead(SIGNAL_5kHz) ;

  if (diff > 150)
  { // ecart est a vérifier dans différent contexte de bruit
    return true;
  }
  else
  {
    return false;
  }
}
