
#include "Capteurs.h"

//Variables

//Suiveur de ligne
int seuilGauche = 800;
int seuilCentre = 800;
int seuilDroite = 800;

//Capteur de couleur
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_4X); //Définit capteur et caractéristiques d'utilisation

//////////////////////////////////////////////////////////////////////////////////////
// Nom de la fonction : calibrerGauche                                                          
// Auteur : Amine                                                                         
// Entree(s) : N/A
// Sortie : seuil
// Description : Determine la valeur seuil du capteur de gauche
//////////////////////////////////////////////////////////////////////////////////////
int calibrerGauche(void) 
{
  Serial.println("Place le capteur GAUCHE sur BLANC");
  delay(2000);
  int blanc = analogRead(CAPTEUR0_GAUCHE);
  Serial.print("Valeur blanc = "); Serial.println(blanc);

  Serial.println("Place le capteur GAUCHE sur NOIR");
  delay(2000);
  int noir = analogRead(CAPTEUR0_GAUCHE);
  Serial.print("Valeur noir = "); Serial.println(noir);

  int seuil = (blanc + noir) / 2;
  Serial.print("Seuil gauche = "); Serial.println(seuil);
  return seuil;
}
//////////////////////////////////////////////////////////////////////////////////////
// Nom de la fonction : calibreCentre                                                          
// Auteur : Amine                                                                         
// Entree(s) : N/A
// Sortie : seuil
// Description : Determine la valeur seuil du capteur du centre
//////////////////////////////////////////////////////////////////////////////////////
int calibreCentre(void) 
{
  Serial.println("Place le capteur CENTRE sur BLANC");
  delay(2000);
  int blanc = analogRead(CAPTEUR0_CENTRE);
  Serial.print("Valeur blanc = "); Serial.println(blanc);

  Serial.println("Place le capteur CENTRE sur NOIR");
  delay(2000);
  int noir = analogRead(CAPTEUR0_CENTRE);
  Serial.print("Valeur noir = "); Serial.println(noir);

  int seuil = (blanc + noir) / 2;
  Serial.print("Seuil centre = "); Serial.println(seuil);
  return seuil;
}
//////////////////////////////////////////////////////////////////////////////////////
// Nom de la fonction : calibrationDroite                                                          
// Auteur : Amine                                                                         
// Entree(s) : N/A
// Sortie : seuil
// Description : Determine la valeur seuil du capteur de droite
//////////////////////////////////////////////////////////////////////////////////////
int calibrationDroite(void) 
{
  Serial.println("Place le capteur DROITE sur BLANC");
  delay(2000);
  int blanc = analogRead(CAPTEUR0_DROITE);
  Serial.print("Valeur blanc = "); Serial.println(blanc);

  Serial.println("Place le capteur DROITE sur NOIR");
  delay(2000);
  int noir = analogRead(CAPTEUR0_DROITE);
  Serial.print("Valeur noir = "); Serial.println(noir);

  int seuil = (blanc + noir) / 2; // parenthèses corrigées
  Serial.print("Seuil droite = "); Serial.println(seuil);
  return seuil;
}
//////////////////////////////////////////////////////////////////////////////////////
// Nom de la fonction : calibrationTotale                                                          
// Auteur : Amine                                                                         
// Entree(s) : N/A
// Sortie : N/A
// Description : Calibre les 3 capteurs et stocke les seuils dans des variables
//////////////////////////////////////////////////////////////////////////////////////
void calibrationTotale(void) 
{
  seuilGauche = calibrerGauche();
  delay(2500);
  seuilCentre = calibreCentre();
  delay(2500);
  seuilDroite = calibrationDroite();
  delay(2500);
}

//////////////////////////////////////////////////////////////////////////////////////
// Nom de la fonction : lireCapteurs                                                          
// Auteur : Amine                                                                         
// Entree(s) : int resultat[3] 
// Sortie : N/A
// Description : Lit les trois capteurs et indique si le capteur voit une ligne (1) ou
//               non dans un tableau
//////////////////////////////////////////////////////////////////////////////////////
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
    valeurGauche = analogRead(CAPTEUR0_GAUCHE);
    valeurCentre = analogRead(CAPTEUR0_CENTRE);
    valeurDroite = analogRead(CAPTEUR0_DROITE);
  }

  resultat = (valeurGauche >= seuilGauche) ? 1 : 0;
  resultat = (((valeurCentre >= seuilCentre) ? 1 : 0) << 1) + resultat;
  resultat = (((valeurDroite >= seuilDroite) ? 1 : 0) << 2) + resultat;
  
  return resultat;
}

//////////////////////////////////////////////////////////////////////////////////////
// Nom de la fonction : initCapteurCouleur                                                          
// Auteur : Rapahel                                                                         
// Entree(s) : N/A
// Sortie : N/A
// Description : Initialise le capteur de couleur
//////////////////////////////////////////////////////////////////////////////////////
void initCapteurCouleur(void)
{
  Wire.begin();
  if (tcs.begin())  //S'assure que le capteur est detecte
  {        
    tcs.setInterrupt(false);
    delay(100);
  }
}

//////////////////////////////////////////////////////////////////////////////////////
// Nom de la fonction : detectCouleur                                                          
// Auteur : Rapahel                                                                         
// Entree(s) : N/A
// Sortie : N/A
// Description : Initialise le capteur de couleur
//////////////////////////////////////////////////////////////////////////////////////
int detectCouleur(void)
{
  uint16_t r, g, b, c;            // Definit les variables r,g,b et c en 16 bits valeurs positives, comme ce que le capteur renvoie
  tcs.getRawData(&r, &g, &b, &c); // Prends les valeurs des capteurs et les renvoies avec les pointeurs de r,g,b et c
  float total = r + g + b;        // Calcul le total afin de faire des proportions de couleurs
  if (total == 0)
  {
    total = 1;
  }                               // Afin d'eviter les divisions par 0
  float rouge = (float)r / total; // variable rouge est la proportion de la couleur
  float vert = (float)g / total;  // la meme avec vert, le float entre parenthèses sert à faire en sorte que la division ne soit pas entière
  float bleu = (float)b / total;  // la meme avec bleu
  if (rouge > bleu * 1.3 && rouge > vert * 1.3)
  {                               // Verifie si rouge est dominant sur les autres couleurs par un coefficient de 1.3
    return couleurRouge;          // Renvoie 0
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
  {                               // Pour le jaune, la moitié de rouge et vert combiné avec un plus petit coefficient
    return couleurJaune;
  }
  else
  {
    return -1;                   // Si aucune couleur dominante, retourne -1
  }
}

//////////////////////////////////////////////////////////////////////////////////////
// Nom de la fonction : inverseDEL                                                          
// Auteur : Rapahel                                                                         
// Entree(s) : pin
// Sortie : N/A
// Description : Inverse l'etat de la DEL indiquee
//////////////////////////////////////////////////////////////////////////////////////
void inverseDEL(int pin)
{
  digitalWrite(pin, !digitalRead(pin)); //Inverse l'etat de la pin
}

//////////////////////////////////////////////////////////////////////////////////////
// Nom de la fonction : eteindreToutesLesLEDs                                                          
// Auteur : Rapahel                                                                         
// Entree(s) : N/A
// Sortie : N/A
// Description : Eteint toute les DELs
//////////////////////////////////////////////////////////////////////////////////////
void eteindreToutesLesDELs(void) 
{
  digitalWrite(LED_ROUGE, LOW);
  digitalWrite(LED_VERTE, LOW);
  digitalWrite(LED_JAUNE, LOW);
  digitalWrite(LED_BLEUE, LOW);
}


 