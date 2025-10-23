#include "Niveau1.h"
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
 * Auteur : Simon-Pierre Robert
 * 
 * Fonction rouge check jusqua voir quille, avance, 180, revient et se realligne
 ******************************************************************************************/
void rouge() {
  
  //Fait tourner le robot sur lui-même pour détecter une quille dans un rayon de 0,25 m.
  //Dès qu’elle est détectée, le robot arrête le balayage, avance pour la faire tomber,
  //fait un 180, revient à sa position initiale, puis revient à son orientation de départ.
  //Ne prend ni ne retourne de valeur.
  

  // --- Paramètres ---
  const float DISTANCE_MAX = 25.0;   // Rayon de détection (cm)
  const float DISTANCE_MIN = 5.0;    // Distance minimale à ignorer (cm)
  const float PAS_ROTATION = 10.0;   // Angle d'incrémentation du balayage (°)
  const float VITESSE = 0.4;         // Vitesse pour les déplacements

  bool quilleDetectee = false;
  float distanceDetectee = 0.0;
  float angleDetection = 0.0;

  // --- 1. Balayage progressif ---
  for (float angle = 0; angle < 360; angle += PAS_ROTATION) {

    // Tourne par petits pas
    tourne(PAS_ROTATION, droite);
    delay(300);

    // Lecture de la distance mesurée par le capteur avant
    float distanceLue = detecDistance(distance1);

    // Vérifie si on détecte un objet dans la zone utile
    if (distanceLue <= DISTANCE_MAX && distanceLue > DISTANCE_MIN) {

      // Quille détectée -> on enregistre et on arrête immédiatement le balayage
      quilleDetectee = true;
      distanceDetectee = distanceLue;
      angleDetection = angle;
      break; // On arrête le balayage ici
    }
  }

  // --- 2. Si aucune quille détectée, ne rien faire ---
  if (!quilleDetectee) {
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
    return;
  }

  // --- 3. Avancer directement pour la faire tomber ---
  translation(distanceDetectee + 2, VITESSE);  // +2 cm pour s'assurer du contact
  delay(500);

  // --- 4. Fait un 180 ---
  tourne(180, droite);
  delay(300);

  // --- 5. Avance pour revenir à la ligne ---
  translation(distanceDetectee + 2, VITESSE);
  delay(500);

  // --- 6. Se remettre droit (revenir à orientation initiale) ---
  tourne(180-angleDetection, droite);
  delay(300);
}

/*******************************************************************************************
 * Auteur : Simon-Pierre Robert
 * 
 * fonction qui permet de changer de place avec le robot voisin. Pour ce deplacer a gauche
 * par en bas ou a droite par en haut 
 * @param direction (int) direction du robot : 0 = gauche(bas), 1 = droite(haut)
 ******************************************************************************************/
void changeRobot(int direction) {

    //fonction permet de changer place avec lequipe da coté, faut lui dire si haut = le robot
    //passe en haut ou bas si le rob ot passe en bas, fonction renvoie rien.

    if (direction == 0) { //techniquement le premier mouv qui va en bas puis a gauche
        tourne(180, droite);
        translation(20);
        tourne(90, droite);
        translation(80);
        tourne(90, droite);
        translation(20);
    }

    if (direction == 1) { //techniquement le deuxieme mouv qui va en haut puis a droite
        translation(20);
        tourne(90, droite);
        translation(80);
        tourne(90, droite);
        translation(20);
        tourne(180, droite);
    }
}