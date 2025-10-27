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
            //avance(VITESSE_MOTEUR*i0, VITESSE_MOTEUR*i1);
            MOTOR_SetSpeed(GAUCHE, VITESSE_MOTEUR*i0);
            MOTOR_SetSpeed(DROITE, VITESSE_MOTEUR*i1);
        }

        if(detectCouleur() != -1)
        {
          return detectCouleur();  
        }
    }
}


/*******************************************************************************************
 * Auteur : Samuuel B. Manelli
 * 
 * Fait la petite dance de l'acte 1
 ******************************************************************************************/
void bleu(){

    //La parti ci-dessous fait faire un carrée au robot et le faire revenir dans le même sens qu'il était
    //Chaque arrête de carrée fera une longueur de 30cm et il va avancer à 50% de la vitesse max
    tourne(135,0.3, DROITE);
    avance(30,0.5);
    for (int i=0; i<3; i++){
        tourne(90,0.5, DROITE);
        avance(30,0.5);
    }
    tourne(30,0.5,GAUCHE); //redressi le robot pour qu'il soit dans la même direction que au départ de la fonciton

    //Les servomoteur sont activés pour être utiliser lors de la dance=============point pour le style;) 
    // SERVO_Enable(0);
    // SERVO_Enable(1);
    // // fais bouger un bras a 90 et lautre a 45 et ensuite les remets a leur place
    // SERVO_SetAngle(0,45);
    // SERVO_SetAngle(0,90)

    // SERVO_SetAngle(0,0);
    // SERVO_SetAngle(0,0);
    // //desactive les servomoteur
    // SERVO_Disable(0);
    // SERVO_Disable(1);
}


/*******************************************************************************************
 * Auteur : Samuel B. Manelli
 * 
 * Avance d'une distance prédéterminer
 * @param Distance 80 cm
 ******************************************************************************************/
void vert(){
    avance(80,0.8);
}

// //amelioration possible faire un while jusqua ce que le suiveur de ligne renvoie un info pour dire qu,il troiver la ligne
// void vertV2(){
//     //Avance en ligne droite jusque une ligne soit detecter
//     while(dectionLigne different du resultat qui est renvoyer lorsque la ligne est detecter){
//         robotSetSpeed(0.8,0);
//     }
// }


/*******************************************************************************************
 * Auteur : Samuel B. Manelli
 * 
 * Permet de faire bouger le bras droit uniquement
 * 
 * @param position AUCUN, AVANT, ARRIERE, HAUT, BAS
 * 
 * 
 ******************************************************************************************/
void bougeBrasDroit(int position){
    //enable juste celui a droite
    switch(position){
        //Ne donne aucune instruction donc les servos ne bouge pas
        case AUCUN:
        break;

        //Met les bras vers l'avant du robot
        case AVANT:
        SERVO_SetAngle(BRAS_DROIT, 90);
        break;
        //Met les bras vers l'arrière du robot
        // case ARRIERE:
        // SERVO_SetAngle(BRAS_DROIT, voir angle au repos);
        // break;
        // //Met les bras vers le haut du robot
        case HAUT:
        SERVO_SetAngle(BRAS_DROIT, 175);
        break;
        //Met les bras vers le bas du robot
        case BAS:
        SERVO_SetAngle(BRAS_DROIT, 5);
        break;
    }
}


/*******************************************************************************************
 * Auteur : Samuel B. Manelli
 * 
 * Permet de faire bouger le bras gauche uniquement
 * 
 * @param position AUCUN, AVANT, ARRIERE, HAUT, BAS
 * 
 ******************************************************************************************/
void bougeBrasGauche(int position){
    //enable juste celui utiliser a gauche
    switch(position){
        //Ne donne aucune instruction donc les servos ne bouge pas
        case AUCUN:
        break;
        
        //Met les bras vers l'avant du robot
        case AVANT:
        SERVO_SetAngle(BRAS_GAUCHE, 90);
        break;
        //Met les bras vers l'arrière du robot
        // case ARRIERE:
        // SERVO_SetAngle(BRAS_GAUCHE, voir angle au repos);
        // break;
        // //Met les bras vers le haut du robot
        case HAUT:
        SERVO_SetAngle(BRAS_GAUCHE, 10);
        break;
        //Met les bras vers le bas du robot
        case BAS:
        SERVO_SetAngle(BRAS_GAUCHE, 175);
        break;
    }
}

/*******************************************************************************************
 * Auteur : Samuel B. Manelli
 * 
 * Permet de faire bouger le bras et le bras droit simultanément
 * 
 * @param posGauche AUCUN, AVANT, ARRIERE, HAUT, BAS
 * @param posDroit AUCUN, AVANT, ARRIERE, HAUT, BAS
 ******************************************************************************************/
void bouge2Bras(int posGauche,int posDroit){
    bougeBrasGauche(posGauche);
    bougeBrasDroit(posDroit);
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
  const float PAS_ROTATION = 5.0;   // Angle d'incrémentation du balayage (°)
  const float VITESSE = 0.4;         // Vitesse pour les déplacements

  bool quilleDetectee = false;
  float distanceDetectee = 0.0;
  float angleDetection = 0.0;
  float distanceLue = 0;

  // --- 1. Balayage progressif ---
  for (float angle = 0; angle < 360; angle += PAS_ROTATION) {

    // Tourne par petits pas
    tourne(PAS_ROTATION, VITESSE_MOTEUR, DROITE);
    delay(300);
    // Lecture de la distance mesurée par le capteur avant
    distanceLue = detecDistance(DISTANCEA);

    // Vérifie si on détecte un objet dans la zone utile
    if (distanceLue <= DISTANCE_MAX && distanceLue > DISTANCE_MIN) {

      // Quille détectée -> on enregistre et on arrête immédiatement le balayage
      quilleDetectee = true;
      distanceDetectee = distanceLue;
      angle = angleDetection;
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
  avance(distanceDetectee + 2, VITESSE);  // +2 cm pour s'assurer du contact
  //delay(500);

  // --- 4. Fait un 180 ---
  tourne(2*QUART_DE_TOUR, VITESSE_MOTEUR, DROITE);
  //delay(300);

  // --- 5. Avance pour revenir à la ligne ---
  avance(distanceDetectee + 2, VITESSE);
  //delay(500);

  // --- 6. Se remettre droit (revenir à orientation initiale) ---
  float angleD;
  if(angleDetection >= 180){
    angleD = angleDetection - 180;
    tourne(angleD, VITESSE_MOTEUR, DROITE);
  }
  else if ( angleDetection < 180){
    angleD = 90+angleDetection;
    tourne(angleD, VITESSE_MOTEUR, GAUCHE);
  }
  //delay(300);
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
        tourne(2*QUART_DE_TOUR, VITESSE_MOTEUR, DROITE);
        delay(TIME);
        avance(20, VITESSE_MOTEUR);
        delay(TIME);
        tourne(QUART_DE_TOUR, VITESSE_MOTEUR, DROITE);
        delay(TIME);
        avance(80, VITESSE_MOTEUR);
        delay(TIME);
        tourne(QUART_DE_TOUR, VITESSE_MOTEUR, DROITE);
        delay(TIME);
        avance(20, VITESSE_MOTEUR);
    }

    if (direction == 1) { //techniquement le deuxieme mouv qui va en haut puis a droite
        avance(20, VITESSE_MOTEUR);
        delay(TIME);
        tourne(QUART_DE_TOUR, VITESSE_MOTEUR, DROITE);
        delay(TIME);
        avance(80, VITESSE_MOTEUR);
        delay(TIME);
        tourne(QUART_DE_TOUR, VITESSE_MOTEUR, DROITE);
        delay(TIME);
        avance(20, VITESSE_MOTEUR);
        delay(TIME);
        tourne(2*QUART_DE_TOUR, VITESSE_MOTEUR, DROITE);
    }
}