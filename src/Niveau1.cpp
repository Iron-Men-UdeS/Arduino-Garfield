#include "Niveau1.h"
#include "Capteurs.h"
#include "Mouvement.h"

float correction = 0;

int couleur=0;
int flagRouge=0;
int flagVert=0;
int flagBleu=0;
int flagJaune=0;

unsigned long clockR=0;
unsigned long clockV=0;
unsigned long clockB=0;
unsigned long clockJ=0;
unsigned long clockN=0;


/*******************************************************************************************
 * Auteur : Raphael
 *
 * Renvoie le flag rouge selon la couleur et les cooldown
 * 
 * Pas de return mais joue sur la variable globale flagRouge et les clockN et clockR
******************************************************************************************/
void malusRouge(){

couleur=detectCouleur();
clockN=millis();

if(clockN-clockR>5000){flagRouge=0;}  //Durée du bonus/malus

if (couleur==COULEURROUGE&(clockN-clockR>10000||clockR==0)){     //Cooldown 
  flagRouge=1;
  clockR=millis();
  }
}


/*******************************************************************************************
 * Auteur : Raphael
 *
 * Renvoie le flag vert selon la couleur et les cooldown
 * 
 * Pas de return mais joue sur la variable globale flagVert et les clockN et clockV
******************************************************************************************/
void bonusVert(){

couleur=detectCouleur();
clockN=millis();

if(clockN-clockV>5000){flagVert=0;}             //Durée du bonus/malus

if (couleur==COULEURVERT&(clockN-clockV>10000||clockV==0)){     //Cooldown 
  flagVert=1;
  clockV=millis();
  }
}

/*******************************************************************************************
 * Auteur : Raphael
 *
 * Renvoie le flag jaune selon la couleur et les cooldown
 * 
 * Pas de return mais joue sur la variable globale flagJaune et les clockN et clockJ
******************************************************************************************/
void bananeJaune(){

couleur=detectCouleur();
clockN=millis();

if (couleur==COULEURJAUNE&(clockN-clockJ>7000||clockJ==0)){    //Cooldown 
  flagJaune=1;
  clockJ=millis();
  while(flagJaune==1){
  digitalWrite(LED_JAUNE,LOW);
  tourne(720,0.4,DROITE);
  digitalWrite(LED_JAUNE,HIGH);
  flagJaune=0;
  }
}
 else if (couleur!=COULEURJAUNE){flagJaune=0;}
}

/*******************************************************************************************
 * Auteur : Raphael
 *
 * Renvoie le flag bleu selon la couleur et les cooldown
 * 
 * Pas de return mais joue sur la variable globale flagBleu et les clockN et clockB
******************************************************************************************/
void gelBleu(){

couleur=detectCouleur();
clockN=millis();

if(clockN-clockB>5000){flagBleu=0;}         //Durée du bonus/malus

if (couleur==COULEURBLEU&(clockN-clockB>10000||clockB==0)){    //Cooldown 
  flagBleu=1;
  clockB=millis();
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
void delBonus(){
if(flagBleu==1){digitalWrite(LED_BLEUE,LOW);}
if(flagRouge==1){digitalWrite(LED_ROUGE,LOW);}
if(flagVert==1){digitalWrite(LED_VERTE,LOW);}
if(flagBleu==0){digitalWrite(LED_BLEUE,HIGH);}
if(flagRouge==0){digitalWrite(LED_ROUGE,HIGH);}
if(flagVert==0){digitalWrite(LED_VERTE,HIGH);}
}

/*******************************************************************************************
 * Auteur : Alexandre Dionne
 *
 * suit la ligne détectee par les capteurs de contraste
 *
 * @return couleur (integer) valeur de la coueleur lue
 ******************************************************************************************/
// suiveur suiveurD = {CAPTEUR0_GAUCHE, CAPTEUR0_DROITE, CAPTEUR0_CENTRE, 1000, 1000, 1000};
// suiveur suiveurG = {CAPTEUR1_GAUCHE, CAPTEUR1_DROITE, CAPTEUR1_CENTRE, 1000, 1000, 1000};
//float correction81 = 0;

int suivreLigne(void)
{
    unsigned long currentTime;
    unsigned long previousTime;
    static int flagR = 0;
    static int flagV = 0;
    static int flagB = 0;
    static int flagJ = 0;
    int couleur;
    static float i0 = 1; // Indice correcteur moteur gauche
    static float i1 = 1; // Indice correcteur moteur droit

    int ligne = 0;

    previousTime = millis();
    couleur = detectCouleur();
    if(flagR == 1){couleur = -1;}
    if(flagV == 1){couleur = -1;}
    if(flagB == 1){couleur = -1;}
    if(flagJ == 1){couleur = -1;}
    while (couleur == -1)
    {
        couleur = detectCouleur();
        if(couleur == COULEURROUGE){flagR = 1;}
        if(couleur == COULEURVERT){flagV = 1;}
        if(couleur == COULEURBLEU){flagB = 1;}
        if(couleur == COULEURJAUNE){flagJ = 1;}
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
                i0 = -0.50;
                i1 = 1;
                break;


            case 0x02: // 000 010  Petite correction vers la gauche
            case 0x06: // 000 110
                       // Serial.println("2");
                i0 = -0.25;
                i1 = 1;
                break;

            case 0x04: // 000 100  Aucune correction
                i0 = 0.95;
                i1 = 1;
                break;

            case 0x0C: // 001 100
                i0 = 1;
                i1 = 1;
                break;

            case 0x08: // 001 000
                i0 = 1;
                i1 = 0.95;
                // Serial.println("3");
                break;

            case 0x18: // 011 000  Petite correction vers la droite
            case 0x10: // 010 000
                i0 = 1;
                i1 = -0.25;
                break;

            case 0x30: // 110 000  Grosse correction vers la droite
            case 0x20: // 100 000
                i0 = 1;
                i1 = -0.50;
                break;

            case 0x3F: // 111 111 ligne partout  Arret complet
                // Serial.println("6");
                i0 = 0;
                i1 = 0;
                break;
            case 0x00: // 000 000 pas de ligne
                       // Serial.println("7");

                break;
            }
            // Serial.println(i0);
            // Serial.println(i1);
            vitesseRoues(VITESSE_MOTEUR * i0, VITESSE_MOTEUR * i1);
        }
    }
    return detectCouleur();
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
void changePlace(int posI, int posF)
{
    if (posI == 1 && posF == 2)
    {
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        while (ENCODER_Read(1) < angleEnco(45) && ENCODER_Read(0) < angleEnco(45))
        {
            robotSetSpeed(calculVitesse(0.7, 0, angleEnco(45)), 1, correction);
        }
        robotSetSpeed(0, 0, correction);
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        while (ENCODER_Read(1) < distanceEnco(28.28) && ENCODER_Read(0) < distanceEnco(28.28))
        {
            robotSetSpeed(-calculVitesse(0.7, 0, distanceEnco(28.28)), 0, correction);
        }
        robotSetSpeed(0, 0, correction);
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        while (ENCODER_Read(1) < angleEnco(45) && ENCODER_Read(0) < angleEnco(45))
        {
            robotSetSpeed(calculVitesse(0.7, 0, angleEnco(45)), -1, correction);
        }
        robotSetSpeed(0, 0, correction);
        ENCODER_Reset(0);
        ENCODER_Reset(1);
    }
    if (posI == 2 && posF == 3)
    {
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        while (ENCODER_Read(1) < angleEnco(45) && ENCODER_Read(0) < angleEnco(45))
        {
            robotSetSpeed(calculVitesse(0.7, 0, angleEnco(45)), -1, correction);
        }
        robotSetSpeed(0, 0, correction);
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        while (ENCODER_Read(1) < distanceEnco(28.28) && ENCODER_Read(0) < distanceEnco(28.28))
        {
            robotSetSpeed(-calculVitesse(0.7, 0, distanceEnco(28.28)), 0, correction);
        }
        robotSetSpeed(0, 0, correction);
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        while (ENCODER_Read(1) < angleEnco(45) && ENCODER_Read(0) < angleEnco(45))
        {
            robotSetSpeed(calculVitesse(0.7, 0, angleEnco(45)), 1, correction);
        }
        robotSetSpeed(0, 0, correction);
        ENCODER_Reset(0);
        ENCODER_Reset(1);
    }
    if (posI == 3 && posF == 4)
    {
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        while (ENCODER_Read(1) < angleEnco(45) && ENCODER_Read(0) < angleEnco(45))
        {
            robotSetSpeed(calculVitesse(0.7, 0, angleEnco(45)), 1, correction);
        }
        robotSetSpeed(0, 0, correction);
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        while (ENCODER_Read(1) < distanceEnco(28.28) && ENCODER_Read(0) < distanceEnco(28.28))
        {
            robotSetSpeed(calculVitesse(0.7, 0, distanceEnco(28.28)), 0, correction);
        }
        robotSetSpeed(0, 0, correction);
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        while (ENCODER_Read(1) < angleEnco(45) && ENCODER_Read(0) < angleEnco(45))
        {
            robotSetSpeed(calculVitesse(0.7, 0, angleEnco(45)), -1, correction);
        }
        robotSetSpeed(0, 0, correction);
        ENCODER_Reset(0);
        ENCODER_Reset(1);
    }
    if (posI == 4 && posF == 1)
    {
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        while (ENCODER_Read(1) < angleEnco(45) && ENCODER_Read(0) < angleEnco(45))
        {
            robotSetSpeed(calculVitesse(0.7, 0, angleEnco(45)), -1, correction);
        }
        robotSetSpeed(0, 0, correction);
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        while (ENCODER_Read(1) < distanceEnco(28.28) && ENCODER_Read(0) < distanceEnco(28.28))
        {
            robotSetSpeed(calculVitesse(0.7, 0, distanceEnco(28.28)), 0, correction);
        }
        robotSetSpeed(0, 0, correction);
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        while (ENCODER_Read(1) < angleEnco(45) && ENCODER_Read(0) < angleEnco(45))
        {
            robotSetSpeed(calculVitesse(0.7, 0, angleEnco(45)), 1, correction);
        }
        robotSetSpeed(0, 0, correction);
        ENCODER_Reset(0);
        ENCODER_Reset(1);
    }
    if (posI == 4 && posF == 0)
    {
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        while (ENCODER_Read(1) < angleEnco(90) && ENCODER_Read(0) < angleEnco(90))
        {
            robotSetSpeed(calculVitesse(0.7, 0, angleEnco(90)), -1, correction);
        }
        robotSetSpeed(0, 0, correction);
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        while (ENCODER_Read(1) < distanceEnco(20) && ENCODER_Read(0) < distanceEnco(20))
        {
            robotSetSpeed(calculVitesse(0.7, 0, distanceEnco(20)), 0, correction);
        }
        robotSetSpeed(0, 0, correction);
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        while (ENCODER_Read(1) < angleEnco(90) && ENCODER_Read(0) < angleEnco(90))
        {
            robotSetSpeed(calculVitesse(0.7, 0, angleEnco(90)), 1, correction);
        }
        robotSetSpeed(0, 0, correction);
        ENCODER_Reset(0);
        ENCODER_Reset(1);
    }
    if (posI == 0 && posF == 1)
    {
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        while (ENCODER_Read(1) < distanceEnco(20) && ENCODER_Read(0) < distanceEnco(20))
        {
            robotSetSpeed(calculVitesse(0.7, 0, distanceEnco(20)), 0, correction);
            robotSetSpeed(0, 0, correction);
            ENCODER_Reset(0);
            ENCODER_Reset(1);
        }
    }
    if (posI == 1 && posF == 0)
    {
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        while (ENCODER_Read(1) < distanceEnco(20) && ENCODER_Read(0) < distanceEnco(20))
        {
            robotSetSpeed(-calculVitesse(0.7, 0, distanceEnco(20)), 0, correction);
            robotSetSpeed(0, 0, correction);
            ENCODER_Reset(0);
            ENCODER_Reset(1);
        }
    }
}
void randomDEL()
{
    int DEL = rand() % 4;
    switch (DEL)
    {
    case 0:
        inverseDEL(LED_BLEUE);
        break;

    case 1:
        inverseDEL(LED_ROUGE);
        break;

    case 2:
        inverseDEL(LED_JAUNE);
        break;

    case 3:
        inverseDEL(LED_VERTE);
        break;

    default:
        break;
    }
}
/*******************************************************************************************
 * Auteur : Samuuel B. Manelli
 *
 * Fait la petite dance de l'acte 1
 ******************************************************************************************/
void bleu()
{

    // La parti ci-dessous fait faire un carrée au robot et le faire revenir dans le même sens qu'il était
    // Chaque arrête de carrée fera une longueur de 30cm et il va avancer à 50% de la vitesse max
    ENCODER_Reset(GAUCHE);
    ENCODER_Reset(DROITE);
    avance(50, VITESSE_MOTEUR);
    tourne(45, 0.3, DROITE);
    avance(30, -VITESSE_MOTEUR);
    tourne(90, VITESSE_MOTEUR, GAUCHE);
    avance(30, -VITESSE_MOTEUR);
    tourne(90, VITESSE_MOTEUR, DROITE);
    avance(30, VITESSE_MOTEUR);
    tourne(90, VITESSE_MOTEUR, GAUCHE);
    avance(30, VITESSE_MOTEUR);
    tourne(60, VITESSE_MOTEUR, DROITE); // redressi le robot pour qu'il soit dans la même direction que au départ de la fonciton
    ENCODER_Reset(GAUCHE);
    ENCODER_Reset(DROITE);

    // Les servomoteur sont activés pour être utiliser lors de la dance=============point pour le style;)
    //  SERVO_Enable(0);
    //  SERVO_Enable(1);
    //  // fais bouger un bras a 90 et lautre a 45 et ensuite les remets a leur place
    //  SERVO_SetAngle(0,45);
    //  SERVO_SetAngle(0,90)

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
 * Distance 80 cm
 ******************************************************************************************/
void vert()
{
    tourne(30, VITESSE_MOTEUR, GAUCHE);
    avance(45, VITESSE_MOTEUR);
    tourne(30, VITESSE_MOTEUR, DROITE);
    avance(80, 0.7);
    tourne(30, VITESSE_MOTEUR, DROITE);
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
void bougeBrasDroit(int position)
{
    // enable juste celui a droite
    switch (position)
    {
    // Ne donne aucune instruction donc les servos ne bouge pas
    case AUCUN:
        break;

    // Met les bras vers l'avant du robot
    case AVANT:
        SERVO_SetAngle(BRAS_DROIT, 90);
        break;
    // Met les bras vers l'arrière du robot
    //  case ARRIERE:
    //  SERVO_SetAngle(BRAS_DROIT, voir angle au repos);
    //  break;
    //  //Met les bras vers le haut du robot
    case HAUT:
        SERVO_SetAngle(BRAS_DROIT, 175);
        break;
    // Met les bras vers le bas du robot
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
void bougeBrasGauche(int position)
{
    // enable juste celui utiliser a gauche
    switch (position)
    {
    // Ne donne aucune instruction donc les servos ne bouge pas
    case AUCUN:
        break;

    // Met les bras vers l'avant du robot
    case AVANT:
        SERVO_SetAngle(BRAS_GAUCHE, 90);
        break;
    // Met les bras vers l'arrière du robot
    //  case ARRIERE:
    //  SERVO_SetAngle(BRAS_GAUCHE, voir angle au repos);
    //  break;
    //  //Met les bras vers le haut du robot
    case HAUT:
        SERVO_SetAngle(BRAS_GAUCHE, 10);
        break;
    // Met les bras vers le bas du robot
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
void bouge2Bras(int posGauche, int posDroit)
{
    bougeBrasGauche(posGauche);
    bougeBrasDroit(posDroit);
}
/*******************************************************************************************
 * Auteur : Simon-Pierre Robert
 *
 * Fonction rouge check jusqua voir quille, avance, 180, revient et se realligne
 ******************************************************************************************/
// void rouge()
// {

//     // Fait tourner le robot sur lui-même pour détecter une quille dans un rayon de 0,25 m.
//     // Dès qu’elle est détectée, le robot arrête le balayage, avance pour la faire tomber,
//     // fait un 180, revient à sa position initiale, puis revient à son orientation de départ.
//     // Ne prend ni ne retourne de valeur.

//     // --- Paramètres ---
//     const float DISTANCE_MAX = 25.0; // Rayon de détection (cm)
//     const float DISTANCE_MIN = 5.0;  // Distance minimale à ignorer (cm)
//     const float PAS_ROTATION = 5.0;  // Angle d'incrémentation du balayage (°)
//     const float VITESSE = 0.4;       // Vitesse pour les déplacements

//     bool quilleDetectee = false;
//     float distanceDetectee = 0.0;
//     float angleDetection = 0.0;
//     float distanceLue = 0;

//     // --- 1. Balayage progressif ---
//     for (float angle = 0; angle < 360; angle += PAS_ROTATION)
//     {

//         // Tourne par petits pas
//         tourne(PAS_ROTATION, VITESSE_MOTEUR, DROITE);
//         delay(300);
//         // Lecture de la distance mesurée par le capteur avant
//         distanceLue = detecDistance(DISTANCEA);

//         // Vérifie si on détecte un objet dans la zone utile
//         if (distanceLue <= DISTANCE_MAX && distanceLue > DISTANCE_MIN)
//         {

//             // Quille détectée -> on enregistre et on arrête immédiatement le balayage
//             quilleDetectee = true;
//             distanceDetectee = distanceLue;
//             angle = angleDetection;
//             break; // On arrête le balayage ici
//         }
//     }

//     // --- 2. Si aucune quille détectée, ne rien faire ---
//     if (!quilleDetectee)
//     {
//         MOTOR_SetSpeed(0, 0);
//         MOTOR_SetSpeed(1, 0);
//         return;
//     }

//     // --- 3. Avancer directement pour la faire tomber ---
//     avance(distanceDetectee + 2, VITESSE); // +2 cm pour s'assurer du contact
//     // delay(500);

//     // --- 4. Fait un 180 ---
//     tourne(2 * QUART_DE_TOUR, VITESSE_MOTEUR, DROITE);
//     // delay(300);

//     // --- 5. Avance pour revenir à la ligne ---
//     avance(distanceDetectee + 2, VITESSE);
//     // delay(500);

//     // --- 6. Se remettre droit (revenir à orientation initiale) ---
//     float angleD;
//     if (angleDetection >= 180)
//     {
//         angleD = angleDetection - 180;
//         tourne(angleD, VITESSE_MOTEUR, DROITE);
//     }
//     else if (angleDetection < 180)
//     {
//         angleD = 90 + angleDetection;
//         tourne(angleD, VITESSE_MOTEUR, GAUCHE);
//     }
//     // delay(300);
// }

void rouge2()
{
    float dist = detecDistance(DISTANCEA);
    int retour;
    ENCODER_Reset(LEFT);
    ENCODER_Reset(RIGHT);
    while (dist > 30)
    { // 40 = valeur en cm peut-etre a changer
        MOTOR_SetSpeed(LEFT, -0.2);
        MOTOR_SetSpeed(RIGHT, 0.2);
        dist = detecDistance(DISTANCEA);
        retour = ENCODER_Read(RIGHT);
    }
    ENCODER_Reset(LEFT);
    ENCODER_Reset(RIGHT);
    double currentmillis = millis();
    while (millis() - currentmillis < 300)
    {
        MOTOR_SetSpeed(LEFT, 0);
        MOTOR_SetSpeed(RIGHT, 0);
    }

    //Serial.println("retour : : " + String(retour));
    avance(dist + 15, 0.5);
    avance(dist + 15, -0.5);
    ENCODER_Reset(LEFT);
    ENCODER_Reset(RIGHT);
    int dep = ENCODER_Read(LEFT);

    while (dep < retour)
    {
        MOTOR_SetSpeed(LEFT, 0.2);
        MOTOR_SetSpeed(RIGHT, -0.2);
        dep = ENCODER_Read(LEFT);
        // Serial.println("deplacement : " + String(dep));
        // Serial.println("retour : : " + String(retour));
    }
    currentmillis = millis();
    while (millis() - currentmillis < 150)
    {
        MOTOR_SetSpeed(LEFT, 0);
        MOTOR_SetSpeed(RIGHT, 0);
    }
    avance(15,VITESSE_MOTEUR);
}

void jauneAntoine()
{
    //float corr;
    float dist = detecDistance(DISTANCEA);
    while (dist > 15)
    { // 40 = valeur en cm peut-etre a changer
        //robotSetSpeed(0.2, 0, corr);
        vitesseRoues(0.2, 0.2);
        dist = detecDistance(DISTANCEA);
    }
    vitesseRoues(0.0, 0.0);
    tourne(QUART_DE_TOUR, VITESSE_MOTEUR, GAUCHE);
    avance(40, 0.5); // 40 = longueur du mur a ajusté
    tourne(QUART_DE_TOUR, VITESSE_MOTEUR, DROITE);
    avance(55, 0.5); // 50 = épaisseur du mur a ajusté
    tourne(QUART_DE_TOUR, VITESSE_MOTEUR, DROITE);
    avance(35, 0.5); // 40 = longueur du mur a ajusté
    tourne(125, VITESSE_MOTEUR, GAUCHE);
}
/*******************************************************************************************
 * Auteur : Simon-Pierre Robert
 *
 * fonction qui permet de changer de place avec le robot voisin. Pour ce deplacer a gauche
 * par en bas ou a droite par en haut
 * @param direction (int) direction du robot : 0 = gauche(bas), 1 = droite(haut)
 ******************************************************************************************/
void changeRobot(int direction)
{

    // fonction permet de changer place avec lequipe da coté, faut lui dire si haut = le robot
    // passe en haut ou bas si le rob ot passe en bas, fonction renvoie rien.

    if (direction == 0)
    { // techniquement le premier mouv qui va en bas puis a gauche
        tourne(2 * QUART_DE_TOUR, VITESSE_MOTEUR, DROITE);
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

    if (direction == 1)
    { // techniquement le deuxieme mouv qui va en haut puis a droite
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
        tourne(2 * QUART_DE_TOUR, VITESSE_MOTEUR, DROITE);
    }
}