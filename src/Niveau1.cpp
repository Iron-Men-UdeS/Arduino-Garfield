#include "Niveau1.h"
#include "Capteurs.h"

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
 * Auteur : Samuuel B. Manelli
 * 
 * Fait la petite dance de l'acte 1
 ******************************************************************************************/
void bleu(){

    //La parti ci-dessous fait faire un carrée au robot et le faire revenir dans le même sens qu'il était
    //Chaque arrête de carrée fera une longueur de 30cm et il va avancer à 50% de la vitesse max
    tourne(135,0.5, DROITE);
    avance(30,0.5);
    for (int i=0; i<3; i++){
        tourne(90,0.5, DROITE);
        avance(30,0.5);
    }
    tourne(45,0.5,GAUCHE); //redressi le robot pour qu'il soit dans la même direction que au départ de la fonciton

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