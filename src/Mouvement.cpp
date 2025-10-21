#include "Mouvement.h"

/**
 * Convertie la distance donné en cm en distance donné en nombre de tick d'encodeur
 *
 * Utilise les définitions:
 *
 * TOUR_COMPLET (nb de tick pour un tour de roue)
 *
 * CIRCON_ROUE_CM (circonférence d'une roue en centimètre)
 *
 * @param distanceCM (float) distance en centimètre
 *
 * @return (integer 32 bits non-signé) distance en tick d'encodeur
 */
uint32_t distanceEnco(float distanceCM)
{
    uint32_t val = (TOUR_COMPLET / CIRCON_ROUE_CM) * distanceCM;
    return val;
}

/**
 * Convertie l'angle de rotation donné en degré en nombre de tick d'encodeur
 *
 * Utilise les définitions:
 *
 * TOUR_COMPLET (nb de tick pour un tour de roue)
 *
 * CIRCON_ROUE_CM (circonférence d'une roue en centimètre)
 *
 * @param angleDEG (integer) angle de rotation en degré
 * @return (integer 32 bits non-signé) rotation en tick d'encodeur
 */
uint32_t angleEnco(int angleDeg)
{
    float rad = radians(angleDeg);
    float deplacementCM = (rad * (19 / 2));
    return round((TOUR_COMPLET * deplacementCM) / CIRCON_ROUE_CM);
}

/**
 * Calcul la vitesse de déplacement en fonction de la distance restante au déplacement selon une courbe sigmoide
 *
 * Augmente la vitesse pendant le premier quart du déplacement
 *
 * Diminue la vitesse pendant la dernière demi du déplacement
 *
 * @param maxSpeed (float) borne maximal de vitesse permise
 *
 * @param position (integer 32 bit non-signé) position actuel en tick d'encodeur
 *
 * @param positionFinal (integer 32 bit non-signé) position final souhaité en tick d'encodeur
 *
 * @return (double) facteur de multiplication de la vitesse [0.2 à 1]
 */
double calculVitesse(float maxSpeed, uint32_t position, uint32_t positionFinal)
{
    double vit;
    double demi = positionFinal / 2;
    double quart = positionFinal / 4;

    if (position < quart)
    { // si dans le premier quart du déplacement accélaire jusqu'a la vitesse requise
        double rate = (position / quart * 6.0) - 3.0;
        vit = 1 / (exp(-rate) + 1);
    }
    else if (position >= demi)
    { // ralentie jusqu'a 0 a partir de la moitié du déplacement
        double rate = ((position - demi) / demi * 6.0) - 3.0;
        vit = 1 - (1 / (exp(-rate) + 1));
    }
    else
    {
        vit = 1;
    }
    vit = vit * maxSpeed; // convertie le facteur vit (entre 0 et 1) en facteur entre 0 et maxSpeed

    // vérifie si la vitesse dépasse extremum et la corrige en fonction
    if (vit < 0.2)
    {
        return 0.2;
    }
    else if (vit > 1)
    {
        return 1.0;
    }
    else
    {
        return vit;
    }
}

/**
 * Controleur Proportionel Intégral Différentiel
 *
 * Réduit l'erreur entre la valeur de consigne et la valeur réel
 *
 * Variables globales utilisé: kp, ki, kd
 *
 * @param error (float) différence entre la valeur de consigne et la valeur réel
 *
 * @param lastError (float) error calculé à la boucle précedente
 *
 * @return (double) terme de correction à additionner à la vitesse du robot
 *
 */
double pid(float error, float &lastError)
{
    static float integral;

    static double lastMillis = 0;
    static double difTemps = 0;

    difTemps = (millis() - lastMillis) / 1000;
    lastMillis = millis();
    double proportional = error;
    integral += error * difTemps;
    double derive = (error - lastError) / difTemps;
    lastError = error;
    double out = (KP * proportional) + (KI * integral) + (KD * derive);
    return out;
}

/**
 * Controle la vitesse des deux moteur tout en corrigeant le moteur droit
 *
 * @param vitesse (float [0 à 1]) vitesse des moteur
 *
 * @param direction (int) direction du robot : -1 = gauche, 0 = avance, 1 = droite
 *
 * @param &correction (float) correction à apporter a la roue droite
 *
 */
void robotSetSpeed(float vitesse, int direction, float &correction)
{

    switch (direction)
    {
    case -1:
        MOTOR_SetSpeed(GAUCHE, -vitesse);
        MOTOR_SetSpeed(DROITE, vitesse + correction);
        break;

    case 0:
        MOTOR_SetSpeed(GAUCHE, vitesse);
        MOTOR_SetSpeed(DROITE, vitesse + correction);
        break;

    case 1:
        MOTOR_SetSpeed(GAUCHE, vitesse);
        MOTOR_SetSpeed(DROITE, -(vitesse + correction));
        break;

    default:
        MOTOR_SetSpeed(GAUCHE, vitesse);
        MOTOR_SetSpeed(DROITE, vitesse + correction);
        break;
    }

    float setPoint = ENCODER_Read(GAUCHE);
    float mesure = ENCODER_Read(DROITE);
    float error = setPoint - mesure;
    static float previousError = 0;
    correction = correction + pid(error, previousError);
}

/**
 * Inverse l'état d'une DEL
 *
 * @param pin (integer) broche de la DEL à inverser
 */
void inverseDEL(int pin)
{
    digitalWrite(pin, !digitalRead(pin)); // Inverse l'etat de la pin
}
