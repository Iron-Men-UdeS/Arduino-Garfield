#include "Mouvement.h"



/*******************************************************************************************
 * Auteur : Antoine
 * 
 * Convertie la distance donné en cm en distance donné en nombre de tick d'encodeur
 *
 * Utilise les définitions:
 *
 * TOUR_COMPLET_ENCO (nb de tick pour un tour de roue)
 *
 * CIRCON_ROUE_CM (circonférence d'une roue en centimètre)
 *
 * @param distanceCM (float) distance en centimètre
 *
 * @return (integer 32 bits non-signé) distance en tick d'encodeur
 ******************************************************************************************/
uint32_t distanceEnco(float distanceCM)
{
    uint32_t val = (TOUR_COMPLET_ENCO / CIRCON_ROUE_CM) * distanceCM;
    return val;
}

double encoToCM(float distanceEnco){
  return distanceEnco * (CIRCON_ROUE_CM/TOUR_COMPLET_ENCO);
}

/*******************************************************************************************
 * Auteur : Antoine
 * 
 * Convertie l'angle de rotation donné en degré en nombre de tick d'encodeur
 *
 * Utilise les définitions:
 *
 * TOUR_COMPLET_ENCO (nb de tick pour un tour de roue)
 *
 * CIRCON_ROUE_CM (circonférence d'une roue en centimètre)
 *
 * @param angleDEG (integer) angle de rotation en degré
 * @return (integer 32 bits non-signé) rotation en tick d'encodeur
 ******************************************************************************************/
uint32_t angleEnco(float angleDeg)
{
    float rad = radians(angleDeg);
    float deplacementCM = (rad * (19 / 2));
    return round((TOUR_COMPLET_ENCO * deplacementCM) / CIRCON_ROUE_CM);
}

/*******************************************************************************************
 * Auteur : Antoine
 * 
 * Calcul la vitesse de déplacement en fonction de la distance restante au déplacement selon
 * une courbe sigmoide
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
 ******************************************************************************************/
double calculVitesse(float maxSpeed, uint32_t position, uint32_t positionFinal)
{
    double vit;
    double demi = positionFinal / 2;
    //double tier = positionFinal / 3;
    double quart = positionFinal/ 4;
    static byte count = 0;
    bool start = false;
    int sign = 0;
    maxSpeed >= 0 ? sign = 1: sign = -1;

    if (position < quart)
    { // si dans le premier quart du déplacement accélaire jusqu'a la vitesse requise
        double rate = (position / quart * 6.0) - 3.0;
        vit = 1 / (exp(-rate) + 1);
        //start = true;
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
    if (vit < 0.15 )
    {
        if(start == true)
        {
          vit = 0.015 * count;
          count++;
          start = false;
        }
        else
        {
          vit = 0.15;
          count = 0;
        }
        vit =! 0 ? vit = 0.15: vit = 0;
        return vit*sign;
    }
    else if (abs(vit) > 1)
    {
        return 1.0*sign;
    }
    else
    {
        count = 0;
        return vit;
    }
}

/*******************************************************************************************
 * Auteur : Antoine 
 * 
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
 ******************************************************************************************/
double pid(float error, float &lastError)
{
    static float integral = 0;
    static double lastMillis = 0;
    static double difTemps = 0;

    difTemps = (millis() - lastMillis)/1000;
    (difTemps <= 0)? difTemps = 0.000001:difTemps = difTemps;// si diff de temps = 0 on le change pour 1 milliSeconde

    lastMillis = millis();
    double proportional = error;
    integral += error * difTemps;
    (integral == NAN)? integral = 0:integral = integral;//si not a number on change pour 0
    
    double derive = (error - lastError) / difTemps;
    (derive == NAN)?derive = 0:derive = derive; //si not a number on change pour 0
    lastError = error;
    double out = (KP * proportional) + (KI * integral) + (KD * derive);
    return out;
}

/*******************************************************************************************
 * Auteur : Antoine
 * 
 * Description : Controle la vitesse des deux moteur tout en corrigeant le moteur droit
 *
 * @param vitesse (float [0 à 1]) vitesse des moteur
 *
 * @param direction (int) direction du robot : -1 = gauche, 0 = avance, 1 = droite
 *
 * @param &correction (float) correction à apporter a la roue droite
 *
 ******************************************************************************************/
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

    float setPoint = abs(ENCODER_Read(GAUCHE));
    float mesure = abs(ENCODER_Read(DROITE));
    float error = setPoint - mesure;
    static float previousError = 0;
    correction = correction + pid(error, previousError);
}

/*******************************************************************************************
 * Auteur : Antoine
 * 
 * Description : Fait avancer le robot d'une nombre de centimètre
 *
 * @param vitesse (float [0 à 0.8]) vitesse des moteur
 *
 * @param distanceCM (int) distance du déplacement [cm]
 *
 ******************************************************************************************/
void avance(int distanceCM, float vitesse){

  ENCODER_Reset(0);
  ENCODER_Reset(1);
  MOTOR_SetSpeed(0,0);
  MOTOR_SetSpeed(1,0);
  float correction = 0;
  float deplacement = distanceEnco(distanceCM);
  float setpoint = ENCODER_Read(GAUCHE);
  float lastErr = 0;
  float speedG = vitesse;
  float speedD = vitesse;
  while(deplacement - abs(setpoint) > 0){

    speedG = calculVitesse(vitesse , setpoint, deplacement);
    speedD = calculVitesse(vitesse + correction, setpoint, deplacement) ;
    setpoint = ENCODER_Read(GAUCHE);
    float real = ENCODER_Read(DROITE);
    float err = setpoint - real;
    correction = correction+pid(err,lastErr);
    Serial.println("");
    MOTOR_SetSpeed(GAUCHE, speedG);
    MOTOR_SetSpeed(DROITE, speedD);
    lastErr = err;
  }
  correction = 0;
  ENCODER_Reset(0);
  ENCODER_Reset(1);
  MOTOR_SetSpeed(0,0);
  MOTOR_SetSpeed(1,0);
}


/*******************************************************************************************
 * Auteur : Antoine
 * 
 * Description : Fait avancer le robot d'une nombre de centimètre
 * 
 * Define : TOUR_COMPLET = 360 deg,   QUART_DE_TOUR = 90 deg
 *
 * @param angleDeg (float) angle de rotation
 * 
 * @param vitesse (float [0 à 0.8]) vitesse des moteur
 *
 * @param direction (bool) direction de rotation: false = GAUCHE, true = DROITE
 *
 ******************************************************************************************/
void tourne(float angleDeg, float vitesse, bool direction){
  ENCODER_Reset(0);
  ENCODER_Reset(1);
  MOTOR_SetSpeed(0,0);
  MOTOR_SetSpeed(1,0);
  uint32_t deplacement = abs(angleEnco(angleDeg));
  float setpoint = abs(ENCODER_Read(GAUCHE));
  float lastErr;
  float speed = vitesse;
  while(deplacement - setpoint > 0) { 
  
    
    if(!direction){
      MOTOR_SetSpeed(GAUCHE, -1*calculVitesse(vitesse , setpoint, deplacement));
      MOTOR_SetSpeed(DROITE, calculVitesse(speed, setpoint, deplacement));
    }
    else if (direction){
      MOTOR_SetSpeed(GAUCHE, calculVitesse(vitesse , setpoint, deplacement));
      MOTOR_SetSpeed(DROITE, -1*calculVitesse(speed, setpoint, deplacement));
    }

    setpoint = abs(ENCODER_Read(GAUCHE));
    float real = abs(ENCODER_Read(DROITE));
    float err = setpoint - real;

    speed = speed + pid(err,lastErr);
    Serial.println("");
    lastErr = err;
   
  }

  ENCODER_Reset(0);
  ENCODER_Reset(1);
  MOTOR_SetSpeed(0,0);
  MOTOR_SetSpeed(1,0);
}

void vitesseRoues(float vitesse1, float vitesse2)
{
    MOTOR_SetSpeed(GAUCHE, vitesse1);
    MOTOR_SetSpeed(DROITE, vitesse2);
}






double angleArc(int encodeur){ 
  return encoToCM(encodeur)/DIST_ROUE;
}

double normeDeplacement(double angle){//mettre angleArc(ENCODEUR_READ())
  return 2*DIST_ROUE*cos(angle/2);//correspond a C
}

double angleDeplacement(double angle){//mettre angleArc(ENCODEUR_READ())
  return (180 - angle)/2;//correspond a omega
}



void positionRobot(vecteur& positionRobot, double gyro){
 // double arcR0 = angleArc(ENCODER_ReadReset(0));
  double arcR0 = angleArc(distanceEnco(10));
  //double arcR1 = angleArc(ENCODER_ReadReset(1));
  double arcR1 = angleArc(distanceEnco(10));
  
  //vecteur de la roue 1
  double normeR0 = normeDeplacement(arcR0);
  double angleR0 = angleDeplacement(arcR0);
  //composants roue 1
  double x_R0 = normeR0 *cos(angleR0);
  double y_R0 = normeR0 *sin(angleR0);
  
  //vecteur de la roue 2
  double normeR1 = normeDeplacement(arcR1);
  double angleR1 = angleDeplacement(arcR1);
  //composants roue 2
  double x_R1 = normeR1 *cos(angleR1);
  double y_R1 = normeR1 *sin(angleR1);

  //composant total
  double x;
  // if(x_R0 - x_R1 < 0.5&& x_R0 - x_R1 > -0.5 ){
  //   x = x_R0 - x_R1;
  // }
  x = (x_R0 - x_R1);
  double y = (y_R0 + y_R1);

  //composant ajustés avec l'orientation du robot
  double xAjust = x;//(x*cos(gyro)) + (y*sin(gyro));
  double yAjust = y;//(-x*sin(gyro)) + (y*cos(gyro));
  positionRobot.x = xAjust;
  positionRobot.y = yAjust;
  positionRobot.angle = atan2(positionRobot.y,positionRobot.x);
  positionRobot.norme =sqrt((positionRobot.x *positionRobot.x) + (positionRobot.y * positionRobot.y));
}



