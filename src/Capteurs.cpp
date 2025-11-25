
#include "Capteurs.h"
#include "LibRobus.h"
// Suiveur de ligne
int seuilGauche = 875;
int seuilCentre = 875;
int seuilDroite = 875;
//créer l'objet pour le 6050
Adafruit_MPU6050 mpu;
// variables de l'accéléromètre
double pos_x= POSX_R1;
double pos_y= POSY_R2;
double temps= 0;
double temps_prec= 0;
double angle= PI/2; //rechanger à 0 pour faire marcher le code actu_pos4 
double angle_prec= PI/2; 
double integ_a[2]= {0,0};
double integ_v[2]= {0,0};
float acc[2]= {0,0};
float vit_prec[2]={0,0};
float err_accx=0;
float err_accy=0;//rechanger après
float err_rot=0;
double vit1=0;
double vit2=0;
double anglef=0;
double xi1=0; //possible de changer ces valeurs pour changer la position initiale du robot, la roue gauche est à 0 0 au départ
double xf1=0;
double yi1=0;
double yf1=0;
double xi2=18.75;
double xf2=18.75;
double yi2=0;
double yf2=0;
double cx=0;
double cy=0;
double dep1=0;
double dep2=0;

double dist=DIST;
double theta1=0;
double w1=0;
double c1=0;
double r1=0;
double theta2=0;
double w2=0;
double c2=0;
double r2=0;
double vmx=0;
double vmy=0;
double norv=0;
double thetav=0;

// Capteur de couleur test
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_4X); // Définit capteur et caractéristiques d'utilisation

/*******************************************************************************************
 * Auteur : Justin
 * 
 * fait une lecture du mpu sur l'axe voulu (0=x, 1=y, 2=z), en rotation ou translation(0=translation, 1=rotation)
 * 
 * @return la valeur demandée (double)
 ******************************************************************************************/
double mpu_lect(int axe, int mode){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  double ret = 0;
  if (mode==0){
    switch (axe)
    {
    case 0:
      ret = (double) a.acceleration.x-err_accx;
      return ret;
    case 1:
      ret = (double) a.acceleration.y-err_accy;
      return ret;
    case 2:
      ret = (double) a.acceleration.z;
      return ret;
  }
}
switch (axe)
    {
    case 0:
      ret = (double) g.gyro.x;
      return ret;
    case 1:
      ret = (double) g.gyro.y;
      return ret;
    case 2:
      ret = (double) g.gyro.z;
      return ret;
  }
}
/*******************************************************************************************
 * Auteur : Justin
 * 
 * fait la moyenne de 8 lectures du mpu sur l'axe voulu (0=x, 1=y, 2=z), en rotation ou translation(0=translation, 1=rotation)
 * 
 * @return la valeur demandée (double)
 ******************************************************************************************/
double mpu_get(int axe, int mode){
  double moy=0;
  for (int i=0; i<=7;i++){
    moy += mpu_lect(axe, mode);
  }
  moy/=8;
  if (abs(moy)>=0.05){
    return moy;
  }
  return 0;
}
/*******************************************************************************************
 * Auteur : Justin
 * 
 * actualise la position du robot, doit être appelé à chaque exécution de loop()
 * 
 * @return rien
 ******************************************************************************************/
void actu_pos(){
  temps= ((double) millis())/1000;
  angle +=mpu_get(2,1)*(temps-temps_prec); //on ajoute la rotation en sens anti-horaire du robot selon l'axe z négatif
  acc[1]=mpu_get(1,0);
  integ_a[1]=(temps-temps_prec)*acc[1];
  integ_v[1]=(temps-temps_prec)*(vit_prec[1]+(integ_a[1]/2));
  vit_prec[1]+=integ_a[1];
  pos_x+=integ_v[1]*cos(angle);
  pos_y+=integ_v[1]*sin(angle);
  temps_prec=temps;
  Serial.println(vit_prec[1]);
}
void actu_pos2(){
  temps= ((double) millis())/1000;
  angle -=(mpu_get(2,1))*(temps-temps_prec); //on ajoute la rotation en sens anti-horaire du robot selon l'axe z négatif
  for (int i=0; i<=1;i++){
    acc[i]=mpu_get(i,0);
    integ_a[i]=(temps-temps_prec)*acc[i];
    integ_v[i]=(temps-temps_prec)*(vit_prec[i]+integ_a[i]);
    vit_prec[i]+=integ_a[i];
  }
  pos_x+=integ_v[1]*cos(angle);
  pos_y+=integ_v[1]*sin(angle);
  temps_prec=temps;
}
void actu_pos3(){
  temps= ((double) millis())/1000;
  double delta_time=temps-temps_prec;
  pos_y+=delta_time*delta_time*mpu_get(1,0);

  temps_prec=temps;
}
void actu_pos4(){ //écrire à justin si vous avez besoin d'aide 
  temps=((double) millis())/1000; //le temps en secondes
  dep1=((double) ENCODER_ReadReset(0)*CMPT)/3200; //les déplacements des roues en mm
  dep2=((double) ENCODER_ReadReset(1)*CMPT)/3200;
  vit1=dep1/(temps-temps_prec);          //les vitesses des roues en mm/sec
  vit2=dep2/(temps-temps_prec);
  if(vit1/vit2<0){
    r1= dist/(1-vit2/vit1);
    r2=r1*-1*vit2/vit1;
    cy=yi1+((yi2-yi1)*r1/dist);
    cx=xi1+((xi2-xi1)*r1/dist);
    anglef=angle+(dep1/(2*PI*r1));
    if(abs(r2)>abs(r1)){
      xf1=cx-(r1*cos(anglef));
      yf1=cy-(r1*sin(anglef));
      xf2=cx-(r2*cos(anglef));
      yf2=cy-(r2*sin(anglef));
    }
    else{
      xf1=cx+(r1*cos(anglef));
      yf1=cy+(r1*sin(anglef));
      xf2=cx+(r2*cos(anglef));
      yf2=cy+(r2*sin(anglef));
    }
  }
  else if (vit1/vit2==1||(vit1==0&&vit2==0)){
    xf1=xi1+(dep1*cos(anglef+(PI/2)));
    yf1=yi1+(dep1*sin(anglef+(PI/2)));
    xf2=xi2+(dep2*cos(anglef+(PI/2)));
    yf2=yi2+(dep2*sin(anglef+(PI/2)));
  }
  else {
    if (vit1==0){
      r1=0;
      r2=dist;
      anglef=angle+(dep2/2*PI*r2);
    }
    else{
      r1=dist/(abs(1-(vit2/vit1)));
      r2=r1*vit2/vit1;
      anglef=angle+(dep1/(2*PI*r1));
    }
    if(r1>600){
      xf1=xi1+(dep1*cos(anglef+(PI/2)));
      yf1=yi1+(dep1*sin(anglef+(PI/2)));
      xf2=xi2+(dep2*cos(anglef+(PI/2)));
      yf2=yi2+(dep2*sin(anglef+(PI/2)));
    }
    else{
    if (abs(r1)>abs(r2)){
      r2=r1-dist;
      cy=yi1+((yi2-yi1)*r1/dist);
      cx=xi1+((xi2-xi1)*r1/dist);
      xf1=cx-(r1*cos(anglef));
      yf1=cy-(r1*sin(anglef));
      xf2=cx-(r2*cos(anglef));
      yf2=cy-(r2*sin(anglef));
    }
    else{
      r2=r1+dist;
      cy=yi1+((yi1-yi2)*r1/dist);
      cx=xi1+((xi1-xi2)*r1/dist);
      xf1=cx+(r1*cos(anglef));
      Serial.println(xf1);
      Serial.println("\n");
      yf1=cy+(r1*sin(anglef));
      xf2=cx+(r2*cos(anglef));
      yf2=cy+(r2*sin(anglef));
    }
  }
  }
  double norme=sqrt(pow((xf2-xf1),2)+pow((yf2-yf1),2));
  xf2=xf1+(((xf2-xf1)/norme)*dist);
  yf2=yf1+(((yf2-yf1)/norme)*dist);
  xi1=xf1;
  xi2=xf2;
  yi1=yf1;
  yi2=yf2;
  temps_prec=temps;
  angle=anglef;
  Serial.println(vit1);
  Serial.println(vit2);
  Serial.println("rayons");
  Serial.println(r1);
  Serial.println(r2);
  Serial.println("centre");
  Serial.println(cx);
  Serial.println(cy);
  Serial.println("angles");
  Serial.println(angle);
  Serial.println(anglef);
  Serial.println("coordonnées");
  Serial.println(xf1);
  Serial.println(yf1);
  Serial.println(xf2);
  Serial.println(yf2);
}
void actu_pos5(){
  temps=millis()/1000;
  dep1= (double) ENCODER_ReadReset(0)*CMPT/3200;
  dep2= (double) ENCODER_ReadReset(1)*CMPT/3200;
  theta1=dep1/dist;
  theta2=dep2/dist;
  c1=2*dist*cos(theta1/2);
  c2=2*dist*cos(theta1/2);
  w1=(PI-theta1)/2;
  w2=(PI-theta2)/2;
  vmx=(c1*cos(w1))-(c2*cos(w2));
  vmy=(c1*sin(w1)+(c2*sin(w2)));
  norv=sqrt(pow(vmx,2)+pow(vmy,2));
  thetav=atan(vmy/vmx);
  if(vmx<0){
    thetav*=-1;
  }
  pos_x+=norv*cos(angle-(PI/2)+thetav);
  pos_y+=norv*cos(angle-(PI/2)+thetav);
  angle=angle-PI/2+thetav;
  Serial.println(c1*cos(w1));
  Serial.println(c2*cos(w2));
}
/*******************************************************************************************
 * Auteur : Justin
 * 
 * renvoie la position x(0) ou y (1) du robot;
 * 
 * @return rien
 ******************************************************************************************/
double get_pos(int axe){ //à rechanger pour posx et posy si on retourne sur laccelero
  if (axe==0){
    return pos_x;
  }
  return pos_y;
}
/*******************************************************************************************
 * Auteur : Justin
 * 
 * renvoie le temps depuis l'allumage du robot en secondes;
 * 
 * @return rien
 ******************************************************************************************/
double get_temps(){
  return temps;
}
/*******************************************************************************************
 * Auteur : Justin
 * 
 * renvoie la position x(0) ou y (1) du robot;
 * 
 * @return rien
 ******************************************************************************************/
double get_or(){
  return angle;
}
/*******************************************************************************************
 * Auteur : Justin
 * 
 * initialise le mpu, plante le robot si l'initialisation foire
 * si appelée avec 1 en argument, la fonction fait aussi une calibration selon les 
 * conditions initiales du robot qui changera les retours de mpu_lect. sinon les 
 * valeurs retournées seront les valeurs brutes du capteur
 * 
 * @return rien
 ******************************************************************************************/
void mpu_init(int calibration){
  float err_accx_temp=0;
  float err_accy_temp=0;
  float err_rot_temp=0;
  while (!Serial){
    delay(10);
  }
  if (!mpu.begin()) {
    Serial.println("Justin est cave");
    while (1) {
      delay(10);
    }
  }
  Serial.println("On performe");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
  Serial.println(""); // jsp pourquoi faut mettre ça
  delay(100);
  if (calibration==1){
  for (int i=0; i<=511;i++){
    err_accx_temp+=mpu_lect(0,0);
    err_accy_temp+=mpu_lect(1,0);
    err_rot_temp+=mpu_lect(2,1);
  }
  err_accx=err_accx_temp/512;
  err_accy=err_accy_temp/512;
  err_rot=err_rot_temp/512;
}
}
void initialiserTemps(){
  temps=millis()/1000;
  temps_prec=millis()/1000;
  pos_x=0;
  pos_y=0;
  angle=0;
  angle_prec=0;
  acc[1]=0;
  vit_prec[1]=0;
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

    Serial.print("C:\t"); Serial.print(clear);
    Serial.print("\tR:\t"); Serial.print(red);
    Serial.print("\tG:\t"); Serial.print(green);
    Serial.print("\tB:\t"); Serial.print(blue);
    Serial.println();

    // Figure out some basic hex code for visualization
    // uint32_t sum = clear;
    // float r, g, b;
    // r = red; r /= sum;
    // g = green; g /= sum;
    // b = blue; b /= sum;
    // r *= 256; g *= 256; b *= 256;

    
    if(red >= 700 && green >= 820)
    {
      return COULEURJAUNE;
    }
    if(red >= green && red >= blue && red >= 700)
    {
      return COULEURROUGE;
    }
    if(green >= blue && green >= red && green >= 850)
    {
       return COULEURVERT;
    }
    if(blue >= 700)
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
