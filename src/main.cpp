/*
Projet: Arduino Lasagne
Equipe: Iron Men
Description: Code pour l'arduino du RobUS controller par la manette
Date: 11/13/2025
*/

/* ****************************************************************************
Inclure les librairies de functions que vous voulez utiliser
**************************************************************************** */
#include "main.h"
/* ****************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */
Adafruit_MPU6050 mpu;
vecteur position;

// void setup()
// {
//   Serial.begin(9600);
//   while (!Serial)
//     delay(10); // will pause Zero, Leonardo, etc until serial console opens

//   Serial.println("Adafruit MPU6050 test!");

//   // Try to initialize!
//   if (!mpu.begin()) {
//     Serial.println("Failed to find MPU6050 chip");
//     while (1) {
//       delay(10);
//     }
//   }
//   Serial.println("MPU6050 Found!");

//    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
// }

// /* ****************************************************************************
// Fonctions de boucle infini (loop())
// **************************************************************************** */
 unsigned long currentMillis = 0;
 unsigned long delai = 1000;
 unsigned long lastTemps = 0;

// void loop()
// {
//     sensors_event_t a, g, temp;
//     mpu.getEvent(&a, &g, &temp);
//   double diffTemps = (millis() - lastTemps);
//   lastTemps = millis();

//   if(millis()-currentMillis > delai){
//   currentMillis = millis();
//   //Serial.println("gyro angle: " + String(g.gyro.z));
//   static double angle = 0;
//   double rot;
//   rot = ((g.gyro.z + 0.03)*diffTemps);
//   angle += rot;
//   Serial.print("gyro z: "); Serial.println(g.gyro.z);
//   Serial.print("angle: "); Serial.println(angle);
  
//  // Serial.println("diffTemps: " +String(diffTemps));
//   Serial.println("");

//   //Serial.println("angle: " + String(angle));
// }
// //   positionRobot(position, g.gyro.z);
// //   if(millis()-currentMillis > delai){
// //     currentMillis = millis();
// //       Serial.println("x = : " + String(position.x));
// //       Serial.println("y = : " + String(position.y));
// //     }


// }

//#########################################################################################
//FONCTIONNE MEGNÉTO
// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_LIS2MDL.h>
 

// Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);
 
// float max_x = 65.25;
// float max_y = 60.60;
// float max_z = 16.5;

// float min_x = -34.35;
// float min_y = -33.90;
// float min_z = -81.00;

// float refMax_x = 45.46;
// float refMax_y = 46.07;
// float refMax_z = 44.33;

// float refMin_x = -49.7;
// float refMin_y = -46.79;
// float refMin_z = -48.43;
// void setup(void)
// {
//   Serial.begin(115200);
//   Serial.println("Magnetometer Test"); Serial.println("");
 
//   /* Initialise the sensor */
//   if(!mag.begin())
//   {
//     /* There was a problem detecting the LIS2MDL ... check your connections */
//     Serial.println("Ooops, no LIS2MDL detected ... Check your wiring!");
//     while(1);
//   }
// }
 
// void loop(void)
// {
//   /* Get a new sensor event */
//   sensors_event_t event;
//   mag.getEvent(&event);
 
//   float Pi = 3.14159;
//   float mag_y = (((event.magnetic.y- min_y)*(refMax_y-refMin_y))/(max_y - min_y)) + refMin_y;
//   float mag_x = (((event.magnetic.x- min_x)*(refMax_x-refMin_x))/(max_x - min_x)) + refMin_x;
 
//   // Calculate the angle of the vector y,x
//   float heading = (atan2(mag_y,mag_x) * 180) / Pi;
 
//   // Normalize to 0-360
//   if (heading < 0)
//   {
//     heading = 360 + heading;
//   }
//   Serial.print("Compass Heading: ");
//   Serial.println(heading);
//   delay(500);
// }

vecteur robot;
 void setup(){
    Serial.begin(9600);
    // int32_t distG = distanceEnco(40);
    // int32_t distD = distanceEnco(10);

    // BoardInit();
    // ENCODER_Reset(0);
    // ENCODER_Reset(1);
    // while(ENCODER_Read(0) < distG ){
    //     Serial.println(distG);
    //     Serial.println(ENCODER_Read(0));
    //     Serial.println("---------------------------");

    //     MOTOR_SetSpeed(0,0.25);
    //     MOTOR_SetSpeed(1,0.5);
    // }
    
    // MOTOR_SetSpeed(0,0);
    // MOTOR_SetSpeed(1,0);

    positionRobot(robot, 0);
    Serial.print("deplacement en x : ");Serial.println(robot.x);
    Serial.print("deplacement en y : ");Serial.println(robot.y);
 }

 void loop(){

 }