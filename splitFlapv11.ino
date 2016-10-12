#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <AnimationStepper.h> //library perso
#include <SoftwareSerial.h>

SoftwareSerial s(12, 13); // RX, TX

Adafruit_MotorShield AFMS1(0x60);                         // Default address, no jumpers 00000
Adafruit_MotorShield AFMS2(0x61);                         // 00001
Adafruit_MotorShield AFMS3(0x62);                         // 00010
Adafruit_MotorShield AFMS4(0x63);                         // 00011

//variables statiques
const bool boolRelease         = false;                   //fonction release;
const bool debug               = true;
const int debugAnim           = 0;
//const int box                 = 1;                                                                                   //numéro de la boite :)
const int offSetZero          = 0;                                                                                   //offset général à la suite de la calibration
//const int boxOffset[3][8]     = {{0, 0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0, 0},    {0, 0, 0, 0, 0, 0, 0, 0}}; //offset pécis

const bool calibrationOnly    = true;
const int nombreDeMoteur      = 8;
const int capteur[8]          = {3, 10, 9, 6, 5, 11, A0, A1};
                                                         //05,01,02,03,04,00,I0,I1
                                                         //c1,c2,c3,c4,c5,c6,c7,c8
                                                         //racourcir de :36,  0,16, 0, 0,28, 18,  0;
const int boundaryTiming[2]    = {5,10};
const int frequenceCalibration = 20;                     //calibration toute les 20 animations
const int vitesseCalibration  = 100;                    //vitesse calibration.
const int accelerationCalibration = 1000;
const int vitesseAnimation    = 100;
float acceleration            = 1000.0;
int nombreDEtape              = 17;                       //nombre d'étape dans une animation (commun a toutes les animations)
const int nombreAnimation     = 7;                       //il y a 3 animation dans la classe resources en ce moment.
const int temporaire          = 0;

//variables
int pointeur                  = 0;                       //numero du moteur en calibration
int numeroAnimation           = 99;                      //animation en cour
int curseur                   = 0;                       //barre ou curseur d'avance dans l'animation courante
int delayingTime              = 0;
int absPosStepper[8]          = {0, 0, 0, 0, 0, 0, 0, 0};//position absolu des steppers par rapport au zero défini dans la phase de calibration
bool calibrationBool          = true;                    //vrai pendant la calibration
int decompteMoteur            = 0;                       //valeur incrémentale, quand elle est égale au nombre de moteur, cela veut dire qu'ils ont tous fini leur animation
unsigned long actualTime                = 0;                       //utilitaire pour delay sans "delay()"
unsigned long targetTime                = 1;                       //utilitaire pour delay sans "delay()"
bool waitBool                 = false;                   //utilitaire pour delay sans "delay()"
bool capteurState[8]           = {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW}; //états des capteurs 0 = Passage de l'aimant | 1 = rien
int lastAnimation[3]           = {99,99,99} ;            //animation pécedente
int animationCounter          = 0;                       //compteur d'animation
int positionPrecedente        = 0;
bool oneTime                  = true;
bool addoffset                = false;
bool arretSurZero             = false;
bool premiereCaptation        = true;
int multipleRandom[20]        = {99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99};
bool oneTimeDelay             = true;
unsigned long targetTimeAnim  = 0;
bool oneTimeDelayAnim         = true;
Adafruit_StepperMotor *stepperContainer[8] = {
  AFMS1.getStepper(200, 2)/*1*/,
  AFMS2.getStepper(200, 2)/*2*/,
  AFMS3.getStepper(200, 2)/*3*/,
  AFMS4.getStepper(200, 2)/*4*/,
  AFMS1.getStepper(200, 1)/*5*/,
  AFMS2.getStepper(200, 1)/*6*/,
  AFMS3.getStepper(200, 1)/*7*/,
  AFMS4.getStepper(200, 1)/*8*/, 
};

AccelStepper temp1(forwardstep1, backwardstep1);
AccelStepper temp2(forwardstep2, backwardstep2);
AccelStepper temp3(forwardstep3, backwardstep3);
AccelStepper temp4(forwardstep4, backwardstep4);
AccelStepper temp5(forwardstep5, backwardstep5);
AccelStepper temp6(forwardstep6, backwardstep6);
AccelStepper temp7(forwardstep7, backwardstep7);
AccelStepper temp8(forwardstep8, backwardstep8);

AccelStepper aStepper[nombreDeMoteur] = {temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8};

//constructeur classe externe
AnimationStepper anim;

void setup() {
 
  //Serial
  Serial.begin(9600);

  //démarage des Shields
  AFMS1.begin();
  AFMS2.begin();
  AFMS3.begin();
  AFMS4.begin();
  
   setGeneralValues(vitesseCalibration,accelerationCalibration);
  //initialisation des steppeurs en mode rapide.
  for (int i = 0; i < nombreDeMoteur; i++) { 
    pinMode(capteur[i], INPUT);
    stepperContainer[i]->quickstepInit();
  }
  

  //setup movement
  setMovement(99); // animation de calibration pour commencer.
}

void loop() {
  
  if (calibrationBool) {
    //update moteur pour calibration
    calibration();

  } else {
    //update des animations en temps normal
    updateAnimation(); 
  }
}


void setMovement(int Id) {
  if (Id == 99) {
    calibrationBool == true;
    for (int i = 0; i < nombreDeMoteur; i++) {
      aStepper[i].setSpeed(vitesseCalibration);
      aStepper[i].setAcceleration(accelerationCalibration);
      aStepper[i].move(9999);   // plusieurs tours pour être sur d'atteindre le capteur.
    }
  } else {
    numeroAnimation = Id;
    for (int i = 0; i < nombreDeMoteur; i++) {
       aStepper[i].setSpeed(vitesseCalibration);
      aStepper[i].setAcceleration(acceleration);
      aStepper[i].move(deplacement(absPosStepper[i], anim.getValue(numeroAnimation, curseur, i)));

      //update absPos
      absPosStepper[i] += deplacement(absPosStepper[i], anim.getValue(numeroAnimation, curseur, i));
      absPosStepper[i] = absPosStepper[i] % 200;
    }

    if (anim.getValue(numeroAnimation, curseur, 8) != 0) { // si il y a du delay
      delayingTime = anim.getValue(numeroAnimation, curseur, 8);
    } else {
      delayingTime = 0;
    }
  }
}

void forwardstep1() {  stepperContainer[0]->quickstep(FORWARD);}
void backwardstep1() {  stepperContainer[0]->quickstep(BACKWARD);}
void forwardstep2() {  stepperContainer[1]->quickstep(FORWARD);}
void backwardstep2() {  stepperContainer[1]->quickstep(BACKWARD);}
void forwardstep3() {  stepperContainer[2]->quickstep(FORWARD);}
void backwardstep3() {  stepperContainer[2]->quickstep(BACKWARD);}
void forwardstep4() { stepperContainer[3]->quickstep(FORWARD);}
void backwardstep4() { stepperContainer[3]->quickstep(BACKWARD);}
void forwardstep5() { stepperContainer[4]->quickstep(FORWARD);}
void backwardstep5() {  stepperContainer[4]->quickstep(BACKWARD);}
void forwardstep6() {  stepperContainer[5]->quickstep(FORWARD);}
void backwardstep6() {  stepperContainer[5]->quickstep(BACKWARD);}
void forwardstep7() {  stepperContainer[6]->quickstep(FORWARD);}
void backwardstep7() {  stepperContainer[6]->quickstep(BACKWARD);}
void forwardstep8() {  stepperContainer[7]->quickstep(FORWARD);}
void backwardstep8() {  stepperContainer[7]->quickstep(BACKWARD);}

int deplacement(int ActualPosition, int Destination) {

  int stepToMove = 0;
  if (Destination != 9999) {                                     // 9999 est une valeur de destination qui indique qu'il faut rester immobile.
    if (Destination < ActualPosition) {                          //si il faut repasser par la position 0 (origine)
      stepToMove = (200 - ActualPosition) + Destination;
    }
    if (Destination > ActualPosition) {                          //si il ne FAUT PAS repasser par la postion 0 (origine)
      stepToMove = Destination - ActualPosition;
    }
  }
  return stepToMove;
}

void updateAnimation() { 
 
    if (decompteMoteur == nombreDeMoteur) {      
        nextStep();         
    }
    else{  
  decompteMoteur = 0;    
        for (int i = 0; i < nombreDeMoteur; i++) {
          if (aStepper[i].distanceToGo() == 0 ) {
            decompteMoteur++;
          }
          aStepper[i].run();
        }    
    }
}

void nextStep() {
  actualTime = millis();
   Serial.print(" targetTimeAnim : ");
   Serial.print(targetTimeAnim);
   Serial.print(" | actualTime : ");
   Serial.println(actualTime);
       
    if (delayingTime == 0 ) {       
      if (curseur < nombreDEtape - 1) {
        curseur++;
        setMovement(numeroAnimation);
        decompteMoteur = 0;
      } else {     
      
        if(oneTimeDelayAnim){
          targetTimeAnim = millis() + floor(random(boundaryTiming[0] ,boundaryTiming[1] ));;
          actualTime = millis();
          oneTimeDelayAnim = false;
        }
        if(actualTime > targetTimeAnim){
          oneTimeDelayAnim = true;
          curseur = 0;   
          decompteMoteur = 0;
          setMovement(randomAnimation()); 
        }
      }    
      
      }else{  
      if(oneTimeDelay){ 
        targetTime = millis() + delayingTime;
        if(boolRelease){  
          for (int i = 0; i < nombreDeMoteur; i++) {
            stepperContainer[i]->release();
          }  
        }
        oneTimeDelay = false;
      }  
      if(actualTime > targetTime ){
         delayingTime = 0;
         oneTimeDelay = true;
          if(boolRelease){ 
             for(int i = 0;i<nombreDeMoteur;i++){
                stepperContainer[i]->quickstepInit();
             }
          }
         decompteMoteur = 0;      
      }    
    }  
}


int randomAnimation() {
  int value = 0;
  animationCounter++;
  if (animationCounter % frequenceCalibration == 0) {
    value = 99;
    calibrationBool = true; 
  } else {
    for(int i = 0; i< sizeof(multipleRandom);i++){
      multipleRandom[i] = floor(random(0,nombreAnimation-1));
      if( multipleRandom[i] != lastAnimation[0] && multipleRandom[i] != lastAnimation[1] && multipleRandom[i] != lastAnimation[2]){
        value = multipleRandom[i];
        i = 0;
        break;
      }     
    }     
   
    if(debug){
    
      value = floor(random(0,7));
    }
    calibrationBool = false;
  }
  
  //décalage
  lastAnimation[0] = lastAnimation[1];
  lastAnimation[1] = lastAnimation[2];
  lastAnimation[2] = value;
  Serial.print("random Animation ");
  Serial.println(value);
  return value;
}

void  calibration() {
   

  ///////////////////////////////////////////////////////////// SAFE EXIT SI JAMAIS UN  CAPTEUR MERDE OU UN MOTEUR
  if(aStepper[pointeur].currentPosition() > 1000){
     aStepper[pointeur].move(0);
    aStepper[pointeur].setCurrentPosition(0);
    absPosStepper[pointeur] = 0;
    pointeur++;

    addoffset = false;
    premiereCaptation = true;
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  capteurState[pointeur] = digitalRead(capteur[pointeur]);
  if (aStepper[pointeur].distanceToGo() == 0 && !premiereCaptation && !addoffset) {
    //a fais soit 9999 step, soit 100 step depuis zero et donc bool changing
    aStepper[pointeur].move(200);
    //arret sur le prochain zero
    arretSurZero = true;
  }
  if (capteurState[pointeur] == 0 ) {
    //j'ai besoin de faire un 1/2 tour pour verif
    if (premiereCaptation) {
      aStepper[pointeur].move(100);
      premiereCaptation = false;
    }
    if (arretSurZero) {
      arretSurZero = false;
      aStepper[pointeur].move(10);
      addoffset = true;
    }
  }
  if (addoffset && aStepper[pointeur].distanceToGo() == 0) {
    aStepper[pointeur].move(0);
    aStepper[pointeur].setCurrentPosition(0);
    absPosStepper[pointeur] = 0;
    pointeur++;

    addoffset = false;
    premiereCaptation = true;
  }

  if (pointeur < nombreDeMoteur) {
    aStepper[pointeur].run();
  } else {
    //s.println("fin calibration");
    waitBool = false;
    calibrationBool = false;
    setMovement(randomAnimation());
    pointeur = 0;

  }
}

/*
void checkSensorPosition() {
  for (int i = 0; i < 8; i++) {
    capteurState[i] = digitalRead(capteur[i]);
  }
}
void compteurDeStep() {
  if (digitalRead(capteur[4]) == LOW && oneTime) {
    oneTime = false;
    int value = aStepper[4].currentPosition() - positionPrecedente;
    //s.println(value);
    positionPrecedente = aStepper[4].currentPosition();
  }
  if (digitalRead(capteur[4]) == HIGH) {
    oneTime = true;
  }
}*/
void setGeneralValues(float vitesse, float accel) {
  for (int i = 0; i < nombreDeMoteur; i++) {
    aStepper[i].setMaxSpeed(vitesse);
    aStepper[i].setAcceleration(accel);
  }
}
