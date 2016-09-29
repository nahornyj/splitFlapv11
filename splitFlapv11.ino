//conio
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
const bool calibrationOnly    = true;
float acceleration            = 10000.0;
const int nombreDeMoteur      = 8;
const int capteur[8]          = {11, 10, 9, 6, 5, 3, A0, A1};
const int frequenceCalibration = 5;                     //calibration toute les 20 animations
const int vitesseCalibration  = 1000;                    //vitesse calibration.
const int accelerationCalibration = 1000;
const int vitesseAnimation    = 100;
const int offSetZero          = 0;                       //toujours négatif
int nombreDEtape              = 8;                       //nombre d'étape dans une animation (commun a toutes les animations)
const int nombreAnimation     = 1;                       //il y a 1 animation dans la classe resources en ce moment.
const int temporaire          = 0;

//variables
int pointeur                  = 0;                       //numero du moteur en calibration
int numeroAnimation           = 99;                      //animation en cour
int curseur                   = 0;                       //barre ou curseur d'avance dans l'animation courante
int delayingTime              = 0;
int absPosStepper[8]          = {0, 0, 0, 0, 0, 0, 0, 0};//position absolu des steppers par rapport au zero défini dans la phase de calibration
bool calibrationBool          = true;                    //vrai pendant la calibration
int decompteMoteur            = 0;                       //valeur incrémentale, quand elle est égale au nombre de moteur, cela veut dire qu'ils ont tous fini leur animation
int actualTime                = 0;                       //utilitaire pour delay sans "delay()"
int targetTime                = 1;                       //utilitaire pour delay sans "delay()"
bool waitBool                 = false;                   //utilitaire pour delay sans "delay()"
bool capteurState[8]           = {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW}; //états des capteurs 0 = Passage de l'aimant | 1 = rien
int lastAnimation             = 0;                       //animation pécedente
int animationCounter          = 0;                       //compteur d'animation
bool precalib                 = true;
int counterSample             = 0;
int somme                     = 0;
bool aetesurzero              = false;
bool stopalalaprochaine       = false;
int positionPrecedente        = 0;
bool oneTime                  = true;
int aquisition                = 0;
Adafruit_StepperMotor *stepperContainer[8] = {
  AFMS4.getStepper(200, 1)/*1*/,
  AFMS3.getStepper(200, 1)/*2*/,
  AFMS2.getStepper(200, 1)/*3*/,
  AFMS1.getStepper(200, 1)/*4*/,
  AFMS1.getStepper(200, 2)/*5*/,
  AFMS2.getStepper(200, 2)/*6*/,
  AFMS3.getStepper(200, 2)/*7*/,
  AFMS4.getStepper(200, 2)/*8*/,
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
  s.begin(9600);

  //démarage des Shields
  AFMS1.begin();
  AFMS2.begin();
  AFMS3.begin();
  AFMS4.begin();

  //reglages des moteurs
  setGeneralValues(vitesseCalibration, accelerationCalibration);

  //démarage hall sensors

  for (int i = 0; i < nombreDeMoteur; i++) {
    pinMode(capteur[i], INPUT);
  }

  //setup movement
  setMovement(99); // animation de calibration pour commencer.
  //checkSensorPosition();
}

void loop() {


  if (calibrationBool) {
    //update moteur pour calibration
    calibration();
  } else {
    //update des animations en temps normal
    //updateAnimation();
   // compteurDeStep();
  }
}
void setMovement(int Id) {
  if (Id == 99) {
    calibrationBool == true;
    setGeneralValues(vitesseCalibration, accelerationCalibration); //éfinir une plus petite vitesse pour la calibration;

    for (int i = 0; i < nombreDeMoteur; i++) {
      aStepper[i].move(9999);   // plusieurs tours pour être sur d'atteindre le capteur.
    }
  } else {
    numeroAnimation = Id;
    setGeneralValues(vitesseAnimation, acceleration);

    for (int i = 0; i < nombreDeMoteur; i++) {
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


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//FONCTIONS UTILITAIRES///////////////////////////////////////////////////////////////////////////////////////////////////

void forwardstep1() { stepperContainer[0]->onestep(FORWARD, SINGLE);}
void backwardstep1() { stepperContainer[0]->onestep(BACKWARD, SINGLE);}
void forwardstep2() {  stepperContainer[1]->onestep(FORWARD, SINGLE);}
void backwardstep2() {  stepperContainer[1]->onestep(BACKWARD, SINGLE);}
void forwardstep3() { stepperContainer[2]->onestep(FORWARD, SINGLE);}
void backwardstep3() {  stepperContainer[2]->onestep(BACKWARD, SINGLE);}
void forwardstep4() { stepperContainer[3]->onestep(FORWARD, SINGLE);}
void backwardstep4() {  stepperContainer[3]->onestep(BACKWARD, SINGLE);}
void forwardstep5() {  stepperContainer[4]->onestep(FORWARD, SINGLE);}
void backwardstep5() {  stepperContainer[4]->onestep(BACKWARD, SINGLE);}
void forwardstep6() { stepperContainer[5]->onestep(FORWARD, SINGLE);}
void backwardstep6() {  stepperContainer[5]->onestep(BACKWARD, SINGLE);}
void forwardstep7() {  stepperContainer[6]->onestep(FORWARD, SINGLE);}
void backwardstep7() {  stepperContainer[6]->onestep(BACKWARD, SINGLE);}
void forwardstep8() {  stepperContainer[7]->onestep(FORWARD, SINGLE);}
void backwardstep8() {  stepperContainer[7]->onestep(BACKWARD, SINGLE);}

void setGeneralValues(float vitesse, float accel) {
  for (int i = 0; i < nombreDeMoteur; i++) {
    aStepper[i].setMaxSpeed(vitesse);
    aStepper[i].setAcceleration(accel);
  }
}

int deplacement(int ActualPosition, int Destination) {

  int stepToMove = 0;
  if (Destination != 9999) { // 9999 est une valeur de destination qui indique qu'il faut rester immobile.
    if (Destination < ActualPosition) { //si il faut repasser par la position 0 (origine)
      stepToMove = (200 - ActualPosition) + Destination;
    }
    if (Destination > ActualPosition) { //si il ne FAUT PAS repasser par la postion 0 (origine)
      stepToMove = Destination - ActualPosition;
    }
    if (Destination == ActualPosition) {
      stepToMove = 200;
    }
    if (Destination == 9999) {
      stepToMove = 0;
    }
  }
  return stepToMove;
}

void updateAnimation() {

  decompteMoteur = 0;
  for (int i = 0; i < nombreDeMoteur; i++) {
    if (aStepper[i].distanceToGo() == 0 ) {
      decompteMoteur++;
    }
    aStepper[i].run();
  }
  if (decompteMoteur == nombreDeMoteur && waitBool == false) {
    nextStep(delayingTime);
  }

  if (waitBool) {

    actualTime = millis();
    if (actualTime >= targetTime) {
      waitBool = false;
      delayingTime = 0;
      setMovement(numeroAnimation);
    }
  }
}

void  calibration() {
   //si sur 0 -> go to 100step et a partir de là va a 0.
  
   capteurState[pointeur] = digitalRead(capteur[pointeur]);

  if(capteurState[pointeur] == 1){
    //methode avec timing  
    if(aetesurzero){
      stopalalaprochaine = true;
    }
  }
  if(capteurState[pointeur] == 0 ){
    //j'ai besoin de faire un tour pour verif
    if(!stopalalaprochaine){
    counterSample++;
    aetesurzero = true;
    }else{
      pointeur++;
      aetesurzero        = false;
      stopalalaprochaine = false;
    }
  }
  if(pointeur < nombreDeMoteur){
    aStepper[pointeur].run();
  }else{
    s.println("fin calibration");
    calibrationBool = false;
    
  }
}

void nextStep(int DelayingTime) {
  int delayingTime = DelayingTime;

  //avance le curseur + remise à zero
  if (curseur < nombreDEtape - 1) {
    curseur++;
  } else {
    curseur = 0;
    setMovement(randomAnimation());
  }

  //sytème de delay
  if (DelayingTime == 0 ) {
    setMovement(numeroAnimation);
  }
  else {
    wait(delayingTime);
    waitBool = true;
  }
}

void wait(int DelayingTime) {
  int delayingTime = DelayingTime;
  actualTime = millis();
  targetTime = millis() + delayingTime;
}

int randomAnimation() {
  int value = 0;
  animationCounter++;

  if (animationCounter % frequenceCalibration == 0) {
    value = 99;
    calibrationBool = true;
  } else {
    value = floor(random(0, nombreAnimation - 1));
    value = 0;
  }
  lastAnimation = value;

  return value;
}

void checkSensorPosition(){
  for(int i=0;i<8;i++){
    capteurState[i] = digitalRead(capteur[i]);  
   }
}

void compteurDeStep(){
 
   if(digitalRead(capteur[4]) == LOW && oneTime){  
     oneTime = false;
      int value = aStepper[4].currentPosition()-positionPrecedente;
      s.println(value);   
     positionPrecedente = aStepper[4].currentPosition();
    }
    if(digitalRead(capteur[4]) == HIGH){
      oneTime = true;
    } 
}
