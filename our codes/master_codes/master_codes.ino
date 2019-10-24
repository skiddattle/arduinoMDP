#include <SharpIR.h>
#include <MsTimer2.h>
#include "PID_v1.h"
#include <EnableInterrupt.h>
#include <DualVNH5019MotorShield.h>

/*  Table of contents:
    1. INITIALISATION
    2. SETTINGS
    3. SETUP & LOOP
    4. MOTORLIB
    5. ALIGNMENT + SENSORSLIB
    
*/

/* ========================= INITIALISATION =======================================*/
#define IR1 A0 // define signal pin
#define IR2 A1 // define signal pin
#define IR3 A2 // define signal pin
#define IR4 A3 // define signal pin
#define IR5 A4 // define signal pin
#define IR6 A5 // define signal pin

#define sensor1 1
#define sensor2 2 
#define sensor3 3 
#define sensor4 4 
#define sensor5 5 
#define sensor6 6

SharpIR SharpIR1(IR1, sensor1);
SharpIR SharpIR2(IR2, sensor2);
SharpIR SharpIR3(IR3, sensor3);
SharpIR SharpIR4(IR4, sensor4);
SharpIR SharpIR5(IR5, sensor5);
SharpIR SharpIR6(IR6, sensor6);

char lastRPIcommand;

/*Declare new variable Orientation. 
    Orientation tells us which direction robot is facing, north south east or west
    0=north
    1=east
    2=south
    3=west
*/
int orientation = 0;
int vert_counter = 0;       
int hori_counter = 0;
int consecForwardcounter = 0; //maybe remove if redundant

/* Used to reduce sensors reading*/
float prevIR1reading = -1;
float prevIR2reading = -1;
float prevIR3reading = -1;
float prevIR4reading = -1;
float prevIR5reading = -1;

//used to keep track of left blocks
boolean middleLeftisBlock = false;

/* ================================ SETTINGS ======================================*/
int alignThreshold = 5;     //4
int forwardThreshold = 5;   //threshold for wallAlign only, increment only if going forward. reset when alignment done

String RPIcommand;

/* ================================ SETUP() AND LOOP() ======================================*/
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  setupMotorEncoder();
  setupPID();
  delay(20);

}

boolean readSomething = false;

void loop() {
  delay(2);
  startListening();
}


void startListening() {
      
//  String RPIcommand;
  long int DistNum;
  while (Serial.available()) {
    RPIcommand = readStr();
    DistNum = RPIcommand.substring(1).toInt();
    if (RPIcommand[0] == 'w') {
      
      moveForward(DistNum);
      
      checkLeftAlign();
      checkFrontAlign();

      lastRPIcommand = 'w';
      done();
    }
    if (RPIcommand[0] == 'd') {
      vert_counter += alignThreshold;       //this is to fix double align
      hori_counter += alignThreshold;
      
      checkLeftAlign();
      checkFrontAlign();
      
      rotateRight(DistNum);

      lastRPIcommand = 'd';
      done();
      
    }
    if (RPIcommand[0] == 's') {
        moveBackward(DistNum);
        lastRPIcommand = 's';
      done();
      
    }
    if (RPIcommand[0] == 'a') {
      vert_counter += alignThreshold;       //this is to fix double align
      hori_counter += alignThreshold;
      
      checkFrontAlign();
      middleLeftisBlock = false;
      
      rotateLeft(DistNum);

      lastRPIcommand = 'a';
      done();
    }
    if (RPIcommand[0] == 'z') {
      sensorReading(DistNum);
      lastRPIcommand = 'z';
    }
    
  }
}


void runTests() {
//  alignUnevenFront(&sensortwo,&sensorthree);
//  moveForward(17);
//  delay(1000);
//  rotateRight(2);
//  delay(1000);

//              
//  
    delay(2000);
    wallAlign(&sensorone,&sensorthree);
//    rotateRight(2);
//    delay(1000);
//    rotateRight(2);
//    delay(1000);
//    rotateRight(2);
//    delay(1000);
//    rotateRight(2);
//  int  test = 4;
//  while(test>0){
//    moveForward(1);//comment out
//    delay(1500);
//    test--;
//  }
//  rotateLeft(2);
//  delay(1500);
//  test = 4;
//  while(test>0){
//    moveForward(1);//comment out
//    delay(1500);
//    test--;
//  }
//  
//  rotateLeft(2);
//  test = 4;
//  while(test>0){
//    moveForward(1);//comment out
//    delay(1500);
//    test--;
//  }
}


String readStr() {
  String Str = "";
  char Character;
  while (Serial.available()) {
    Character = Serial.read();
    Str.concat(Character);
    delay(10);
  }

  if (Str != "") {
    return Str;
  }
}

void done() {
  Serial.println("done");
}
