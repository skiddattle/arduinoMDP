#include "DualVNH5019MotorShield.h"
#include <EnableInterrupt.h>
#include "PID_v1.h"
#include "SharpIR.h"

#define ENRIGHTA     3
#define ENLEFTA     11

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

/*  Table of contents:
    1. INITIALISATION
    2. SETTINGS
    3. SETUP & LOOP
    4. MAIN FUNCTIONS
    5. ALIGNMENT
    6. SENSORS
    7. MOVEMENT
    
*/

/* ========================= INITIALISATION =======================================*/

SharpIR SharpIR1(IR1, sensor1);
SharpIR SharpIR2(IR2, sensor2);
SharpIR SharpIR3(IR3, sensor3);
SharpIR SharpIR4(IR4, sensor4);
SharpIR SharpIR5(IR5, sensor5);
SharpIR SharpIR6(IR6, sensor6);

DualVNH5019MotorShield md;

volatile unsigned int E1ticks = 0;
volatile unsigned int E2ticks = 0;
double E1ticksmoved = 0;
double E2ticksmoved = 0;
double M1speed = 0;//230
double M2speed = 0;//230
double invertedM1speed = 0;
double invertedM2speed = 0;

double Setpointr; // will be the desired value
double Setpointl;
int SampleTime = 50;

double Inputl; // left motor input
double Outputl ; // left motor output
double Inputr; // right motor input
double Outputr ; //right motor output
char lastRPIcommand;

double voltr =0;
double voltl =0;


double Kp = 1, Ki =0, Kd =0;
double Kp2 =1, Ki2=0, Kd2 =0;

//create PID instance
PID myPIDright(&Inputr, &Outputr, &Setpointr, Kp, Ki, Kd, DIRECT);
PID myPIDleft(&Inputl, &Outputl, &Setpointl, Kp2, Ki2, Kd2,DIRECT);


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

/* ================================ SETUP() AND LOOP() ======================================*/

void setup() {

  pinMode(ENRIGHTA, INPUT_PULLUP);
  pinMode(ENLEFTA, INPUT_PULLUP);

  enableInterrupt(ENRIGHTA, E1, RISING);
  enableInterrupt(ENLEFTA, E2, RISING);
  Serial.begin(115200); // the bigger number the better
  Serial.setTimeout(50);
  delay(1000);


  //initialize for pid
//  Inputr = E1ticks;
//  Inputl = E2ticks;
  Setpointr=220; //220//234//214//176
  Setpointl=240; //240//230//195
  
  //Turn the PID on
  myPIDright.SetMode(AUTOMATIC);
  myPIDleft.SetMode(AUTOMATIC);
  md.init();
              // a personal quirk
//  md.setM1Speed(M1speed);
//  md.setM2Speed(M2speed);
  myPIDright.SetTunings(Kp, Ki, Kd);
  myPIDleft.SetTunings(Kp, Ki, Kd);
  myPIDright.SetSampleTime(SampleTime);
  myPIDleft.SetSampleTime(SampleTime);
//  Serial.println("Setup Complete");
}

void loop() {
  //runTests();
  startListening();
}

/* ================================ MAIN FUNCTIONS ======================================*/

void startListening() {
      
  String RPIcommand;
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
      
      rotateLeft(DistNum);

      lastRPIcommand = 'a';
      done();
    }
    if (RPIcommand[0] == 'z') {
      sensorReading(DistNum);
      lastRPIcommand = 'z';
    }
    if (RPIcommand[0] == 'y') {
      rightSensorReading();
      lastRPIcommand = 'y';
    }
    if (RPIcommand[0] == 'g') {
      Setpointr=222;
      lastRPIcommand = 'g';
    }
    
  }
}


void runTests() {
  delay(5000);
  int  test = 4;
  while(test>0){
    moveForward(1);//comment out
    delay(1500);
    test--;
  }
  rotateLeft(2);
  delay(1500);
  test = 4;
  while(test>0){
    moveForward(1);//comment out
    delay(1500);
    test--;
  }
  
  rotateLeft(2);
  test = 4;
  while(test>0){
    moveForward(1);//comment out
    delay(1500);
    test--;
  }
 
  delay(1000);
  while(test<4){
    moveBackward(1);
    delay(1500);
    test++;
  }
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

/* ================================ ALIGNMENT ====================================== */

void frontAlignFastestPath(){
  float near;
  float aligned;
  float diff = sensortwo(true) - sensorthree(true);
  if(diff>0){
      while(1){
      aligned = sensortwo(true) - sensorthree(true);
      if(aligned<=0.5){
        break;
      }
      
      md.setSpeeds(-100,0);  
      delay(100);
      md.setSpeeds(0, 0);
      md.setBrakes(400, 400);
      delay(5);
    }
  }
  else if(diff<0){
    while(1){
      aligned = sensortwo(true) - sensorthree(true);
      if(aligned>=-0.5){
        break;
      }  
      md.setSpeeds(0,-100);  
      delay(100);
      md.setSpeeds(0, 0);
      md.setBrakes(400, 400);
      delay(5);
    }
  }

  float average = (sensortwo(true) + sensorthree(true))/2;
  if(average)<=8){
    if(average<=4.5){
      while(1){
        moveBackward(99);  
        near = ((sensortwo(true) + sensorthree(true))/2);
        if(near>=5){
          break;
        }
      }
    }else if(average>=5.5){
      while(1){
        moveForward(99);
        near = ((sensortwo(true) + sensorthree(true))/2);
        if(near<=5){
          break;
        }
      }
    }
  }else if((average>=9)&&(average<=18)){
    if(average<=14.5){
      while(1){
        moveBackward(99);  
        near = ((sensortwo(true) + sensorthree(true))/2);
        if(near>=15){
          break;
        }
      }
    }else if(average>=15.5){
      while(1){
        moveForward(99);
        near = ((sensortwo(true) + sensorthree(true))/2);
        if(near<=15){
          break;
        }
      }
    }
  }else if((average>=19)&&(average<=29)){
    if(average<=24.5){
      while(1){
        moveBackward(99);  
        near = ((sensortwo(true) + sensorthree(true))/2);
        if(near>=25){
          break;
        }
      }
    }else if(average>=25.5){
      while(1){
        moveForward(99);
        near = ((sensortwo(true) + sensorthree(true))/2);
        if(near<=25){
          break;
        }
      }
    }
  }
}
//TODO-IMPLEMENT 2x2 ALIGN. REQUIRES KEEPING TRACK OF PREV BLOCKS. REQUIRES 2 OUT OF 3 SENSORS. REMOVE FORWARD THRESHOLD
//REWRITE CHECK FRONT CHECK LEFT ALIGN

void checkLeftAlign() {
    boolean align;
    
    if (orientation % 2 == 0) {     //facing north or south
        if (hori_counter > alignThreshold) {
            align = true;
        } else { 
            hori_counter++;         
            align = false;
        }
    } else {
        if (vert_counter > alignThreshold) {
            align = true;
        } else { 
            vert_counter++;         
            align = false;
        }
    }
    //check left left and left right, 5 and 4
    if (align && sensorfive(false)<=10 && sensorfour(false)<=10) {
        wallAlign(&sensortwo, &sensorthree);                    //left and right
        resetLeftAlignCounter();
    }
    else if (align && lastRPIcommand == 'w'  && middleLeftisBlock && sensorfour(false)<=10) {       //2by2 align
        wallAlign(&sensorone, &sensorthree);                    //mid and right
        resetLeftAlignCounter();
    }
    
    
    //code to keep track of left blocks
    if (sensorfour(false)<=10) {
        middleLeftisBlock = true;
    } else {
        middleLeftisBlock = false;
    }
}

void checkFrontAlign() {
    boolean align;
    
    if (orientation % 2 == 0) {     //facing north or south
        if (vert_counter > alignThreshold) {
            align = true;
        } else { 
            vert_counter++;         
            align = false;
        }
    } else {
        if (hori_counter > alignThreshold) {
            align = true;
        } else { 
            hori_counter++;         
            align = false;
        }
    }
    //check left and right, 2 and 3
    if (align && sensortwo(false)<=10&& sensorthree(false)<=10) {
        alignFront(&sensortwo, &sensorthree);
        resetFrontAlignCounter();
    } else if (align && sensortwo(false)<=10&& sensorone(false)<=10) {      //check left and mid, 2 and 1
        alignFront(&sensortwo, &sensorone);                                 //order matters
        resetFrontAlignCounter();
    } else if align && sensorone(false)<=10&& sensorthree(false)<=10) {      //check mid and right, 1 and 3
        alignFront(&sensorone, &sensorthree);
        resetFrontAlignCounter();
    }
}

void resetFrontAlignCounter() {
  if (orientation % 2 == 0) {     //facing north or south
      vert_counter = 0;
  } else {
      hori_counter = 0;
  }
}

void resetLeftAlignCounter() {
    if (orientation % 2 == 0) {     //facing north or south
        hori_counter = 0;
    } else {
        vert_counter = 0;
    }
}

void alignLeft(){
   float rotate;
   float alignedl;
    rotate = sensorfive(true) - sensorfour(true);
    //rotate clockwise
    if(rotate>0){
      while(1){
      alignedl = sensorfive(true) - sensorfour(true);
      if(alignedl<=0.4){
        break;
      }  
      md.setSpeeds(0,100);  
      delay(100);
      md.setSpeeds(0, 0);
      md.setBrakes(400, 400);
      delay(5);
      if(sensorfive(true)>10||sensorfour(true)>10){
         break;
      }

    }
  }
  //rotate anti-clockwise
  else if(rotate<0){

      while(1){
      alignedl = sensorfive(true) - sensorfour(true);
      if(alignedl>=-0.4){
        break;
        }  
      md.setSpeeds(0,-100);  
      delay(100);
      md.setSpeeds(0, 0);
      md.setBrakes(400, 400);
      delay(5);
      if(sensorfive(true)>10||sensorfour(true)>10){
         break;
      }
      }
    }
    
}

void alignFront(void (*left)(boolean),void (*right)(boolean)){
  float near;
  float aligned;
  float diff = left(false) - right(false);
  //rotation
  if(diff>0){
      while(1){
      aligned = left(false) - right(false);
      if(aligned<=0.5){
        break;
      }
      
      md.setSpeeds(-100,0);  
      delay(100);
      md.setSpeeds(0, 0);
      md.setBrakes(400, 400);
      delay(5);
      
      if(left(true)>10||right(true)>10){
         break;
      }
    }
  }
  else if(diff<0){

      while(1){
      aligned = left(false) - right(false);
      if(aligned>=-0.5){
        break;
      }  
      md.setSpeeds(0,-100);  
      delay(100);
      md.setSpeeds(0, 0);
      md.setBrakes(400, 400);
      delay(5);

      if(left(true)>10||right(true)>10){
         break;
      }

    }
  }
  //displacement
  float average = (left(false) + right(false))/2;
  if(average<=4.5){
    while(1){
      moveBackward(99);  
      near = (left(true) + right(true))/2);
      if(near>=5){
        break;
      }
    }
  }else if(average>=5.5){
      while(1){
      moveForward(99);
      near = (left(true) + right(true))/2);
      if(near<=5){
        break;
      }
    }
  }
   
  //get new raw sensors readings
  resetSensorsReadings();
}

void wallAlign(void (*left)(boolean),void (*right)(boolean)){
  resetLeftAlignCounter();

  rotateLeft(2);
  alignFront(left, right);
  rotateRight(2);
  //alignLeft();        //removed as we are implementing 2-block alignment. If keeping, we have to check sensors are ok before alignment

  //take note of displacement after this alignment 


}

/* ================================ SENSORS ====================================== */
void resetSensorsReadings() {
    #signals that sensors should get raw readings next time.
    prevIR1reading = -1;
    prevIR2reading = -1;
    prevIR3reading = -1;
    prevIR4reading = -1;
    prevIR5reading = -1;
}

float sensorone(boolean useRaw){
  #if less than 0, it means get new sensor reading. 
  if (useRaw || prevIR1reading<0) {
      prevIR1reading = SharpIR1.distance() + 0.5;
  }
  
  return prevIR1reading;
}

float sensortwo(boolean useRaw){
  #if less than 0, it means get new sensor reading. 
  if (useRaw || prevIR2reading<0) {
      prevIR2reading = SharpIR2.distance() - 0.5;
  }
  
  return prevIR2reading;
}

float sensorthree(boolean useRaw){
  #if less than 0, it means get new sensor reading. 
  if (useRaw || prevIR3reading<0) {
      prevIR3reading = SharpIR3.distance() - 0.3;
  }
  
  return prevIR3reading;
}

float sensorfour(boolean useRaw){
  #if less than 0, it means get new sensor reading. 
  if (useRaw || prevIR4reading<0) {
      prevIR4reading = SharpIR4.distance() - 0.3;
  }
  
  return prevIR4reading;
}

float sensorfive(boolean useRaw){
  #if less than 0, it means get new sensor reading. 
  if (useRaw || prevIR5reading<0) {
      prevIR5reading = SharpIR5.distance() + 0.2;
  }
  
  return prevIR5reading;
}

void sensorReading(long Sensor) {
  String rawDistance = "";
  String blocks = "";
  int disR, disLTL, disLTR, disFR, disFL, disFM = 0;
  int blkR, blkLTL, blkLTR, blkFR, blkFL, blkFM = 0;
  float newdisFM =0;
  float newdisFL =0;
  float newdisFR =0;
  if (Sensor % 10 == 1) {
    disR = SharpIR6.distance(); // this returns the distance for Right
    blkR = rightCal(disR);
  } else {
    disR = -1;
    blkR = -1;
  }
  Sensor /= 10;
  if (Sensor % 10 == 1) {
    disLTL = sensorfive(false); // this returns the distance for Left Top Left
    blkLTL = leftCal(disLTL);
  } else {
    disLTL = -1;
    blkLTL = -1;
  }
  Sensor /= 10;
  if (Sensor % 10 == 1) {
    disLTR = sensorfour(false); // this returns the distance for Left Top Right
    blkLTR = leftCal(disLTR);
  } else {
    disLTR = -1;
    blkLTR = -1;
  }
  Sensor /= 10;
  if (Sensor % 10 == 1) {
    disFR = sensorthree(false); // this returns the distance for Front Right
    blkFR = frontCal(disFR);
  } else {
    blkFR = -1;
    disFR = -1;
  }
  Sensor /= 10;
  if (Sensor % 10 == 1) {
    disFL = sensortwo(false); // this returns the distance for Front Left
    blkFL = frontCal(disFL);
  } else {
    disFL = -1;
    blkFL = -1;
  }
  Sensor /= 10;
  if (Sensor % 10 == 1) {
    disFM = sensorone(false); // this returns the distance for Front Middle
    blkFM = frontMidCal(disFM);
  } else {
    disFM = -1;
    blkFM = -1;
  }
  Sensor /= 10;

  rawDistance.concat(disFM);
  rawDistance.concat(",");
  rawDistance.concat(disFL);
  rawDistance.concat(",");
  rawDistance.concat(disFR);
  rawDistance.concat(",");
  rawDistance.concat(disLTR);
  rawDistance.concat(",");
  rawDistance.concat(disLTL);
  rawDistance.concat(",");
  rawDistance.concat(disR);

  blocks.concat(blkFM);
  blocks.concat(",");
  blocks.concat(blkFL);
  blocks.concat(",");
  blocks.concat(blkFR);
  blocks.concat(",");
  blocks.concat(blkLTR);
  blocks.concat(",");
  blocks.concat(blkLTL);
  blocks.concat(",");
  blocks.concat(blkR);

//  Serial.println(rawDistance);
  Serial.println(blocks);

}
void rightSensorReading(){
  String blocks = "";
  int disRM=SharpIR6.distance();  // this returns the distance for Left Top Left
  int blkRM=rightCal(disRM);
  

  blocks.concat(blkRM);

  Serial.println(blocks);
}

int rightCal(int receiveDFR) {
  int sentBFR = -1;
  if( receiveDFR<=16){
    sentBFR =-1;
  }
  else if (receiveDFR >= 17 && receiveDFR <= 27) {
    sentBFR = 2;
  }
  else if (receiveDFR >= 26 && receiveDFR <= 37) {
    sentBFR = 3;
  }
  else if (receiveDFR >= 36 && receiveDFR <= 47) {
    sentBFR = 4;
  }
  else if (receiveDFR >= 48) {
    sentBFR = 5;
  }
  return sentBFR;
}

int frontCal(int receiveDFR) {
  int sentBFR = -1;
  if( receiveDFR<=8){
    sentBFR =0;
  }
  else if (receiveDFR >= 9 && receiveDFR <= 17) { //16
    sentBFR = 1;
  }
  else if (receiveDFR >= 18 && receiveDFR <= 26) {//17
    sentBFR = 2;
  }
  else if (receiveDFR >= 27) {
    sentBFR = 3;
  }
  return sentBFR;
}


int frontMidCal(int receiveDFR) {
  int sentBFR = -1;
  if( receiveDFR<=8){
    sentBFR =0;
  }
  else if (receiveDFR >= 9 && receiveDFR <= 17) {//16
    sentBFR = 1;
  }
  else if (receiveDFR >= 18 && receiveDFR <= 26) {//17
    sentBFR = 2;
  }
  else if (receiveDFR >= 27) {
    sentBFR = 3;
  }
  return sentBFR;
}

int leftCal(int receiveDFL) {
  int sentBFR = -1;
  if( receiveDFL<=8){
    sentBFR =0;
  }
  else if (receiveDFL >= 9 && receiveDFL <= 17) {
    sentBFR = 1;
  }
  else if (receiveDFL >= 18 && receiveDFL <= 26) {
    sentBFR = 2;
  }
  else if (receiveDFL >= 27) {
    sentBFR = 3;
  }
  return sentBFR;
}

/* ================================ MOVEMENT ====================================== */

void PIDController(int directionflag) {
//  voltr = (2.9090 * (((E1ticks / 562.25)/0.05) * 60) + 18.907)*51/80;
//  voltr = (2.9248 * (((E1ticks / 562.25)/0.05) * 60) + 26.889)*51/80; 
//  voltl = (2.9248 * (((E2ticks / 562.25)/0.05) * 60) + 26.889)*51/80;
    voltr = (2.639 * (((E1ticks / 562.25)/0.05) * 60) + 9.833)*51/80; 
    voltl = (2.643 * (((E2ticks / 562.25)/0.05) * 60) + 24.21)*51/80;
  Inputr = voltr;
  Inputl = voltl;
  myPIDleft.Compute();
  myPIDright.Compute();
  //change values of a and b for PWM
  M1speed = (Outputr*80)/51;
  M2speed = (Outputl*80)/51;
  invertedM1speed = -M1speed;
  invertedM2speed = -M2speed;
  if (directionflag == 1) {
    md.setSpeeds(M1speed, M2speed);
  }
  else if (directionflag == 2) {
    md.setSpeeds(invertedM1speed, invertedM2speed);
  }
  else if (directionflag == 3) {
    md.setSpeeds(M1speed, invertedM2speed);
  }
  else if (directionflag == 4) {
    md.setSpeeds(invertedM1speed, M2speed);
  }
  E1ticksmoved += E1ticks;
  E2ticksmoved += E2ticks;
  E1ticks = 0;
  E2ticks = 0;
}

void moveForward(int blockstomove) {
  E1ticks = 0;
  E2ticks = 0;
  E1ticksmoved = 0;
  E2ticksmoved = 0;
  md.setBrakes(0, 0);
//  initializeTick();
  initializeMotor_Start();
  int tickstomove;
  switch (blockstomove) {
    case 1:
      {
        if (lastRPIcommand == 'a' || lastRPIcommand == 'd') {
          tickstomove = 279;//280
        }
        else {
          tickstomove = 279;//280
        }
        break;
      }
    case 2:
      {
        tickstomove = 540;
        break;
      }
    case 3:
      {
        tickstomove = 830;
        break;
      }
    case 4:
      {
        tickstomove = 1130;
        break;
      }
    case 5:
      {
        tickstomove = 1410;
        break;
      }
    case 6:
      {
        tickstomove = 1720;
        break;
      }
    case 7:
      {
        tickstomove = 2030;
        break;
      }
    case 8:
      {
        tickstomove = 2310;
        break;
      }
    case 9:
      {
        tickstomove = 2600;
        break;
      }
    case 10:
      {
        tickstomove = 2930;
        break;
      }
    case 11:
      {
        tickstomove = 3230;
        break;
      }
    case 12:
      {
        tickstomove = 3500;
        break;
      }
    case 13:
      {
        tickstomove = 3800;
        break;
      }
    case 14:
      {
        tickstomove = 4120;
        break;
      }
    case 15:
      {
        tickstomove = 4350;
        break;
      }
    case 99:
    {
        tickstomove = 13;
        break;
  }
  }

  while (1) {
    
    if (E1ticksmoved > tickstomove || E2ticksmoved > tickstomove) {
        md.setSpeeds(0, 0);
        md.setBrakes(400, 400);
        delay(5);
      break;
    }
    PIDController(1);
//    Serial.println(M1speed);
//    Serial.print("M2Speed: ");
//    Serial.println(M2speed);
  }
  
  //get new raw sensors readings
  resetSensorsReadings();
}




void moveBackward(int blockstomove) {
  E1ticks = 0;
  E2ticks = 0;
  E1ticksmoved = 0;
  E2ticksmoved = 0;
  md.setBrakes(0, 0);
//  initializeTick();
  initializeMotor_Start();
  int tickstomove;
  switch (blockstomove) {
    case 1:
      {
        if (lastRPIcommand == 'a' || lastRPIcommand == 'd') {
          tickstomove = 269;//maybe want to increase this value 
        }
        else {
          tickstomove = 269;
        }
        break;
      }
    case 2:
      {
        tickstomove = 540;
        break;
      }
    case 3:
      {
        tickstomove = 830;
        break;
      }
    case 4:
      {
        tickstomove = 1130;
        break;
      }
    case 5:
      {
        tickstomove = 1410;
        break;
      }
    case 6:
      {
        tickstomove = 1720;
        break;
      }
    case 7:
      {
        tickstomove = 2030;
        break;
      }
    case 8:
      {
        tickstomove = 2310;
        break;
      }
    case 9:
      {
        tickstomove = 2600;
        break;
      }
    case 10:
      {
        tickstomove = 2930;
        break;
      }
    case 11:
      {
        tickstomove = 3230;
        break;
      }
    case 12:
      {
        tickstomove = 3500;
        break;
      }
    case 13:
      {
        tickstomove = 3800;
        break;
      }
    case 14:
      {
        tickstomove = 4120;
        break;
      }
    case 15:
      {
        tickstomove = 4350;
        break;
      }
    case 99:
    {
        tickstomove = 10;
        break;
  }
  }
  while (1) {
    if (E1ticksmoved > tickstomove || E2ticksmoved > tickstomove) {
        md.setSpeeds(0, 0);
        md.setBrakes(400, 400);
        delay(5);
//      md.setBrakes(0, 0);
//      delay(100);
      break;
    }
    PIDController(2);
  }
  
  //get new raw sensors readings
  resetSensorsReadings();
}
void rotateLeft(int degreetomove) {
  orientation = orientation + 3 % 4;
  vert_counter+=2;
  hori_counter+=2;
  E1ticks = 0;
  E2ticks = 0;  
  E1ticksmoved = 0;
  E2ticksmoved = 0;
  md.setBrakes(0, 0);
//  initializeTick();
  initializeMotor_Start();
  int tickstomove = 0;
  if (degreetomove == 1) {
    tickstomove = 105; //45 degrees
  }
  else if (degreetomove == 2) {
      tickstomove = 369;//377 //90 degrees
  }
  else if (degreetomove == 3) {//270
    tickstomove = 720; //180 degrees
  }
  while (1) {
    if (E1ticksmoved > tickstomove || E2ticksmoved > tickstomove) {
        md.setSpeeds(0, 0);
        md.setBrakes(400, 400);
        delay(5);
//      md.setBrakes(0, 0);
//      delay(100);
      int correction1 = E1ticksmoved - tickstomove; //correction code, maybe want to add it in
      int correction2 = E2ticksmoved - tickstomove;
      break;
    }
    PIDController(3);
  }
  
  //get new raw sensors readings
  resetSensorsReadings();
}

void rotateRight(int degreetomove) {
  orientation = orientation + 1 % 4;
  vert_counter+=1;
  hori_counter+=1;
  E1ticks = 0;
  E2ticks = 0;
  E1ticksmoved = 0;
  E2ticksmoved = 0;
  md.setBrakes(0, 0);
//  initializeTick();
  initializeMotor_Start();
  int tickstomove = 0;
  if (degreetomove == 1) {
    tickstomove = 105;//45 degrees
  }
  else if (degreetomove == 2) {
    tickstomove = 365;//366 //90 degrees
  }
  else if (degreetomove == 3) {
    tickstomove = 720; //180 degrees
  }
  while (1) {
    if (E1ticksmoved > tickstomove || E2ticksmoved > tickstomove) {
        md.setSpeeds(0, 0);
        md.setBrakes(400, 400);
        delay(5);
//      md.setBrakes(0, 0);
//      delay(100);
      break;
    }
    PIDController(4);
  }
  
  //get new raw sensors readings
  resetSensorsReadings();
  
}

void E1() {
  E1ticks++;
}

void E2() {
  
  E2ticks++;
}
void initializeTick() {
  Inputr = 0;
  Outputr = 0;
  Setpointr = 0;
  Inputl = 0;
  Outputl = 0;
  Setpointl = 0;
}

void initializeMotor_Start() {
  md.setSpeeds(0, 0);
  md.setBrakes(0, 0);
}



