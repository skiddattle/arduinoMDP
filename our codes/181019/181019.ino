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
int alignThreshold = 6;//4

int forwardThreshold = 5;
int consecForwardcounter = 0;


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
//delay(5000);
//int  test = 4;
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
//  rotateLeft(2);
//  test = 4;
//  while(test>0){
//    moveForward(1);//comment out
//    delay(1500);
//    test--;
//  }
  
////  
//  delay(1000);
//  while(test<4){
//    moveBackward(1);
//    delay(1500);
//    test++;
//  }
//
  
  
  String RPIcommand;
  long int DistNum;
  while (Serial.available()) {
    RPIcommand = readStr();
    DistNum = RPIcommand.substring(1).toInt();
    if (RPIcommand[0] == 'w') {
      consecForwardcounter++;
      moveForward(DistNum);
        if(consecForwardcounter>= forwardThreshold && SharpIR4.distance()<=10&&SharpIR5.distance()<=10){
          wallAlign();
          consecForwardcounter =0;
        }
        if (checkFrontAlign()) {
            alignFront();
         }
//      counter++;
//      if(abs(SharpIR5.distance()-SharpIR4.distance())>=1.5 && SharpIR4.distance()<=10&&SharpIR5.distance()<=10){
//        wallAlign();
//        counter=0;
//      }else if(counter>=3&&SharpIR4.distance()<=10&&SharpIR5.distance()<=10){
//        wallAlign();
//        counter=0;        
//      }
//      if(SharpIR2.distance()<=9&&SharpIR3.distance()<=9){
//        alignFront();
//      }
      lastRPIcommand = 'w';
      done();
    }
    if (RPIcommand[0] == 'd') {
      consecForwardcounter = 0;
      vert_counter += 10;
      hori_counter += 10;
      if (checkLeftAlign()) {
        wallAlign();
       }
      if (checkFrontAlign()) {
          alignFront();
      }
      rotateRight(DistNum);
      vert_counter = 0;
      hori_counter = 0;

//      counter++;
//      if(SharpIR4.distance()<=10&&SharpIR5.distance()<=10){
//        alignLeft();
//      }
//      if(counter>=3&&SharpIR4.distance()<=10&&SharpIR5.distance()<=10){
//        wallAlign();
//        counter=0;        
//      }
//      if(SharpIR2.distance()<=9&&SharpIR3.distance()<=9){
//        alignFront();
//      }
      lastRPIcommand = 'd';
      done();
      
    }
    if (RPIcommand[0] == 's') {
        moveBackward(DistNum);
        lastRPIcommand = 's';
      done();
      
    }
    if (RPIcommand[0] == 'a') {
      consecForwardcounter = 0;
      vert_counter++;
      hori_counter++;
      rotateLeft(DistNum);
//      counter++;
//      if(SharpIR4.distance()<=10&&SharpIR5.distance()<=10){
//        alignLeft();
//      }
//      if(counter>=3&&SharpIR4.distance()<=10&&SharpIR5.distance()<=10){
//        wallAlign();
//        counter=0;        
//      }
//      if(SharpIR2.distance()<=9&&SharpIR3.distance()<=9){
//        alignFront();
//      }
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

void frontAlignFastestPath(){
  float near;
  float aligned;
  float  diff = SharpIR2.distance() - SharpIR3.distance();
  if(diff>0){
      while(1){
      aligned = SharpIR2.distance() - SharpIR3.distance();
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
      aligned = SharpIR2.distance() - SharpIR3.distance();
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

  if((SharpIR2.distance() + SharpIR3.distance()/2)<=8){
  
      if((SharpIR2.distance() + SharpIR3.distance()/2)<=4.5){
          while(1){
          moveBackward(99);  
          near = ((SharpIR2.distance() + SharpIR3.distance())/2);
          if(near>=5){
            break;
          }
         }
        }else if((SharpIR2.distance() + SharpIR3.distance()/2)>=5.5){
          while(1){
          moveForward(99);
          near = ((SharpIR2.distance() + SharpIR3.distance())/2);
          if(near<=5){
            break;
          }
        }
       }
  }else if(((SharpIR2.distance() + SharpIR3.distance()/2)>=9)&&((SharpIR2.distance() + SharpIR3.distance()/2)<=18)){
          if((SharpIR2.distance() + SharpIR3.distance()/2)<=14.5){
          while(1){
          moveBackward(99);  
          near = ((SharpIR2.distance() + SharpIR3.distance())/2);
          if(near>=15){
            break;
          }
         }
        }else if((SharpIR2.distance() + SharpIR3.distance()/2)>=15.5){
          while(1){
          moveForward(99);
          near = ((SharpIR2.distance() + SharpIR3.distance())/2);
          if(near<=15){
            break;
          }
        }
       }
  }else if(((SharpIR2.distance() + SharpIR3.distance()/2)>=19)&&((SharpIR2.distance() + SharpIR3.distance()/2)<=29)){
          if((SharpIR2.distance() + SharpIR3.distance()/2)<=24.5){
          while(1){
          moveBackward(99);  
          near = ((SharpIR2.distance() + SharpIR3.distance())/2);
          if(near>=25){
            break;
          }
         }
        }else if((SharpIR2.distance() + SharpIR3.distance()/2)>=25.5){
          while(1){
          moveForward(99);
          near = ((SharpIR2.distance() + SharpIR3.distance())/2);
          if(near<=25){
            break;
          }
        }
      }
  }
}


boolean checkLeftAlign() {
    
    if (orientation % 2 == 0) {     //facing north or south
        if (hori_counter > alignThreshold && SharpIR4.distance()<=10&&SharpIR5.distance()<=10) {
            return true;
        } else { 
            hori_counter++;         
            return false;
        }
    } else {
        if (vert_counter > alignThreshold && SharpIR4.distance()<=10&&SharpIR5.distance()<=10) {
            return true;
        } else { 
            vert_counter++;         
            return false;
        }
    }
}

boolean checkFrontAlign() {
    if (orientation % 2 == 0) {     //facing north or south
        if (vert_counter > alignThreshold && SharpIR2.distance()<=10&&SharpIR3.distance()<=10) {
            return true;
        } else { 
            vert_counter++;         
            return false;
        }
    } else {
        if (hori_counter > alignThreshold &&SharpIR2.distance()<=10&&SharpIR3.distance()<=10) {
            return true;
        } else { 
            hori_counter++;         
            return false;
        }
    }
}

void alignLeft(){
   float rotate;
   float alignedl;
    rotate = SharpIR5.distance() - SharpIR4.distance();
    if(rotate>0){
      while(1){
      alignedl = SharpIR5.distance() - SharpIR4.distance();
      if(alignedl<=0.4){
        break;
      }  
      md.setSpeeds(0,100);  
      delay(100);
      md.setSpeeds(0, 0);
      md.setBrakes(400, 400);
      delay(5);
      if(SharpIR5.distance()>10||SharpIR4.distance()>10){
         break;
      }

    }
  }
  else if(rotate<0){

      while(1){
      alignedl = SharpIR5.distance() - SharpIR4.distance();
      if(alignedl>=-0.4){
        break;
        }  
      md.setSpeeds(0,-100);  
      delay(100);
      md.setSpeeds(0, 0);
      md.setBrakes(400, 400);
      delay(5);
      if(SharpIR5.distance()>10||SharpIR4.distance()>10){
         break;
      }
      }
    }
    
}

void alignFront(){
  if (orientation % 2 == 0) {     //facing north or south
      vert_counter = 0;
  } else {
      hori_counter = 0;
  }
  float near;
  float aligned;
  float  diff = SharpIR2.distance() - SharpIR3.distance();
  if(diff>0){
      while(1){
      aligned = SharpIR2.distance() - SharpIR3.distance();
      if(aligned<=0.5){
        break;
      }
      
      md.setSpeeds(-100,0);  
      delay(100);
      md.setSpeeds(0, 0);
      md.setBrakes(400, 400);
      delay(5);
      if(SharpIR2.distance()>10||SharpIR3.distance()>10){
         break;
      }


    }
  }
  else if(diff<0){

      while(1){
      aligned = SharpIR2.distance() - SharpIR3.distance();
      if(aligned>=-0.5){
        break;
      }  
      md.setSpeeds(0,-100);  
      delay(100);
      md.setSpeeds(0, 0);
      md.setBrakes(400, 400);
      delay(5);

      if(SharpIR2.distance()>10||SharpIR3.distance()>10){
         break;
      }

    }
  }

    if((SharpIR2.distance() + SharpIR3.distance()/2)<=4.5){
      while(1){
      moveBackward(99);  
      near = ((SharpIR2.distance() + SharpIR3.distance())/2);
      if(near>=5){
        break;
      }
     }
    }else if((SharpIR2.distance() + SharpIR3.distance()/2)>=5.5){
      while(1){
      moveForward(99);
      near = ((SharpIR2.distance() + SharpIR3.distance())/2);
      if(near<=5){
        break;
      }
    }
   }
}

void wallAlign(){
    if (orientation % 2 == 0) {     //facing north or south
        hori_counter = 0;
    } else {
        vert_counter = 0;
    }
//  alignLeft();
  rotateLeft(2);
  alignFront();
  rotateRight(2);
  alignLeft();

  //take note of displacement after this alignment 


}

//int sensorone(){
//  return SharpIR1.distance() +0.5;
//}
//
//int sensortwo(){
//  return SharpIR2.distance() -0.5;
//}
//
//int sensorthree(){
//  return SharpIR3.distance() -0.3;
//}
//
//int sensorfour(){
//  return SharpIR4.distance() -0.3;
//}
//
//int sensorfive(){
//  return SharpIR5.distance() + 0.2;
//}

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
    disLTL = SharpIR5.distance(); // this returns the distance for Left Top Left
    blkLTL = leftCal(disLTL);
  } else {
    disLTL = -1;
    blkLTL = -1;
  }
  Sensor /= 10;
  if (Sensor % 10 == 1) {
    disLTR = SharpIR4.distance(); // this returns the distance for Left Top Right
    blkLTR = leftCal(disLTR);
  } else {
    disLTR = -1;
    blkLTR = -1;
  }
  Sensor /= 10;
  if (Sensor % 10 == 1) {
    disFR = SharpIR3.distance(); // this returns the distance for Front Right
    blkFR = frontCal(disFR);
  } else {
    blkFR = -1;
    disFR = -1;
  }
  Sensor /= 10;
  if (Sensor % 10 == 1) {
    disFL = SharpIR2.distance(); // this returns the distance for Front Left
    blkFL = frontCal(disFL);
  } else {
    disFL = -1;
    blkFL = -1;
  }
  Sensor /= 10;
  if (Sensor % 10 == 1) {
    disFM = SharpIR1.distance(); // this returns the distance for Front Middle
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

void done() {
  Serial.println("done");
}
